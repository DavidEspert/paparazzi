#include "generated/airframe.h" // AC_ID is required
#include "device.h"
#include "mcu_periph/dynamic_buffer.h"
#include <string.h> //required for memcpy
#include "mcu_periph/sys_time_arch.h"

#define XBEE_MY_ADDR AC_ID

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"

/** Initialisation in API mode and setting of the local address
 * FIXME: busy wait */
#if XBEE_BAUD == B9600
    #define XBEE_BAUD_ALTERNATE B57600
    #define XBEE_ATBD_CODE "ATBD3\rATWR\r"
    #pragma message "Experimental: XBEE-API@9k6 auto-baudrate 57k6 -> 9k6 (stop ground link for correct operation)"
#elif XBEE_BAUD == B57600
    #define XBEE_BAUD_ALTERNATE B9600
    #define XBEE_ATBD_CODE "ATBD6\rATWR\r"
    #pragma message "Experimental: XBEE-API@57k6 auto-baudrate 9k6 -> 57k6 (stop ground link for correct operation)"
#else
    #warning XBEE-API Non default baudrate: auto-baud disabled
#endif

extern void xbee_init_callback(void *slot_p);

#define XBEE_CONFIG_PRIORITY 0
static inline void XBeeTransport_send_data(struct device* dev, uint8_t *data, uint8_t data_len) {
  uint8_t dev_slot;
  uint8_t buff_slot;
  uint8_t ta_len =    dev->api.transaction_len;

  /* 1.- try to get a device's 'transaction' slot */
  if(dev->api.check_free_space(dev->data, &dev_slot)){
    /* 2.- try to get a slot in dynamic buffer */
    if(dynamic_buffer_check_free_space(&dynamic_buff, (ta_len + data_len), &buff_slot)){
      /* 3.- get buffer pointer */
      uint8_t *buff = dynamic_buffer_get_slot_pointer(&dynamic_buff, buff_slot);

      /* SET TRANSACTION: CONTAINS DATA POINTER, LENGTH AND CALLBACK */
      /* 4.- set transaction in buffer */
      dev->api.transaction_pack(buff, (buff + ta_len), (data_len), NULL, 0, &xbee_init_callback);

      /* SET DATA */
      /* 5.- set data in buffer */
      memcpy((buff + ta_len), data, data_len);

      /* SUMMIT TRANSACTION */
      /* 6.- send data */
      dev->api.transaction_summit(dev->data, dev_slot, buff, XBEE_CONFIG_PRIORITY);
    }
    else {
      /* 7.- release device's slot */
      dev->api.free_space(dev->data, dev_slot);
    }
  }
}

static inline uint8_t XBeeTransport_print_string(char* s, uint8_t* buff) {
  uint8_t i = 0;
  while (s[i]) {
    buff[i] = (uint8_t) (s[i]);
    i++;
  }
  return i;
}

static inline uint8_t XBeeTransport_print_hex(uint8_t* data, uint8_t data_len, uint8_t* buff) {
  const uint8_t hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
//   uint8_t high;
//   uint8_t low;

  for (uint8_t i = 0; i < data_len; i++) {
//     high = (data[i] & 0xF0)>>4;
//     low  = data[i] & 0x0F;
    buff[(2*i)] =   hex[ ((data[i] & 0xF0)>>4) ];
    buff[(2*i+1)] = hex[  (data[i] & 0x0F)     ];
  }
  return (2*data_len); // CAUTION: if data_len >= 128 output will be truncated.
}

static inline uint8_t xbee_text_reply_is_ok(struct device* rx_dev) {
  char c[2];
  int count = 0;

//   while (TransportLink(XBEE_UART,ChAvailable()))
  while (rx_dev->api.byte_available(rx_dev->data))
  {
    char cc = rx_dev->api.get_byte(rx_dev->data);
    if (count < 2)
      c[count] = cc;
    count++;
  }

  if ((count > 2) && (c[0] == 'O') && (c[1] == 'K'))
    return TRUE;

  return FALSE;
}

static inline uint8_t xbee_try_to_enter_api(struct device* rx_dev) {

  /** Switching to AT mode (FIXME: busy waiting) */
//   XBeePrintString(XBEE_UART,AT_COMMAND_SEQUENCE);

  char command_seq[4] = AT_COMMAND_SEQUENCE;
  XBeeTransport_send_data(rx_dev, (uint8_t*) command_seq, 3);

  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  return xbee_text_reply_is_ok(rx_dev);
}

static inline void xbee_config_link(struct device* dev) {
  uint8_t buff[128];
  uint16_t offset = 0;
  char eol[3] = "\r";

  // Empty buffer before init process
//   while (TransportLink(XBEE_UART,ChAvailable()))
//     TransportLink(XBEE_UART,Getch());
  while(dev->api.byte_available(dev->data))
    dev->api.get_byte(dev->data);

#ifndef NO_XBEE_API_INIT
  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  if (! xbee_try_to_enter_api(dev) )
  {
    #ifdef XBEE_BAUD_ALTERNATE

      // Badly configured... try the alternate baudrate:
//       XBeeUartSetBaudrate(XBEE_BAUD_ALTERNATE);
      dev->api.set_baudrate(dev->data, XBEE_BAUD_ALTERNATE);
      if ( xbee_try_to_enter_api(dev) )
      {
        // The alternate baudrate worked,
//         XBeePrintString(XBEE_UART,XBEE_ATBD_CODE);
        char atbd[14] = XBEE_ATBD_CODE;
        offset += XBeeTransport_print_string(atbd, &buff[offset]);
      }
      else
      {
        // Complete failure, none of the 2 baudrates result in any reply
        // TODO: set LED?

        // Set the default baudrate, just in case everything is right
//         XBeeUartSetBaudrate(XBEE_BAUD);
//         XBeePrintString(XBEE_UART,"\r");
        dev->api.set_baudrate(dev->data, XBEE_BAUD);
        offset += XBeeTransport_print_string(eol, &buff[offset]);
      }

    #endif
    // Continue changing settings until the EXIT is issued.
  }

  /** Setting my address */
//   XBeePrintString(XBEE_UART,AT_SET_MY);
//   uint16_t addr = XBEE_MY_ADDR;
//   XBeePrintHex16(XBEE_UART,addr);
//   XBeePrintString(XBEE_UART,"\r");
// 
//   XBeePrintString(XBEE_UART,AT_AP_MODE);

  char at_set_my[5] = AT_SET_MY;
  uint16_t addr = XBEE_MY_ADDR;
  char at_ap_mode[8] = AT_AP_MODE;
  offset += XBeeTransport_print_string(at_set_my, &buff[offset]);
  offset += XBeeTransport_print_hex((uint8_t*) &addr, 2, &buff[offset]);
  offset += XBeeTransport_print_string(eol, &buff[offset]);

  offset += XBeeTransport_print_string(at_ap_mode, &buff[offset]);

#ifdef XBEE_INIT
//   XBeePrintString(XBEE_UART,XBEE_INIT);
  char init[32] = XBEE_INIT;
  offset += XBeeTransport_print_string(init, &buff[offset]);
#endif

  /** Switching back to normal mode */
//   XBeePrintString(XBEE_UART,AT_EXIT);
  char at_exit[7] = AT_EXIT;
  offset += XBeeTransport_print_string(at_exit, &buff[offset]);

  XBeeTransport_send_data(dev, buff, offset);

//   XBeeUartSetBaudrate(XBEE_BAUD);
  sys_time_usleep(0050000); //wait 0.05s before switch back (data has to be transmitted)
  dev->api.set_baudrate(dev->data, XBEE_BAUD);

#endif
}
// #define XBeeUartSetBaudrate(_a) TransportLink(XBEE_UART,SetBaudrate(_a))
// #define XBeePrintString(_dev, s) TransportLink(_dev,PrintString(s))
// #define XBeePrintHex16(_dev, x) TransportLink(_dev,PrintHex16(x))

