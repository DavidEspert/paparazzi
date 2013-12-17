# Hey Emacs, this is a -*- makefile -*-

telemetry_CFLAGS = -DUSE_$(MODEM_PORT)
telemetry_CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

#Common files for both Downlink and Datalink
  #devices & transports
telemetry_srcs = subsystems/datalink/device_uart.c
telemetry_srcs += subsystems/datalink/transport_pprz.c subsystems/datalink/transport_xbee.c



#--- DOWNLINK ---
telemetry_CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
  #specific Downlink required files
telemetry_srcs += subsystems/datalink/downlink.c mcu_periph/dynamic_buffer.c
  #TX transport layers enabled
telemetry_CFLAGS += -DTRANSPORT_TX_1=PPRZ
telemetry_CFLAGS += -DTRANSPORT_TX_2=XBEE -DXBEE_BAUD=B9600
  #Downlink transport layer (fixed now but maybe modifiable in future)
telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=TRANSPORT_TX_1
# telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport



#--- UPLINK ---
telemetry_CFLAGS += -DDATALINK -DDATALINK_DEVICE=UART0
  #specific Uplink required files
telemetry_srcs += subsystems/datalink/datalink.c $(SRC_FIRMWARE)/datalink.c
  #RX transport layers enabled
telemetry_CFLAGS += -DTRANSPORT_RX_PPRZ_1
telemetry_CFLAGS += -DTRANSPORT_RX_PPRZ_2
telemetry_CFLAGS += -DTRANSPORT_RX_XBEE_1
  #Uplinktransport layer
telemetry_CFLAGS += -DDATALINK_TRANSPORT=XBEE_1


# Add telemetry to Ap and FBW
ap.CFLAGS += $(telemetry_CFLAGS)
ap.srcs += $(telemetry_srcs)

fbw.CFLAGS += $(telemetry_CFLAGS)
fbw.srcs += $(telemetry_srcs)

