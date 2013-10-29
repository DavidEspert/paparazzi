# Hey Emacs, this is a -*- makefile -*-


ap.CFLAGS += -DUSE_$(MODEM_PORT)
ap.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)

ap.CFLAGS += -DDOWNLINK -DDOWNLINK_FBW_DEVICE=$(MODEM_PORT) -DDOWNLINK_AP_DEVICE=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
ap.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
ap.srcs += subsystems/datalink/downlink.c mcu_periph/device.c mcu_periph/transmit_buffer.c subsystems/datalink/transport_pprz.c subsystems/datalink/pprz_transport.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c

# GJN Addition during development...
ap.srcs += mcu_periph/transmit_buffer.c subsystems/datalink/transport_pprz.c
# ap.CFLAGS += -DUSE_UART0 -DDOWNLINK_FBW_DEVICE=UART0 -DDOWNLINK_AP_DEVICE=UART0 -DPPRZ_UART=UART0 -DUART0_BAUD=B9600
# ap.srcs += mcu_periph/device_uart.c
ap.CFLAGS += -DUSE_I2C1 -DDOWNLINK_FBW_DEVICE=I2C1 -DDOWNLINK_AP_DEVICE=I2C1 -DPPRZ_UART=I2C1 -DI2C1_BAUD=B9600
ap.srcs += mcu_periph/device_i2c.c
#... end of GJN Addition
