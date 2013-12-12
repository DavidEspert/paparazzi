# Hey Emacs, this is a -*- makefile -*-

telemetry_CFLAGS = -DUSE_$(MODEM_PORT)
telemetry_CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
telemetry_CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
telemetry_srcs = subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c

# GJN Addition during development...
#transmition transport layers enabled
telemetry_CFLAGS += -DTRANSPORT_TX_1=PPRZ
telemetry_CFLAGS += -DTRANSPORT_TX_2=PPRZ
#Reception transport layers enabled
telemetry_CFLAGS += -DTRANSPORT_RX_1=PPRZ
telemetry_CFLAGS += -DTRANSPORT_RX_2=PPRZ
#Downlink
telemetry_CFLAGS += -DDOWNLINK
telemetry_CFLAGS += -DDOWNLINK_DEVICE=SIM_UART
# telemetry_CFLAGS += -DDOWNLINK_TRANSPORT=TRANSPORT_TX_2
telemetry_CFLAGS += -DDATALINK
telemetry_CFLAGS += -DDATALINK_DEVICE_1=SIM_UART
telemetry_CFLAGS += -DDATALINK_TRANSPORT=TRANSPORT_RX_2
# telemetry_CFLAGS += -DDATALINK_DEVICE_2=UART1
# telemetry_CFLAGS += -DDATALINK_TRANSPORT="TRANSPORT_RX_2"
# telemetry_srcs += mcu_periph/transmit_buffer.c subsystems/datalink/transport_pprz.c
# telemetry_CFLAGS += -DUSE_UART0 -DDOWNLINK_FBW_DEVICE=UART0 -DDOWNLINK_AP_DEVICE=UART0 -DPPRZ_UART=UART0 -DUART0_BAUD=B9600
# telemetry_srcs += mcu_periph/device_uart.c


ap.CFLAGS += $(telemetry_CFLAGS)
ap.srcs += $(telemetry_srcs) $(SRC_FIRMWARE)/datalink.c

fbw.CFLAGS += $(telemetry_CFLAGS)
fbw.srcs += $(telemetry_srcs)

# ap.CFLAGS += -DDOWNLINK -DDOWNLINK_FBW_DEVICE=$(MODEM_PORT) -DDOWNLINK_AP_DEVICE=$(MODEM_PORT) -DPPRZ_UART=$(MODEM_PORT)
# ap.CFLAGS += -DDOWNLINK_TRANSPORT=PprzTransport -DDATALINK=PPRZ
# ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c
# ap.srcs += $(SRC_FIRMWARE)/datalink.c

# GJN Addition during development...
ap.srcs += subsystems/datalink/device_uart.c subsystems/datalink/datalink.c
ap.srcs += mcu_periph/transmit_queue.c mcu_periph/dynamic_buffer.c subsystems/datalink/transport_pprz.c
#... end of GJN Addition
