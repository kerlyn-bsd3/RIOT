# 6LoBAC 6LBR: MS/TP (RFC 8163) downstream <-> SLIP or USB CDC-ECM uplink to a Linux host.
APPLICATION = mstp_6lbr

BOARD ?= nrf52840dk
RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# --- MS/TP downstream (the clean-room driver) ---
USEMODULE += mstp
USEMODULE += iolist

# --- uplink: UPLINK=slip (default) or UPLINK=cdc-ecm ---
#  slip:    slipdev on UART_DEV(0) = nRF52840-DK J-Link VCP; host runs
#           dist/tools/sliptty/start_network.sh at the same SLIP_BAUDRATE.
#  cdc-ecm: USB Ethernet (CDC-ECM) on the nRF *native* USB port (2nd cable); the
#           host sees a cdc_ether netdev — run ./start_ecm.sh. No J-Link OB or
#           UART in the path. Console is forced to RTT (USB carries only ECM).
# Either way the uplink gets link-local fe80::2, default route via host fe80::1.
UPLINK ?= slip
ifeq (slip,$(UPLINK))
  USEMODULE += slipdev
  # 1 Mbaud verified (2026-10-15) once UARTE0 runs at IRQ prio 0 (see main.c).
  # Pass the same rate to start_network.sh.
  SLIP_BAUDRATE ?= 1000000
  CFLAGS += -DSLIPDEV_PARAM_UART=UART_DEV\(0\)
  CFLAGS += -DSLIPDEV_PARAM_BAUDRATE=$(SLIP_BAUDRATE)
  CFLAGS += -DBR_UPLINK_SLIP=1
else ifeq (cdc-ecm,$(UPLINK))
  USEMODULE += auto_init_usbus
  USEMODULE += usbus_cdc_ecm
  # per-netif IPv6 event bus: main.c re-adds fe80::2 after each USB link-up
  USEMODULE += gnrc_netif_bus
  # pid.codes test VID/PID (as mstp_monitor); fine for a bench, not for shipping.
  USB_VID ?= $(USB_VID_TESTING)
  USB_PID ?= $(USB_PID_TESTING)
  CFLAGS += -DBR_UPLINK_CDC_ECM=1
  ifeq (cdc_acm,$(CONSOLE))
    $(error UPLINK=cdc-ecm uses the native USB for ECM; use CONSOLE=rtt)
  endif
  CONSOLE ?= rtt
else
  $(error UPLINK must be slip or cdc-ecm)
endif

# --- gnrc 6LoWPAN border router: 6LBR NIB role + IPv6 router + forwarding ---
# pulls gnrc_ipv6_nib_6lbr + gnrc_ipv6_router_default + gnrc_sixlowpan.
USEMODULE += gnrc_netif
USEMODULE += gnrc_sixlowpan_border_router_default
USEMODULE += gnrc_icmpv6_echo
# auto-init creates the slip uplink netif; the mstp downstream is created manually
# in main() (custom L2 ops). mstp has no auto-init entry, so only slip is auto-made.
USEMODULE += auto_init_gnrc_netif

# --- console: CONSOLE=cdc_acm (default with slip) or CONSOLE=rtt (forced with cdc-ecm) ---
# SLIP owns the J-Link VCP, so stdio can't use UART0.
#  cdc_acm: nRF native USB (2nd cable, 2nd /dev/ttyACM*); `make term PORT=/dev/ttyACMx`.
#           Boot banner is lost to the USB re-enumeration race after a reset.
#  rtt:     SEGGER RTT over the same J-Link cable as flash + SLIP (no nRF USB cable);
#           `make CONSOLE=rtt JLINK_SERIAL=<SN> term` (stdio_rtt selects the J-Link RTT terminal). RTT buffers output, so the
#           banner survives until the terminal attaches.
CONSOLE ?= cdc_acm
ifeq (rtt,$(CONSOLE))
  USEMODULE += stdio_rtt
else
  USEMODULE += stdio_cdc_acm
endif

# --- shell + diagnostics ---
USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += ps
USEMODULE += gnrc_pktdump
USEMODULE += shell_cmd_gnrc_ipv6_nib

# --- ztimer backends used by the mstp driver ---
USEMODULE += ztimer
USEMODULE += ztimer_msec
USEMODULE += ztimer_usec

# RFC 8163 MS/TP carries MSDUs up to 1500 octets (the driver reports
# NETOPT_MAX_PDU_SIZE = MSTP_MAX_MSDU = 1500), but gnrc pins every 6Lo
# interface to the IPv6 minimum MTU of 1280 unless this is set. With it the
# mstp netif MTU becomes 1500 (largest unfragmented echo: ping -s 1452).
CFLAGS += -DCONFIG_GNRC_NETIF_NONSTANDARD_6LO_MTU=1

# MS/TP This Station address: `make MSTP_ADDR=<n> ...` (default 2). Must be unique
# on the bus and <= Nmax_manager (8 on this bed). Changing it rebuilds (CFLAGS change).
MSTP_ADDR ?= 2
CFLAGS += -DMSTP_SRC_ADDR=$(MSTP_ADDR)U

include $(RIOTBASE)/Makefile.include
