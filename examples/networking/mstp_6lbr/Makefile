# 6LoBAC 6LBR: MS/TP (RFC 8163) downstream <-> SLIP uplink to a Linux host.
APPLICATION = mstp_6lbr

BOARD ?= nrf52840dk
RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# --- MS/TP downstream (the clean-room driver) ---
USEMODULE += mstp
USEMODULE += iolist

# --- SLIP uplink: slipdev on UART_DEV(0) = nRF52840-DK J-Link VCP ---
# SLIP owns the J-Link VCP; stdio goes to the native USB CDC-ACM (below).
USEMODULE += slipdev
# Override on the command line, e.g. `make SLIP_BAUDRATE=115200`, and pass the same
# rate to start_network.sh. 115200 is verified on the J-Link OB VCP (2026-10-14);
# 576000 (the Contiki BDK rate) fails there — see the session log.
SLIP_BAUDRATE ?= 576000
CFLAGS += -DSLIPDEV_PARAM_UART=UART_DEV\(0\)
CFLAGS += -DSLIPDEV_PARAM_BAUDRATE=$(SLIP_BAUDRATE)

# --- gnrc 6LoWPAN border router: 6LBR NIB role + IPv6 router + forwarding ---
# pulls gnrc_ipv6_nib_6lbr + gnrc_ipv6_router_default + gnrc_sixlowpan.
USEMODULE += gnrc_netif
USEMODULE += gnrc_sixlowpan_border_router_default
USEMODULE += gnrc_icmpv6_echo
# auto-init creates the slip uplink netif; the mstp downstream is created manually
# in main() (custom L2 ops). mstp has no auto-init entry, so only slip is auto-made.
USEMODULE += auto_init_gnrc_netif

# --- console: CONSOLE=cdc_acm (default) or CONSOLE=rtt ---
# SLIP owns the J-Link VCP, so stdio can't use UART0.
#  cdc_acm: nRF native USB (2nd cable, 2nd /dev/ttyACM*); `make term PORT=/dev/ttyACMx`.
#           Boot banner is lost to the USB re-enumeration race after a reset.
#  rtt:     SEGGER RTT over the same J-Link cable as flash + SLIP (no nRF USB cable);
#           `make CONSOLE=rtt JLINK_SERIAL=<SN> term-rtt`. RTT buffers output, so the
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

include $(RIOTBASE)/Makefile.include
