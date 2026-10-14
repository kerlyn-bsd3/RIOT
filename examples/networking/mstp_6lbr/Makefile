# 6LoBAC 6LBR: MS/TP (RFC 8163) downstream <-> SLIP uplink to a Linux host.
APPLICATION = mstp_6lbr

BOARD ?= nrf52840dk
RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# --- MS/TP downstream (the clean-room driver) ---
USEMODULE += mstp
USEMODULE += iolist

# --- SLIP uplink: slipdev on UART_DEV(0) = nRF52840-DK J-Link VCP, 576000 baud ---
# SLIP owns the J-Link VCP; stdio goes to the native USB CDC-ACM (below). 576000 is
# honoured because the VCP is a real UART bridge (matches the Contiki BDK link).
USEMODULE += slipdev
CFLAGS += -DSLIPDEV_PARAM_UART=UART_DEV\(0\)
CFLAGS += -DSLIPDEV_PARAM_BAUDRATE=576000

# --- gnrc 6LoWPAN border router: 6LBR NIB role + IPv6 router + forwarding ---
# pulls gnrc_ipv6_nib_6lbr + gnrc_ipv6_router_default + gnrc_sixlowpan.
USEMODULE += gnrc_netif
USEMODULE += gnrc_sixlowpan_border_router_default
USEMODULE += gnrc_icmpv6_echo
# auto-init creates the slip uplink netif; the mstp downstream is created manually
# in main() (custom L2 ops). mstp has no auto-init entry, so only slip is auto-made.
USEMODULE += auto_init_gnrc_netif

# --- console on the nRF native USB CDC-ACM (SLIP owns the J-Link VCP) ---
# Fallback if the native USB port won't enumerate: comment the stdio_cdc_acm line,
# uncomment stdio_rtt, rebuild, and use `make BOARD=nrf52840dk term-rtt`.
USEMODULE += stdio_cdc_acm
# USEMODULE += stdio_rtt

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
