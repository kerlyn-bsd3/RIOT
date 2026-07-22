# 6LoBAC: IPv6-over-MS/TP (RFC 8163) node on the ST Nucleo-F767ZI + DFR0259.
APPLICATION = mstp_6lo

BOARD ?= nucleo-f767zi
RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# MS/TP driver: provides the netdev (mstp_driver), the engine (mstp_start), and
# the Type-34 TX enqueue (mstp_tx_ipv6). Pulls periph_uart/gpio + ztimer via its
# own Makefile.dep.
USEMODULE += mstp
# iolist_to_buffer() gathers the gnrc pktsnip chain into our MSDU staging buffer.
USEMODULE += iolist

# gnrc 6LoWPAN / IPv6 stack. The netif is created manually in main.c (custom L2
# ops), so we do NOT use auto_init_gnrc_netif / gnrc_netdev_default.
USEMODULE += gnrc_netif
USEMODULE += gnrc_sixlowpan_default
USEMODULE += gnrc_ipv6_default
USEMODULE += gnrc_icmpv6_echo

# shell + diagnostics
USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += ps
USEMODULE += gnrc_pktdump

# ztimer backends used by the driver
USEMODULE += ztimer
USEMODULE += ztimer_msec
USEMODULE += ztimer_usec

include $(RIOTBASE)/Makefile.include
