# 6LoBAC MS/TP bus monitor: active MS/TP station (default 0) that mirrors every
# octet on the bus — received and its own transmitted — out the nRF52840 native
# USB as a CDC-ACM byte stream (drop-in for the BDK monitor + Karg's capture tool).
APPLICATION = mstp_monitor

BOARD ?= nrf52840dk
RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# --- MS/TP (clean-room driver), token passing only: no IPv6 stack ---
USEMODULE += mstp

# --- data out: native USB CDC-ACM, set up by hand in main() so that the CDC-ACM
# port carries ONLY bus octets (stdio stays on the J-Link VCP / UARTE0) ---
USEMODULE += usbus_cdc_acm
DISABLE_MODULE += auto_init_usbus
# pid.codes test VID/PID (RIOT's development IDs) — fine for bench use.
USB_VID ?= $(USB_VID_TESTING)
USB_PID ?= $(USB_PID_TESTING)

# --- console on the J-Link VCP (default nrf52840dk stdio = UART0) + shell ---
USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += ps

USEMODULE += ztimer
USEMODULE += ztimer_msec
USEMODULE += ztimer_usec

# MS/TP This Station address: `make MSTP_ADDR=<n> ...` (default 0). Must be unique
# on the bus and <= Nmax_manager (8 on this bed). Changing it rebuilds (CFLAGS change).
MSTP_ADDR ?= 0
CFLAGS += -DMSTP_SRC_ADDR=$(MSTP_ADDR)U

include $(RIOTBASE)/Makefile.include
