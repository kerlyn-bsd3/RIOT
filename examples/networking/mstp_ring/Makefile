# MS/TP token-ring join bring-up (RFC 8163 / 6LoBAC)
APPLICATION = mstp_ring

# ST Nucleo-F767ZI + DFRobot RS485 Shield (DFR0259).
BOARD ?= nucleo-f767zi

RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# The MS/TP driver (drivers/mstp) pulls in periph_uart, periph_gpio,
# ztimer_periodic and core_thread_flags via its Makefile.dep.
USEMODULE += mstp
USEMODULE += ztimer
USEMODULE += ztimer_msec
USEMODULE += ztimer_usec

# Max_Manager: highest MS-TP manager address this node polls (0..Nmax_manager).
CFLAGS += -DMSTP_DEFAULT_NMAX_MANAGER=8

include $(RIOTBASE)/Makefile.include
