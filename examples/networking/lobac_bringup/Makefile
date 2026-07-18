# 6LoBAC (RFC 8163) MS/TP Frame Type 34 bring-up test
APPLICATION = lobac_bringup

# Default target board: ST Nucleo-F767ZI + DFRobot RS485 Shield (DFR0259).
BOARD ?= nucleo-f767zi

# Absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../..

DEVELHELP ?= 1
QUIET ?= 1

# Peripherals + timers used by the bring-up emitter.
USEMODULE += periph_uart
USEMODULE += periph_gpio
USEMODULE += ztimer
USEMODULE += ztimer_usec
USEMODULE += ztimer_msec

include $(RIOTBASE)/Makefile.include
