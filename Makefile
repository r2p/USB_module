##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O3 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions -fno-threadsafe-statics
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = no
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

ifeq ($(R2P_USE_DEBUGTRANSPORT),)
  R2P_USE_DEBUGTRANSPORT = yes
endif

ifeq ($(R2P_USE_RTCANTRANSPORT),)
  R2P_USE_RTCANTRANSPORT = yes
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

R2P_ROOT ?= /opt/R2P
CHIBIOS=$(R2P_ROOT)/core/ChibiOS_2.6.5
RTCAN=$(R2P_ROOT)/core/RTCAN
R2P=$(R2P_ROOT)/core/Middleware


# Define project name here
PROJECT = fw

# Imported source files and paths
include ./board.mk
include $(CHIBIOS)/os/hal/platforms/STM32F30x/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F3xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(RTCAN)/platforms/STM32/platform.mk
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk
include $(RTCAN)/RTCAN.mk
include $(R2P)/r2p.mk
include $(R2P)/port/chibios/port.mk


# Define linker script file here
LDSCRIPT= $(PORTLD)/STM32F303xC.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
       $(CHIBIOS)/os/various/chprintf.c \
       $(CHIBIOS)/os/various/memstreams.c \
       $(CHIBIOS)/os/various/shell.c \
       $(RTCANSRC) \
       $(RTCANPLATFORMSRC) \
       stubs.c \
       usbcfg.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(R2PSRC) \
         $(R2PNODES)/led.cpp \
         chnew.cpp

ifeq ($(MAIN), blinking_led)
	CSRC += test/blinking_led.c
endif

ifeq ($(MAIN), hardware_test)
	CSRC   += $(FATFSSRC) 
	CPPSRC += test/hardware_test.cpp
endif

ifeq ($(MAIN), hardware_test_console)
	CPPSRC += test/hardware_test_console.cpp
endif

ifeq ($(MAIN), rosserial_test)
	CPPSRC += ros_lib/duration.cpp \
              ros_lib/time.cpp \
              test/rosserial_test.cpp
endif

ifeq ($(MAIN), bootloader_test)
	CPPSRC += test/bootloader_test.cpp
	UDEFS += -DR2P_USE_BOOTLOADER=1
	LDSCRIPT = ./STM32F303xB_bootloader.ld
endif

ifeq ($(MAIN), triskar)
  CPPSRC += main_triskar.cpp
endif

ifeq ($(MAIN), tilty)
  CPPSRC += ros_lib/duration.cpp \
			ros_lib/time.cpp \
			main_tilty.cpp
endif

ifeq ($(MAIN),)
  CPPSRC += main.cpp
endif

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various \
         $(FATFSINC) \
         $(RTCANINC) $(RTCANPLATFORMINC) \
         $(R2PINC) ./ros_lib

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS += -DCHPRINTF_USE_FLOAT=1 -DR2P_ITERATE_PUBSUB=1 -DR2P_USE_BRIDGE_MODE=0 #-DR2P_USE_BOOTLOADER=0 -DR2P_ASSERT=""

# Define ASM defines here
UADEFS +=

# List all user directories here
UINCDIR += ros_lib

# List the user directory to look for the libraries here
ULIBDIR +=

# List all user libraries here
ULIBS +=

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/ports/GCC/ARMCMx
include $(RULESPATH)/rules.mk
