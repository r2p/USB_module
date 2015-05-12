MODULE = USB_module
MODULE_NAME = usb

ifeq ($(TEST),blinking_led)
	PRJ_CSRC += test/blinking_led.c
endif
	
ifeq ($(TEST),hardware_test)
	PACKAGES += led
	PRJ_CPPSRC += test/hardware_test.cpp
endif

ifeq ($(TEST),hardware_test_console)
	PACKAGES += led
	PRJ_CPPSRC += test/hardware_test_console.cpp
endif

ifeq ($(TEST),)
	PACKAGES += led
	PRJ_CPPSRC += main.cpp
endif

include $(R2P_ROOT)/core/r2p.mk
