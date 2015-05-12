#include <cstdio>

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/std_msgs.hpp>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "USB"
#endif

#ifndef SERIAL_USB
#define SERIAL_USB			0
#endif

#ifndef DEBUG_TRANSPORT
#define DEBUG_TRANSPORT		1
#endif

#ifndef RTCAN_TRANSPORT
#define RTCAN_TRANSPORT		0
#endif

#ifndef R2P_USE_BRIDGE_MODE
#define R2P_USE_BRIDGE_MODE		1
#endif

static WORKING_AREA(wa_info, 1024);
#define BOOT_STACKLEN   1024

#if R2P_USE_BRIDGE_MODE
enum { PUBSUB_BUFFER_LENGTH = 16 };
r2p::Middleware::PubSubStep pubsub_buf[PUBSUB_BUFFER_LENGTH];
#endif

#if DEBUG_TRANSPORT
static char dbgtra_namebuf[64];
#if SERIAL_USB
static r2p::DebugTransport dbgtra("SDU1", reinterpret_cast<BaseChannel *>(&SDU1),
		dbgtra_namebuf);
#else
static r2p::DebugTransport dbgtra("SD3", reinterpret_cast<BaseChannel *>(&SD3),
		dbgtra_namebuf);
#endif

static WORKING_AREA(wa_rx_dbgtra, 1024);
static WORKING_AREA(wa_tx_dbgtra, 1024);
#endif // DEBUG_TRANSPORT

#if RTCAN_TRANSPORT
static r2p::RTCANTransport rtcantra(RTCAND1);
RTCANConfig rtcan_config = { 1000000, 100, 60 };
#endif // RTCAN_TRANSPORT


r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME
#if R2P_USE_BRIDGE_MODE
		, pubsub_buf, PUBSUB_BUFFER_LENGTH
#endif
		);


void usb_lld_disconnect_bus(USBDriver *usbp) {

	(void)usbp;
	palClearPort(GPIOA, (1<<GPIOA_USB_DM) | (1<<GPIOA_USB_DP));
	palSetPadMode(GPIOA, GPIOA_USB_DM, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_OUTPUT_PUSHPULL);
}

void usb_lld_connect_bus(USBDriver *usbp) {

	(void)usbp;
	palClearPort(GPIOA, (1<<GPIOA_USB_DM) | (1<<GPIOA_USB_DP));
	palSetPadMode(GPIOA, GPIOA_USB_DM, PAL_MODE_ALTERNATE(14));
	palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_ALTERNATE(14));
}


/*
 * Test node
 */
msg_t test_pub_node(void *arg) {
	r2p::Node node("test_pub");
	r2p::Publisher<r2p::String64Msg> pub;
	r2p::String64Msg * msgp;
	uint16_t * uuid = (uint16_t *)0x1FFFF7AC;

	(void) arg;
	chRegSetThreadName("test_pub");

	node.advertise(pub, "test");

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(1000);

	sprintf(msgp->data, "\n\n"R2P_MODULE_NAME" module [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	return CH_SUCCESS;
}


/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	palClearPad(LED1_GPIO, LED1);
	chThdSleepMilliseconds(500);
	palSetPad(LED1_GPIO, LED1);

	void *boot_stackp = NULL;
	if (r2p::Middleware::is_bootloader_mode()) {
		uint8_t *stackp = new uint8_t[BOOT_STACKLEN + sizeof(stkalign_t)];
		R2P_ASSERT(stackp != NULL);
		stackp += (sizeof(stkalign_t) - reinterpret_cast<uintptr_t>(stackp)) % sizeof(stkalign_t);
		boot_stackp = stackp;
	}

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST, boot_stackp, BOOT_STACKLEN,
			r2p::Thread::LOWEST);

//	r2p::Middleware::instance.preload_bootloader_mode(false);

#if DEBUG_TRANSPORT
#if SERIAL_USB
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1000);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);
#else
	sdStart(&SD3, NULL);
#endif
	dbgtra.initialize(wa_rx_dbgtra, sizeof(wa_rx_dbgtra), r2p::Thread::LOWEST + 11,
			wa_tx_dbgtra, sizeof(wa_tx_dbgtra), r2p::Thread::LOWEST + 10);
#endif // DEBUG_TRANSPORT

#if RTCAN_TRANSPORT
	rtcantra.initialize(rtcan_config);
#endif // RTCAN_TRANSPORT

	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = {"led"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	for (;;) {
		if (r2p::Middleware::is_bootloader_mode()) {
			palTogglePad(LED1_GPIO, LED1);
		}
		r2p::Thread::sleep(r2p::Time::ms(100));
	}

	return CH_SUCCESS;
}
}
