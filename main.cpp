#include <stdlib.h> // atof()

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "usbcfg.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/motor.hpp>
#include <r2p/msg/imu.hpp>
#include <r2p/msg/proximity.hpp>


#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "uDC"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

r2p::Node vel_node("speedpub", false);
r2p::Publisher<r2p::Speed2Msg> vel_pub;

r2p::Node pidcfg_node("pidcfg", false);
r2p::Publisher<r2p::PIDCfgMsg> pidcfg_pub;

bool stream_imu = false;
bool stream_enc = false;
bool stream_proxy = false;

BaseSequentialStream * serialp;

/*
 * DP resistor control is not possible on the STM32F3-Discovery, using stubs
 * for the connection macros.
 */
#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

/*===========================================================================*/
/* Kinematics.                                                               */
/*===========================================================================*/
/*
 *               y
 *               ^
 *               |
 *               |
 *   2           @---->x     1
 *  ||                       ||  |
 *  ||_______________________||  | R
 *  ||                       ||
 *  ||                       ||
 *               L
 *   <----------------------->
 *
 */

// Robot parameters
#define _L        0.400f    // Wheel distance [m]
#define _R        0.05f    // Wheel radius [m]

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
	size_t n, size;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
	static const char *states[] = { THD_STATE_NAMES };
	Thread *tp;

	(void) argv;
	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n", (uint32_t) tp, (uint32_t) tp->p_ctx.r13,
				(uint32_t) tp->p_prio, (uint32_t)(tp->p_refs - 1), states[tp->p_state], (uint32_t) tp->p_time);
		tp = chRegNextThread(tp);
	} while (tp != NULL);
}

static void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::Speed2Msg * msgp;

	(void) argv;

	if (argc != 2) {
		chprintf(chp, "Usage: r <forward> <angular>\r\n");
		return;
	}

	serialp = chp;
	vel_node.set_enabled(true);

	float forward = atof(argv[0]);
	float angular = atof(argv[1]);

	// Motor setpoints
	if (vel_pub.alloc(msgp)) {
		msgp->value[0] = (1 / _R) * (forward + (_L / 2) * angular);
		msgp->value[1] = -(1 / _R) * (forward - (_L / 2) * angular);
		vel_pub.publish(*msgp);
	}

	vel_node.set_enabled(false);
}

static void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::Speed2Msg * msgp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: s\r\n");
		return;
	}

	vel_node.set_enabled(true);

	if (vel_pub.alloc(msgp)) {
		msgp->value[0] = 0;
		msgp->value[1] = 0;
		vel_pub.publish(*msgp);
	}

	vel_node.set_enabled(false);
}

static void cmd_pidcfg(BaseSequentialStream *chp, int argc, char *argv[]) {
	static bool first_time = true;
	r2p::PIDCfgMsg * msgp;

	(void) argv;

	if (argc != 3) {
		chprintf(chp, "Usage: pidcfg <k> <ti> <td>\r\n");
		return;
	}

	pidcfg_node.set_enabled(true);

	if (first_time) {
		pidcfg_node.advertise(pidcfg_pub, "pidcfg", r2p::Time::INFINITE);
		first_time = false;
	}
	if (pidcfg_pub.alloc(msgp)) {
		msgp->k = atof(argv[0]);
		msgp->ti = atof(argv[1]);
		msgp->td = atof(argv[2]);
		pidcfg_pub.publish(*msgp);
	}

	pidcfg_node.set_enabled(false);
}

static void cmd_enc(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: e\r\n");
		return;
	}

	serialp = chp;
	stream_enc = !stream_enc;
}

static void cmd_imu(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: i\r\n");
		return;
	}

	serialp = chp;
	stream_imu = !stream_imu;
}

static void cmd_proxy(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: p\r\n");
		return;
	}

	serialp = chp;
	stream_proxy = !stream_proxy;
}

static const ShellCommand commands[] = { { "mem", cmd_mem }, { "threads", cmd_threads }, { "r", cmd_run }, { "s",
		cmd_stop }, { "pidcfg", cmd_pidcfg }, { "e", cmd_enc }, { "i", cmd_imu }, { "p", cmd_proxy }, { NULL, NULL } };

static const ShellConfig usb_shell_cfg = { (BaseSequentialStream *) &SDU1, commands };

static const ShellConfig serial_shell_cfg = { (BaseSequentialStream *) &SD3, commands };


/*
 * Encoder subscriber node.
 */
msg_t encoder_sub_node(void * arg) {
	r2p::Node node("enc_sub");
	r2p::Subscriber<r2p::Encoder2Msg, 5> enc_sub;
	r2p::Encoder2Msg * msgp;

	(void) arg;
	chRegSetThreadName("enc_sub");

	node.subscribe(enc_sub, "encoder2");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
		if (enc_sub.fetch(msgp)) {
			if (stream_enc) {
				chprintf((BaseSequentialStream*) serialp, "%f %f\r\n", msgp->delta[0], msgp->delta[1]);
			}
			enc_sub.release(*msgp);
		} else {
			r2p::Thread::sleep(r2p::Time::ms(1));
		}
	}

	return CH_SUCCESS;
}


/*
 * IMU subscriber node.
 */
msg_t r2p_sub_node(void * arg) {
	r2p::Node node("tilt_sub");
	r2p::Subscriber<r2p::IMUMsg, 5> imu_sub;
	r2p::IMUMsg * msgp;

	(void) arg;
	chRegSetThreadName("imu_sub");

	node.subscribe(imu_sub, "imu");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
		if (imu_sub.fetch(msgp)) {
			if (stream_imu) {
				chprintf((BaseSequentialStream*) serialp, "%f %f %f\r\n", msgp->roll, msgp->pitch, msgp->yaw);
			}
			imu_sub.release(*msgp);
		} else {
			r2p::Thread::sleep(r2p::Time::ms(1));
		}
	}

	return CH_SUCCESS;
}


/*
 * Proximity subscriber node.
 */
msg_t proxy_sub_node(void * arg) {
	r2p::Node node("prox_sub");
	r2p::Subscriber<r2p::ProximityMsg, 5> proxy_sub;
	r2p::ProximityMsg * msgp;

	(void) arg;
	chRegSetThreadName("proxy_sub");

	node.subscribe(proxy_sub, "proximity");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
		if (proxy_sub.fetch(msgp)) {
			if (stream_proxy) {
				chprintf((BaseSequentialStream*) serialp, "%5d %5d %5d %5d %5d %5d %5d %5d \r\n", msgp->value[0], msgp->value[1], msgp->value[2], msgp->value[3], msgp->value[4], msgp->value[5], msgp->value[6], msgp->value[7]);
			}
			proxy_sub.release(*msgp);
		} else {
			r2p::Thread::sleep(r2p::Time::ms(1));
		}
	}

	return CH_SUCCESS;
}


/*
 * Application entry point.
 */
extern "C" {
int main(void) {
	Thread *usb_shelltp = NULL;
//	Thread *serial_shelltp = NULL;

	halInit();
	chSysInit();

	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(500);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);

	/* Start the serial driver. */
	sdStart(&SD3, NULL);

	/*
	 * Shell manager initialization.
	 */
	shellInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledpub_conf ledpub_conf = { "led", 1 };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledpub_node, &ledpub_conf);

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p_sub_node, NULL);

	vel_node.advertise(vel_pub, "speed2", r2p::Time::INFINITE);

	for (;;) {
		if (!usb_shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			usb_shelltp = shellCreate(&usb_shell_cfg, SHELL_WA_SIZE,
			NORMALPRIO);
		else if (chThdTerminated(usb_shelltp)) {
			chThdRelease(usb_shelltp); /* Recovers memory of the previous shell.   */
			usb_shelltp = NULL; /* Triggers spawning of a new shell.        */
		}
		/*
		 if (!serial_shelltp)
		 serial_shelltp = shellCreate(&serial_shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		 else if (chThdTerminated(serial_shelltp)) {
		 chThdRelease(serial_shelltp);
		 serial_shelltp = NULL;
		 }
		 */
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
