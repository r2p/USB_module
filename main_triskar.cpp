#include <stdlib.h> // atof()

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "usbcfg.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/motor.hpp>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "USB"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

r2p::Node vel_node("speedpub", false);
r2p::Publisher<r2p::Speed3Msg> vel_pub;
bool speed_first_time = true;

r2p::Node pidcfg_node("pidcfg", false);
r2p::Publisher<r2p::PIDCfgMsg> pidcfg_pub;
bool pidcfg_first_time = true;

BaseSequentialStream * serialp;
bool stream_enc = false;

/*
 * DP resistor control is not possible on the STM32F3-Discovery, using stubs
 * for the connection macros.
 */
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


/*===========================================================================*/
/* Kinematics.                                                               */
/*===========================================================================*/

template<typename T> static inline T clamp(T min, T value, T max) {
	return (value < min) ? min : ((value > max) ? max : value);
}

/*
 *  //_______________________\\
 * //            x            \\
 *   \  2        ^        1  /
 *    \          |          /
 *     \         |         /
 *       \ y<----@       /
 *        \      z      /
 *         \           /
 *           \       /
 *            \  3  /
 *             \___/
 *             =====
 *
 * Body frame velocity to wheel angular velocity:
 * R * dth1 = cos(60°) * dx - cos(30°) * dy - L * dgamma
 * R * dth2 = cos(60°) * dx + cos(30°) * dy - L * dgamma
 * R * dth3 =           -dx                 - L * dgamma
 *
 * Name mapping and units:
 * dx     = strafe     [m/s]
 * dy     = forward    [m/s]
 * dgamma = angular    [rad/s]
 * dth{1,2,3}          [rad/s]
 */

// Robot parameters
#define _L        0.160f    // Wheel distance [m]
#define _R        0.035f    // Wheel radius [m]
#define _MAX_DTH  52.0f     // Maximum wheel angular speed [rad/s]

#define _m1_R     (-1.0f / _R)
#define _mL_R     (-_L / _R)
#define _C60_R    (0.500000000f / _R)   // cos(60°) / R
#define _C30_R    (0.866025404f / _R)   // cos(30°) / R

#define _TICKS 64.0f
#define _RATIO 29.0f
#define _PI 3.14159265359f

#define R2T(r) (_TICKS * _RATIO)/(r * 2 * _PI)
#define T2R(t) (t / R2T)

#define M2T(m) (m * _TICKS * _RATIO)/(2 * _PI * _R)
#define T2M(t) (t / _M2TICK)

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

static void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::Speed3Msg * msgp;

	(void) argv;

	if (argc != 3) {
		chprintf(chp, "Usage: run <forward> <strafe> <angular>\r\n");
		return;
	}

	vel_node.set_enabled(true);

	if (speed_first_time) {
		vel_node.advertise(vel_pub, "speed3", r2p::Time::INFINITE);
		speed_first_time = false;
	}

	float x = atof(argv[0]);
	float y = atof(argv[1]);
	float w = atof(argv[2]);

	// Wheel angular speeds
	const float dthz123 = _mL_R * w;
	const float dx12 = _C60_R * y;
	const float dy12 = _C30_R * x;

	float dth1 = dx12 - dy12 + dthz123;
	float dth2 = dx12 + dy12 + dthz123;
	float dth3 = _m1_R * y + dthz123;

	// Motor setpoints
	if (vel_pub.alloc(msgp)) {
		msgp->value[0] = (int16_t) clamp(-_MAX_DTH, dth1, _MAX_DTH);
		msgp->value[1] = (int16_t) clamp(-_MAX_DTH, dth2, _MAX_DTH);
		msgp->value[2] = (int16_t) clamp(-_MAX_DTH, dth3, _MAX_DTH);
		vel_pub.publish(*msgp);
	}

	chprintf(chp, "SETPOINT: %f %f %f\r\n", dth1, dth2, dth3);

	vel_node.set_enabled(false);
}

static void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::Speed3Msg * msgp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: stop\r\n");
		return;
	}

	vel_node.set_enabled(true);

	if (speed_first_time) {
		vel_node.advertise(vel_pub, "speed3", r2p::Time::INFINITE);
		speed_first_time = false;
	}

	// Stop motors
	if (vel_pub.alloc(msgp)) {
		msgp->value[0] = 0;
		msgp->value[1] = 0;
		msgp->value[2] = 0;
		vel_pub.publish(*msgp);
	}

	vel_node.set_enabled(false);
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


static void cmd_pidcfg(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::PIDCfgMsg * msgp;

	(void) argv;

	if (argc != 3) {
		chprintf(chp, "Usage: pidcfg <k> <ti> <td>\r\n");
		return;
	}

	pidcfg_node.set_enabled(true);

	if (pidcfg_first_time) {
		pidcfg_node.advertise(pidcfg_pub, "pidcfg", r2p::Time::INFINITE);
		pidcfg_first_time = false;
	}

	if (pidcfg_pub.alloc(msgp)) {
		msgp->k = atof(argv[0]);
		msgp->ti = atof(argv[1]);
		msgp->td = atof(argv[2]);
		pidcfg_pub.publish(*msgp);
	}

	pidcfg_node.set_enabled(false);
}

static const ShellCommand commands[] = { { "r", cmd_run }, { "s", cmd_stop }, { "e", cmd_enc }, { "pidcfg", cmd_pidcfg}, { NULL, NULL } };

static const ShellConfig usb_shell_cfg = { (BaseSequentialStream *) &SDU1, commands };

static const ShellConfig serial_shell_cfg = { (BaseSequentialStream *) &SD3, commands };


/*
 * Encoder subscriber node.
 */
msg_t encoder_sub_node(void * arg) {
	r2p::Node node("enc1_sub");
	r2p::Subscriber<r2p::EncoderMsg, 5> enc_sub;
	r2p::EncoderMsg * msgp;

	(void) arg;
	chRegSetThreadName("enc_sub");

	node.subscribe(enc_sub, "encoder1");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
		if (enc_sub.fetch(msgp)) {
			if (stream_enc) {
				chprintf((BaseSequentialStream*) serialp, "%f\r\n", msgp->delta * 50); // delta_rad to rad/s
			}
			enc_sub.release(*msgp);
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
	Thread *serial_shelltp = NULL;

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

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, encoder_sub_node, NULL);

	for (;;) {
		if (!usb_shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			usb_shelltp = shellCreate(&usb_shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(usb_shelltp)) {
			chThdRelease(usb_shelltp); /* Recovers memory of the previous shell.   */
			usb_shelltp = NULL; /* Triggers spawning of a new shell.        */
		}

		if (!serial_shelltp)
			serial_shelltp = shellCreate(&serial_shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(serial_shelltp)) {
			chThdRelease(serial_shelltp);
			serial_shelltp = NULL;
		}

		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
