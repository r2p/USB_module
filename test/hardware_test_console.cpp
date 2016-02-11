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
#include <r2p/msg/std_msgs.hpp>

#define _TICKS 64.0f
#define _RATIO 29.0f
#define _PI 3.14159265359f

#define R2T(r) ((r / (2 * _PI)) * (_TICKS * _RATIO))
#define T2R(t) ((t / (_TICKS * _RATIO)) * (2 * _PI))

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);


msg_t udc_test_node(void * arg);
msg_t imu_test_node(void * arg);
msg_t imuraw_test_node(void * arg);
msg_t gps_test_node(void * arg);
msg_t servo_test_node(void * arg);

bool imu_raw = false;

bool enc_callback(const r2p::EncoderMsg &msg);
r2p::Node node("udc_test", false);
r2p::Publisher<r2p::SpeedMsg> vel_pub;
r2p::Subscriber<r2p::EncoderMsg, 5> enc_sub(enc_callback);


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


static void cmd_udc_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread * tp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: udc\r\n");
		return;
	}

	tp = chThdCreateFromHeap(NULL, THD_WA_SIZE(2048), NORMALPRIO, udc_test_node, chp);
	chThdWait(tp);

}


static void cmd_imu_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread * tp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: imu\r\n");
		return;
	}

	imu_raw = false;
	tp = chThdCreateFromHeap(NULL, THD_WA_SIZE(2048), NORMALPRIO, imu_test_node, chp);
	chThdWait(tp);

}

static void cmd_imuraw_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread * tp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: imuraw\r\n");
		return;
	}

	imu_raw = true;
	tp = chThdCreateFromHeap(NULL, THD_WA_SIZE(2048), NORMALPRIO, imu_test_node, chp);
	chThdWait(tp);

}

static void cmd_gps_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread * tp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: gps\r\n");
		return;
	}

	tp = chThdCreateFromHeap(NULL, THD_WA_SIZE(2048), NORMALPRIO, gps_test_node, chp);
	chThdWait(tp);

}

static void cmd_servo_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread * tp;

	(void) argv;

	if (argc > 0) {
		chprintf(chp, "Usage: servo\r\n");
		return;
	}

	tp = chThdCreateFromHeap(NULL, THD_WA_SIZE(2048), NORMALPRIO, servo_test_node, chp);
	chThdWait(tp);

}

static const ShellCommand commands[] = { { "mem", cmd_mem }, { "threads", cmd_threads }, { "udc", cmd_udc_test}, { "imu", cmd_imu_test}, { "imuraw", cmd_imuraw_test}, { "gps", cmd_gps_test}, { "servo", cmd_servo_test}, { NULL, NULL } };

static const ShellConfig usb_shell_cfg = { (BaseSequentialStream *) &SDU1, commands };

/*
 * uDC test node.
 */
float delta = 0;

bool enc_callback(const r2p::EncoderMsg &msg) {

	delta = msg.delta;

	return true;
}


msg_t udc_test_node(void * arg) {
	BaseSequentialStream * chp = reinterpret_cast<BaseSequentialStream *>(arg);
	r2p::SpeedMsg * msgp;
	int16_t setpoint = 0;
	int16_t step = 100;
	r2p::Time last_publish(0);
	r2p::Time last_setpoint(0);

	chRegSetThreadName("udc_test");

	node.set_enabled(true);
	node.advertise(vel_pub, "speed");
	node.subscribe(enc_sub, "encoder");

	last_setpoint = r2p::Time::now();

	while (setpoint >= 0) {
		node.spin(500);

		if (r2p::Time::now() - last_publish < r2p::Time::ms(200)) continue;

		chprintf(chp, "Setpoint: %d, encoder: %f\r\n", setpoint, delta * 50 * 60 / (2 * _PI)); // RAD/tick -> RPM

		if (vel_pub.alloc(msgp)) {
			msgp->value = setpoint / 60*(2 * _PI); // RPM -> RAD/S
			vel_pub.publish(*msgp);
		}

		last_publish = r2p::Time::now();

		if (r2p::Time::now() - last_setpoint > r2p::Time::ms(1000)) {
			if (setpoint >= 500) {
				step = -step;
			}

			setpoint += step;

			last_setpoint = r2p::Time::now();
		}
	}

	node.set_enabled(false);
}


/*
 * IMU test node.
 */
msg_t imu_test_node(void * arg) {
	BaseSequentialStream * chp = reinterpret_cast<BaseSequentialStream *>(arg);
	r2p::Node node("tilt_sub");
	r2p::Subscriber<r2p::IMUMsg, 5> imu_sub;
	r2p::IMUMsg * msgp;
	r2p::Subscriber<r2p::IMURaw9, 5> imuraw_sub;
	r2p::IMURaw9 * rawmsgp;

	(void) arg;
	chRegSetThreadName("imu_test");

	node.subscribe(imu_sub, "imu");
	node.subscribe(imuraw_sub, "imu_raw");

	while (!chThdShouldTerminate()) {
		node.spin(r2p::Time::ms(1000));
		r2p::Thread::sleep(r2p::Time::ms(100));
		if (imu_raw && imuraw_sub.fetch(rawmsgp)) {
			chprintf(chp, "%5d %5d %5d %5d %5d %5d %5d %5d %5d\r\n", rawmsgp->acc_x, rawmsgp->acc_y, rawmsgp->acc_z, rawmsgp->gyro_x, rawmsgp->gyro_y, rawmsgp->gyro_z, rawmsgp->mag_x, rawmsgp->mag_y, rawmsgp->mag_z);
			imuraw_sub.release(*rawmsgp);
		}else if (!imu_raw && imu_sub.fetch(msgp)) {
			chprintf(chp, "%f %f %f\r\n", msgp->roll, msgp->pitch, msgp->yaw);
			imu_sub.release(*msgp);
		} else {
			chprintf(chp, "Timeout\r\n");
		}
	}

	return CH_SUCCESS;
}

/*
 * GPS test node.
 */
msg_t gps_test_node(void * arg) {
	BaseSequentialStream * chp = reinterpret_cast<BaseSequentialStream *>(arg);
	r2p::Node node("gps_test");
	r2p::Subscriber<r2p::GPSMsg, 5> gps_sub;
	r2p::GPSMsg * msgp;

	(void) arg;
	chRegSetThreadName("gps_test");

	node.subscribe(gps_sub, "gps");

	while (!chThdShouldTerminate()) {
		node.spin(r2p::Time::ms(2000));
		if (gps_sub.fetch(msgp)) {
			chprintf(chp, "%d %d %f %f\r\n", msgp->valid, msgp->satellites, msgp->latitude, msgp->longitude);
			gps_sub.release(*msgp);
		} else {
			chprintf(chp, "Timeout\r\n");
		}
	}

	return CH_SUCCESS;
}

/*
 * Servo test node.
 */
msg_t servo_test_node(void * arg) {
	BaseSequentialStream * chp = reinterpret_cast<BaseSequentialStream *>(arg);
	r2p::Node node("servo_sub");
	r2p::Subscriber<r2p::ServoMsg, 5> servo_sub;
	r2p::ServoMsg * msgp;

	(void) arg;
	chRegSetThreadName("servo_test");

	node.subscribe(servo_sub, "rcin");

	while (!chThdShouldTerminate()) {
		node.spin(r2p::Time::ms(1000));
		r2p::Thread::sleep(r2p::Time::ms(100));
		if (servo_sub.fetch(msgp)) {
			chprintf(chp, "%4d %4d %4d %4d %4d %4d %4d %4d \r\n", msgp->pulse[0],  msgp->pulse[1],  msgp->pulse[2],  msgp->pulse[3],  msgp->pulse[4],  msgp->pulse[5],  msgp->pulse[6],  msgp->pulse[7]);
			servo_sub.release(*msgp);
		} else {
			chprintf(chp, "Timeout\r\n");
		}
	}

	return CH_SUCCESS;
}

/*
 * Test node.
 */
msg_t test_sub_node(void * arg) {
	r2p::Node node("test_sub");
	r2p::Subscriber<r2p::String64Msg, 5> sub;
	r2p::String64Msg * msgp;

	(void) arg;
	chRegSetThreadName("test_sub");

	node.subscribe(sub, "test");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
		if (sub.fetch(msgp)) {
			chprintf((BaseSequentialStream*)&SDU1, "%s\r\n", msgp->data);
			sub.release(*msgp);
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

	/*
	 * Shell manager initialization.
	 */
	shellInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledpub_conf ledpub_conf = { "led", 1 };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledpub_node, &ledpub_conf);

	r2p::ledsub_conf ledsub_conf = {"led"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, test_sub_node, NULL);

	for (;;) {
		if (!usb_shelltp && (SDU1.config->usbp->state == USB_ACTIVE)) {
			usb_shelltp = shellCreate(&usb_shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		} else if (chThdTerminated(usb_shelltp)) {
			chThdRelease(usb_shelltp); /* Recovers memory of the previous shell.   */
			usb_shelltp = NULL; /* Triggers spawning of a new shell.        */
		}
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
