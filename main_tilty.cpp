#include <stdint.h> // atof()

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "usbcfg.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/motor.hpp>
#include <r2p/msg/imu.hpp>

#define USE_USB_SERIAL 1

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <r2p_msgs/ImuRaw.h>
#include <r2p_msgs/PidParameters.h>
#include <r2p_msgs/Vector3_32.h>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "uDC"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

r2p::Node vel_node("velpub", false);
r2p::Publisher<r2p::Velocity3Msg> vel_pub;

r2p::Node balcfg_node("balcfg", false);
r2p::Publisher<r2p::PIDCfgMsg> balcfg_pub;

r2p::Node velcfg_node("velcfg", false);
r2p::Publisher<r2p::PIDCfgMsg> velcfg_pub;

ros::NodeHandle nh;


/*
 * Cycle USB connection on power up.
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

static void cmd_balcfg(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::PIDCfgMsg * msgp;

	(void) argv;

	if (argc != 3) {
		chprintf(chp, "Usage: bcfg <k> <ti> <td>\r\n");
		return;
	}

	balcfg_node.set_enabled(true);

	if (balcfg_pub.alloc(msgp)) {
		msgp->k = atof(argv[0]);
		msgp->ti = atof(argv[1]);
		msgp->td = atof(argv[2]);
		balcfg_pub.publish(*msgp);
	}

	balcfg_node.set_enabled(false);
}

static void cmd_velcfg(BaseSequentialStream *chp, int argc, char *argv[]) {
	r2p::PIDCfgMsg * msgp;

	(void) argv;

	if (argc != 3) {
		chprintf(chp, "Usage: vcfg <k> <ti> <td>\r\n");
		return;
	}

	velcfg_node.set_enabled(true);

	if (velcfg_pub.alloc(msgp)) {
		msgp->k = atof(argv[0]);
		msgp->ti = atof(argv[1]);
		msgp->td = atof(argv[2]);
		velcfg_pub.publish(*msgp);
	}

	velcfg_node.set_enabled(false);
}

static const ShellCommand commands[] = { { "mem", cmd_mem }, { "threads", cmd_threads },
		{ "bcfg", cmd_balcfg }, { "vcfg", cmd_velcfg }, { NULL, NULL } };

static const ShellConfig usb_shell_cfg = { (BaseSequentialStream *) &SDU1, commands };

static const ShellConfig serial_shell_cfg = { (BaseSequentialStream *) &SD3, commands };

/*
 * R2P subscriber node.
 */
struct imu_data_t {
	float roll;
	float pitch;
	float yaw;
} imu_data;

struct imu_raw_data_t {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
} imu_raw_data;

struct odometry_data_t {
	float x;
	float y;
	float w;
} odometry_data;

bool imu_cb(const r2p::IMUMsg &msg) {

	imu_data.roll = msg.roll;
	imu_data.pitch = msg.pitch;
	imu_data.yaw= msg.yaw;

	return true;
}

bool imu_raw_cb(const r2p::IMURaw9 &msg) {

	imu_raw_data.acc_x = msg.acc_x;
	imu_raw_data.acc_y = msg.acc_y;
	imu_raw_data.acc_z = msg.acc_z;
	imu_raw_data.gyro_x = msg.gyro_x;
	imu_raw_data.gyro_y = msg.gyro_y;
	imu_raw_data.gyro_z = msg.gyro_z;
	imu_raw_data.mag_x = msg.mag_x;
	imu_raw_data.mag_y = msg.mag_y;
	imu_raw_data.mag_z = msg.mag_z;

	return true;
}

bool odometry_cb(const r2p::Velocity3Msg &msg) {

	odometry_data.x = msg.x;
	odometry_data.y = msg.y;
	odometry_data.w= msg.w;

	return true;
}

msg_t r2p_sub_node(void * arg) {
	r2p::Node node("r2p_sub");
	r2p::Subscriber<r2p::IMUMsg, 5> imu_sub(imu_cb);
	r2p::Subscriber<r2p::IMURaw9, 5> imu_raw_sub(imu_raw_cb);
	r2p::Subscriber<r2p::Velocity3Msg, 5> odometry_sub(odometry_cb);

	(void) arg;
	chRegSetThreadName("r2p_sub");

	node.subscribe(imu_sub, "imu");
	node.subscribe(imu_raw_sub, "imu_raw");
	node.subscribe(odometry_sub, "odometry");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
	}

	return CH_SUCCESS;
}

/*
 * ROS rosserial publisher thread.
 */

float yaw = 0;
geometry_msgs::Vector3 odom_msg;

msg_t rosserial_pub_thread(void * arg) {
	r2p_msgs::Vector3_32 odometry_msg;
	r2p_msgs::Vector3_32 imu_msg;
	r2p_msgs::ImuRaw imu_raw_msg;
	ros::Publisher odometry_pub("odom", &odometry_msg);
	ros::Publisher imu_pub("imu", &imu_msg);
	ros::Publisher imu_raw_pub("imu_raw", &imu_raw_msg);
	systime_t last_sample;

	odom_msg.x = 0.0;
	odom_msg.y = 0.0;
	odom_msg.z = 0.0;

	(void) arg;
	chRegSetThreadName("rosserial_pub");

	nh.initNode();
	nh.advertise(odometry_pub);
	nh.advertise(imu_pub);
	nh.advertise(imu_raw_pub);

	for (;;) {
		last_sample = chTimeNow();

		odometry_msg.x = odometry_data.x;
		odometry_msg.y = odometry_data.y;
		odometry_msg.z = odometry_data.w;
		odometry_pub.publish(&odometry_msg);

		imu_msg.x = imu_data.roll;
		imu_msg.y = imu_data.pitch;
		imu_msg.z = imu_data.yaw;
		imu_pub.publish(&imu_msg);

		imu_raw_msg.linear_acceleration.x = imu_raw_data.acc_x;
		imu_raw_msg.linear_acceleration.y = imu_raw_data.acc_y;
		imu_raw_msg.linear_acceleration.z = imu_raw_data.acc_z;
		imu_raw_msg.angular_velocity.x = imu_raw_data.gyro_x;
		imu_raw_msg.angular_velocity.y = imu_raw_data.gyro_y;
		imu_raw_msg.angular_velocity.z = imu_raw_data.gyro_z;
		imu_raw_msg.magnetic_field.x = imu_raw_data.mag_x;
		imu_raw_msg.magnetic_field.y = imu_raw_data.mag_y;
		imu_raw_msg.magnetic_field.z = imu_raw_data.mag_z;
		imu_raw_pub.publish(&imu_raw_msg);

		nh.spinOnce();

		// Sleep for 50 milliseconds from last execution
		chThdSleepUntil(last_sample + MS2ST(50));
	}

	return CH_SUCCESS;
}


/*
 * ROS rosserial subscriber thread.
 */
void cmd_vel_cb( const geometry_msgs::Twist& cmd_vel_msg){
	r2p::Velocity3Msg * msgp;

	vel_node.set_enabled(true);

	// Motor setpoints
	if (vel_pub.alloc(msgp)) {
		msgp->x = cmd_vel_msg.linear.x;
		msgp->y = cmd_vel_msg.linear.y;
		msgp->w = cmd_vel_msg.angular.z;
		vel_pub.publish(*msgp);
	}

	vel_node.set_enabled(false);
}

void balcfg_cb( const r2p_msgs::PidParameters& PID_config_msg){
	r2p::PIDCfgMsg * msgp;

	balcfg_node.set_enabled(true);

	if (balcfg_pub.alloc(msgp)) {
		msgp->k = PID_config_msg.k;
		msgp->ti = PID_config_msg.ti;
		msgp->td = PID_config_msg.td;
		balcfg_pub.publish(*msgp);
	}

	balcfg_node.set_enabled(false);

}

void velcfg_cb( const r2p_msgs::PidParameters& PID_config_msg){
	r2p::PIDCfgMsg * msgp;

	velcfg_node.set_enabled(true);

	if (velcfg_pub.alloc(msgp)) {
		msgp->k = PID_config_msg.k;
		msgp->ti = PID_config_msg.ti;
		msgp->td = PID_config_msg.td;
		velcfg_pub.publish(*msgp);
	}

	velcfg_node.set_enabled(false);

}

msg_t rosserial_sub_thread(void * arg) {
	ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb );
	ros::Subscriber<r2p_msgs::PidParameters> balcfg_sub("balcfg", balcfg_cb );
	ros::Subscriber<r2p_msgs::PidParameters> velcfg_sub("velcfg", velcfg_cb );
	(void) arg;
	chRegSetThreadName("cmd_vel_sub");

	nh.initNode();
	nh.subscribe(cmd_vel_sub);
	nh.subscribe(balcfg_sub);
	nh.subscribe(velcfg_sub);

	for (;;) {
		nh.spinOnce();
		chThdSleepMilliseconds(5);
	}

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {
//	Thread *usb_shelltp = NULL;
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
//	shellInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = {"led"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, r2p_sub_node, NULL);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(4096), NORMALPRIO, rosserial_pub_thread, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO, rosserial_sub_thread, NULL);

	balcfg_node.advertise(balcfg_pub, "balcfg", r2p::Time::INFINITE);
	velcfg_node.advertise(velcfg_pub, "velcfg", r2p::Time::INFINITE);
	vel_node.advertise(vel_pub, "velocity", r2p::Time::INFINITE);

	for (;;) {
/*
		if (!usb_shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			usb_shelltp = shellCreate(&usb_shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(usb_shelltp)) {
			chThdRelease(usb_shelltp);
			usb_shelltp = NULL;
		}
*/
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
