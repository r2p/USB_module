#include <cstdio>

#include "ch.h"
#include "hal.h"
#include "ff.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/std_msgs.hpp>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "USB"
#endif



static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);


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
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
FATFS MMC_FS;

/**
 * MMC driver instance.
 */
MMCDriver MMCD1;

/* FS mounted and ready.*/
static bool_t fs_ready = FALSE;

/* Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig hs_spicfg = {NULL, GPIOB, GPIOB_SPI_CS, 0, 0};

/* Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig ls_spicfg = {NULL, GPIOB, GPIOB_SPI_CS,
                              SPI_CR1_BR_2 | SPI_CR1_BR_1, 0};

/* MMC/SD over SPI driver configuration.*/
static MMCConfig mmccfg = {&SPID2, &ls_spicfg, &hs_spicfg};

/* Generic large buffer.*/
uint8_t fbuff[1024];

int free_space = 0;

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

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);
	sprintf(msgp->data, "\n\n"R2P_MODULE_NAME" module [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
	pub.publish(msgp);

	chThdSleepMilliseconds(500);

	if (fs_ready) {
		while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);
		sprintf(msgp->data, "SD card OK (%dKb free)", free_space / 1000);
		pub.publish(msgp);
		chThdSleepMilliseconds(100);

	} else {
		while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);
		sprintf(msgp->data, "SD card FAIL");
		pub.publish(msgp);
		chThdSleepMilliseconds(100);
	}

	return CH_SUCCESS;
}


/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	mmcObjectInit(&MMCD1);
	mmcStart(&MMCD1, &mmccfg);

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = {"led"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, test_pub_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(100));

		if (!fs_ready && (palReadPad(GPIOA, GPIOA_SD_CD) == PAL_LOW)) {
			FRESULT err;
			uint32_t clusters;
			FATFS *fsp;

			if (mmcConnect(&MMCD1)) {
				continue;
			}

			err = f_mount(0, &MMC_FS);
			if (err != FR_OK) {
				mmcDisconnect(&MMCD1);
				continue;
			}

			fs_ready = TRUE;

			if (f_getfree("/", &clusters, &fsp) == FR_OK) {
				free_space = clusters * (uint32_t)MMC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE;
			} else {
				free_space = 0;
			}
		}

		if (fs_ready && (palReadPad(GPIOA, GPIOA_SD_CD) == PAL_HIGH)){
			mmcDisconnect(&MMCD1);
			fs_ready = FALSE;
		}

		if (fs_ready){
			palSetPad(GPIOA, GPIOA_SD_LED);
		} else {
			palClearPad(GPIOA, GPIOA_SD_LED);
		}
	}

	return CH_SUCCESS;
}
}
