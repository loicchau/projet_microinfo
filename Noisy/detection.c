#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <detection.h>
#include <stdbool.h>
#include <audio/play_melody.h>
#include <leds.h>


/*#define PINPROX_FRONT_RIGHT17		GPIOC, 2
#define PINPROX_FRONT_RIGHT49		GPIOC, 3
#define PINPROX_RIGHT				GPIOC, 4
#define PINPROX_BACK_RIGHT			GPIOC, 5
#define PINPROX_BACK_LEFT			GPIOB, 1
#define PINPROX_LEFT				GPIOB, 0
#define PINPROX_FRONT_LEFT17		GPIOC, 0
#define PINPROX_FRONT_LEFT49		GPIOC, 1*/

#define PROX_FRONT_RIGHT_F		1
#define PROX_FRONT_RIGHT_R		2
#define PROX_RIGHT				3
#define PROX_BACK_RIGHT			4
#define PROX_BACK_LEFT			5
#define PROX_LEFT				6
#define PROX_FRONT_LEFT_L		7
#define PROX_FRONT_LEFT_F		8

#define NB_PROX_SENSOR			9

static float prox_value[NB_PROX_SENSOR];
//bool motor_stop=false;

void sensors_init(void) {
    // TOF sensor
    VL53L0X_start();
    // Proximity sensors
    proximity_start();
    calibrate_ir();
    chThdSleepMilliseconds(500);
}

static THD_WORKING_AREA(waProxThread, 512);
static THD_FUNCTION(ProxThread, arg) {
	 (void) arg;
	 chRegSetThreadName(__FUNCTION__);

	 while(1){
		 prox_value[PROX_FRONT_RIGHT_F]=get_calibrated_prox(PROX_FRONT_RIGHT_F);
		 prox_value[PROX_FRONT_RIGHT_R]=get_calibrated_prox(PROX_FRONT_RIGHT_R);
		 prox_value[PROX_RIGHT]=get_calibrated_prox(PROX_RIGHT);
		 prox_value[PROX_BACK_RIGHT]=get_calibrated_prox(PROX_BACK_RIGHT);
		 prox_value[PROX_BACK_LEFT]=get_calibrated_prox(PROX_BACK_LEFT);
		 prox_value[PROX_LEFT]=get_calibrated_prox(PROX_LEFT);
		 prox_value[PROX_FRONT_LEFT_L]=get_calibrated_prox(PROX_FRONT_LEFT_L);
		 prox_value[PROX_FRONT_LEFT_F]=get_calibrated_prox(PROX_FRONT_LEFT_F);

		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_FRONT_RIGHT_F]= %f \n", prox_value[PROX_FRONT_RIGHT_F]);
		/* chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_FRONT_RIGHT_R]= %f \n", prox_value[PROX_FRONT_RIGHT_R]);
		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_RIGHT]= %f \n", prox_value[PROX_RIGHT]);
		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_BACK_RIGHT]= %f \n", prox_value[PROX_BACK_RIGHT]);
		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_BACK_LEFT]= %f \n", prox_value[PROX_BACK_LEFT]);
		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_LEFT]= %f \n", prox_value[PROX_LEFT]);
		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_FRONT_LEFT_L]= %f \n", prox_value[PROX_FRONT_LEFT_L]);
		 chprintf((BaseSequentialStream *)&SD3,"prox_value[PROX_FRONT_LEFT_F]= %f \n", prox_value[PROX_FRONT_LEFT_F]);*/

		// chThdSleepMilliseconds(500);
	 }
}

void proxthd(void) {
	chThdCreateStatic(waProxThread, sizeof(waProxThread), NORMALPRIO, ProxThread, NULL);
}

void obstacle_detection(bool motor_stop) {
	if (prox_value[PROX_FRONT_RIGHT_F]>10 || prox_value[PROX_FRONT_RIGHT_R]>10 || prox_value[PROX_RIGHT]>10 || prox_value[PROX_BACK_RIGHT]>10 || prox_value[PROX_BACK_LEFT]>10 || prox_value[PROX_LEFT]>10 || prox_value[PROX_FRONT_LEFT_L]>10 || prox_value[PROX_FRONT_LEFT_F]>10) {
		motor_stop = true;
	}
	else {
		motor_stop = false;
	}
	return motor_stop;

}
