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


//static float prox_value[NB_PROX_SENSOR];

void sensors_init(void) {
    // TOF sensor
    VL53L0X_start();
    // Proximity sensors
    proximity_start();
    calibrate_ir();
    chThdSleepMilliseconds(500);
}
/*
static THD_WORKING_AREA(waProxThread, 1024);
static THD_FUNCTION(ProxThread, arg) { //changer le nom par sensorthread
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

		 chThdSleepMilliseconds(20);
	 }
}

void proxthd(void) {
	chThdCreateStatic(waProxThread, sizeof(waProxThread), NORMALPRIO, ProxThread, NULL);
}*/

void obstacle_detection(float* sens_values, uint16_t* dist_from_obstacle) {

	sens_values[PROX_FRONT_RIGHT_F]=get_calibrated_prox(PROX_FRONT_RIGHT_F);
	sens_values[PROX_FRONT_RIGHT_R]=get_calibrated_prox(PROX_FRONT_RIGHT_R);
	sens_values[PROX_RIGHT]=get_calibrated_prox(PROX_RIGHT);
	sens_values[PROX_BACK_RIGHT]=get_calibrated_prox(PROX_BACK_RIGHT);
	sens_values[PROX_BACK_LEFT]=get_calibrated_prox(PROX_BACK_LEFT);
	sens_values[PROX_LEFT]=get_calibrated_prox(PROX_LEFT);
	sens_values[PROX_FRONT_LEFT_L]=get_calibrated_prox(PROX_FRONT_LEFT_L);
	sens_values[PROX_FRONT_LEFT_F]=get_calibrated_prox(PROX_FRONT_LEFT_F);

	*dist_from_obstacle = VL53L0X_get_dist_mm();
}

