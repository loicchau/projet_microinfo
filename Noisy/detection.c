#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <detection.h>
#include <stdbool.h>
#include <leds.h>


void sensors_init(void) {
    // Proximity sensors
    proximity_start();
    calibrate_ir();
    chThdSleepMilliseconds(100);
}

void obstacle_detection(float* prox_values) {

	prox_values[PROX_FRONT_RIGHT_F]=get_calibrated_prox(PROX_FRONT_RIGHT_F);
	prox_values[PROX_FRONT_RIGHT_R]=get_calibrated_prox(PROX_FRONT_RIGHT_R);
	prox_values[PROX_RIGHT]=get_calibrated_prox(PROX_RIGHT);
	prox_values[PROX_LEFT]=get_calibrated_prox(PROX_LEFT);
	prox_values[PROX_FRONT_LEFT_L]=get_calibrated_prox(PROX_FRONT_LEFT_L);
	prox_values[PROX_FRONT_LEFT_F]=get_calibrated_prox(PROX_FRONT_LEFT_F);

}

