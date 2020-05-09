#include "ch.h"
#include "hal.h"
#include <main.h>
#include <sensors/proximity.h>
#include <detection.h>


void sensors_init(void) {
    // Proximity sensors
    proximity_start();
    calibrate_ir();
    chThdSleepMilliseconds(100);
}

// Store the calibrated sensor values in the prox_values array
void obstacle_detection(int* prox_values) {

	prox_values[PROX_FRONT_RIGHT_F]=get_calibrated_prox(PROX_FRONT_RIGHT_F);
	prox_values[PROX_FRONT_RIGHT_R]=get_calibrated_prox(PROX_FRONT_RIGHT_R);
	prox_values[PROX_RIGHT]=get_calibrated_prox(PROX_RIGHT);
	// We don't use the back sensors
	prox_values[PROX_BACK_RIGHT] = 0;
	prox_values[PROX_BACK_LEFT] = 0;
	prox_values[PROX_LEFT]=get_calibrated_prox(PROX_LEFT);
	prox_values[PROX_FRONT_LEFT_L]=get_calibrated_prox(PROX_FRONT_LEFT_L);
	prox_values[PROX_FRONT_LEFT_F]=get_calibrated_prox(PROX_FRONT_LEFT_F);

}

