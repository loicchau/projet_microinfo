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


void sensors_init(void) {
    // TOF sensor
    VL53L0X_start();
    // Proximity sensors
    proximity_start();
    calibrate_ir();
    chThdSleepMilliseconds(500);
}
