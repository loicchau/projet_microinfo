#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <motors.h>
#include "i2c_bus.h"
#include <audio/microphone.h>
#include <detection.h>
#include <audio_processing.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init( & bus, & bus_lock, & bus_condvar);

    //inits the motors
    motors_init();

    //start the proximity sensors processing thread
    sensors_init();

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(100);
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
