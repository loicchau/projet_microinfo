#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <detection.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD 15000
#define MIN_MAG_THRESHOLD_RIGHT 2200
#define MIN_MAG_THRESHOLD_LEFT 500
#define MIN_PROX_THRESHOLD 50
#define MAX_PROX_THRESHOLD 250

#define MIN_FREQ		16	//we don't analyze before this index to not use resources for nothing
#define FREQ_MOVE		19	//297Hz
#define MAX_FREQ		22	//we don't analyze after this index to not use resources for nothing

#define FREQ_MOVE_L		(FREQ_MOVE-2)
#define FREQ_MOVE_H		(FREQ_MOVE+2)


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* back){
	float prox_values[NB_PROX_SENSOR];
	float phase_average_left = 0, phase_average_right = 0;
	float mag_average_left = 0, mag_average_right = 0;
	//float mag_average_front = 0;
	volatile int16_t max_norm_index = -1;

	obstacle_detection(prox_values);

	if(prox_values[PROX_FRONT_RIGHT_R] > MIN_PROX_THRESHOLD && (prox_values[PROX_RIGHT] < prox_values[PROX_FRONT_RIGHT_R] ||
																prox_values[PROX_FRONT_RIGHT_R] < prox_values[PROX_FRONT_RIGHT_F])){
		left_motor_set_speed(-300);
		right_motor_set_speed(300);
		palWritePad(GPIOD, GPIOD_LED1, 1);
		palWritePad(GPIOD, GPIOD_LED3, 1);
		palWritePad(GPIOD, GPIOD_LED5, 1);
		palWritePad(GPIOD, GPIOD_LED7, 0);
	}
	else if(prox_values[PROX_RIGHT] > MIN_PROX_THRESHOLD && prox_values[PROX_RIGHT] > prox_values[PROX_FRONT_RIGHT_R]){
		if(prox_values[PROX_FRONT_RIGHT_R] > MAX_PROX_THRESHOLD){
			left_motor_set_speed(-300);
			right_motor_set_speed(300);
			palWritePad(GPIOD, GPIOD_LED1, 1);
			palWritePad(GPIOD, GPIOD_LED3, 1);
			palWritePad(GPIOD, GPIOD_LED5, 1);
			palWritePad(GPIOD, GPIOD_LED7, 0);
		}else{
			left_motor_set_speed(300);
			right_motor_set_speed(300);
			palWritePad(GPIOD, GPIOD_LED1, 0);
			palWritePad(GPIOD, GPIOD_LED3, 1);
			palWritePad(GPIOD, GPIOD_LED5, 1);
			palWritePad(GPIOD, GPIOD_LED7, 1);
		}
	}
	else if(prox_values[PROX_FRONT_LEFT_L] > MIN_PROX_THRESHOLD && (prox_values[PROX_LEFT] < prox_values[PROX_FRONT_LEFT_L] ||
																	prox_values[PROX_FRONT_LEFT_L] < prox_values[PROX_FRONT_LEFT_F])){
		left_motor_set_speed(300);
		right_motor_set_speed(-300);
		palWritePad(GPIOD, GPIOD_LED1, 1);
		palWritePad(GPIOD, GPIOD_LED3, 0);
		palWritePad(GPIOD, GPIOD_LED5, 1);
		palWritePad(GPIOD, GPIOD_LED7, 1);
	}
	else if(prox_values[PROX_LEFT] > MIN_PROX_THRESHOLD && prox_values[PROX_LEFT] > prox_values[PROX_FRONT_LEFT_L]){
		if(prox_values[PROX_FRONT_LEFT_L] > MAX_PROX_THRESHOLD){
			left_motor_set_speed(300);
			right_motor_set_speed(-300);
			palWritePad(GPIOD, GPIOD_LED1, 1);
			palWritePad(GPIOD, GPIOD_LED3, 1);
			palWritePad(GPIOD, GPIOD_LED5, 1);
			palWritePad(GPIOD, GPIOD_LED7, 0);
		}else{
			left_motor_set_speed(300);
			right_motor_set_speed(300);
			palWritePad(GPIOD, GPIOD_LED1, 0);
			palWritePad(GPIOD, GPIOD_LED3, 1);
			palWritePad(GPIOD, GPIOD_LED5, 1);
			palWritePad(GPIOD, GPIOD_LED7, 1);
		}
	}
	
	else{
		//search for the highest peak
		for(uint16_t i = 2*MIN_FREQ ; i <= 2*MAX_FREQ ; i+=2){
			phase_average_right += atan2(micRight_cmplx_input[i+1],micRight_cmplx_input[i])/(MAX_FREQ-MIN_FREQ);
			phase_average_left += atan2(micLeft_cmplx_input[i+1],micLeft_cmplx_input[i])/(MAX_FREQ-MIN_FREQ);
			mag_average_left += micLeft_output[i/2]/(MAX_FREQ-MIN_FREQ);
			mag_average_right += micRight_output[i/2]/(MAX_FREQ-MIN_FREQ);
			//mag_average_front += micFront_output[i]/(MAX_FREQ-MIN_FREQ);
			if(back[i/2] > MIN_VALUE_THRESHOLD){
				max_norm_index = i/2;
			}
		}
		//
		if(max_norm_index >= FREQ_MOVE_L && max_norm_index <= FREQ_MOVE_H){
			if(mag_average_left > mag_average_right + MIN_MAG_THRESHOLD_LEFT && phase_average_left > phase_average_right){//||
				left_motor_set_speed(-300);
				right_motor_set_speed(300);
				palWritePad(GPIOD, GPIOD_LED1, 1);
				palWritePad(GPIOD, GPIOD_LED3, 1);
				palWritePad(GPIOD, GPIOD_LED5, 1);
				palWritePad(GPIOD, GPIOD_LED7, 0);
			}
			else if(mag_average_left < mag_average_right - MIN_MAG_THRESHOLD_RIGHT && phase_average_left < phase_average_right){
				left_motor_set_speed(300);
				right_motor_set_speed(-300);
				palWritePad(GPIOD, GPIOD_LED1, 1);
				palWritePad(GPIOD, GPIOD_LED3, 0);
				palWritePad(GPIOD, GPIOD_LED5, 1);
				palWritePad(GPIOD, GPIOD_LED7, 1);
			}
			else{
				left_motor_set_speed(300);
				right_motor_set_speed(300);
				palWritePad(GPIOD, GPIOD_LED1, 0);
				palWritePad(GPIOD, GPIOD_LED3, 1);
				palWritePad(GPIOD, GPIOD_LED5, 1);
				palWritePad(GPIOD, GPIOD_LED7, 1);
			}
		}
		else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			palWritePad(GPIOD, GPIOD_LED1, 1);
			palWritePad(GPIOD, GPIOD_LED3, 1);
			palWritePad(GPIOD, GPIOD_LED5, 1);
			palWritePad(GPIOD, GPIOD_LED7, 1);
		}
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);



		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		nb_samples = 0;

		sound_remote(micBack_output);
	}
}
