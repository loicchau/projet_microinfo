#include "ch.h"
#include "hal.h"
#include <main.h>

#include <motors.h>
#include <detection.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>


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

#define MIN_VALUE_THRESHOLD 30000
#define MIN_MAG_THRESHOLD_RIGHT 2200
#define MIN_MAG_THRESHOLD_LEFT 500
#define MIN_PROX_THRESHOLD 50
#define MAX_PROX_THRESHOLD 250

#define MIN_FREQ		14	//we don't analyze before this index to not use resources for nothing
#define FREQ_MOVE		19	//297Hz
#define MAX_FREQ		24	//we don't analyze after this index to not use resources for nothing

#define FREQ_MOVE_L		(FREQ_MOVE-1)
#define FREQ_MOVE_H		(FREQ_MOVE+1)

//Final state machine to determine how to move
//First detects if there is an obstacle, otherwise follows the sound source
void sound_remote(float* back, float* front){

	int prox_values[NB_PROX_SENSOR];
	float phase_average_left = 0, phase_average_right = 0;
	float mag_average_left = 0, mag_average_right = 0;
	int16_t max_norm_index = 0;

	// Store the values collected by the IR sensors in prox_values
	obstacle_detection(prox_values);

	// Search for the highest peak and compute an average of the magnitude and the phase
	for(uint16_t i = 2*MIN_FREQ ; i <= 2*MAX_FREQ ; i+=2){

		phase_average_right += atan2(micRight_cmplx_input[i+1],micRight_cmplx_input[i])/(MAX_FREQ-MIN_FREQ);
		phase_average_left += atan2(micLeft_cmplx_input[i+1],micLeft_cmplx_input[i])/(MAX_FREQ-MIN_FREQ);
		mag_average_left += micLeft_output[i/2]/(MAX_FREQ-MIN_FREQ);
		mag_average_right += micRight_output[i/2]/(MAX_FREQ-MIN_FREQ);

		if(back[i/2] > MIN_VALUE_THRESHOLD || front[i/2] > MIN_VALUE_THRESHOLD){
			max_norm_index = i/2;
		}
	}

	// Start of the finite state machine
	if(max_norm_index){
		if(prox_values[PROX_FRONT_RIGHT_R] > MIN_PROX_THRESHOLD && (prox_values[PROX_RIGHT] < prox_values[PROX_FRONT_RIGHT_R] ||
																	prox_values[PROX_FRONT_RIGHT_R] < prox_values[PROX_FRONT_RIGHT_F])){
			//turn left
			left_motor_set_speed(-300);
			right_motor_set_speed(300);
			writeLED(1,1,1,0);
		}
		else if(prox_values[PROX_RIGHT] > MIN_PROX_THRESHOLD && prox_values[PROX_RIGHT] > prox_values[PROX_FRONT_RIGHT_R]){
			if(prox_values[PROX_FRONT_RIGHT_R] > MAX_PROX_THRESHOLD){
				//turn left
				left_motor_set_speed(-300);
				right_motor_set_speed(300);
				writeLED(1,1,1,0);
			}
			else{
				//continue straight
				left_motor_set_speed(300);
				right_motor_set_speed(300);
				writeLED(0,1,1,1);
			}
		}
		else if(prox_values[PROX_FRONT_LEFT_L] > MIN_PROX_THRESHOLD && (prox_values[PROX_LEFT] < prox_values[PROX_FRONT_LEFT_L] ||
																		prox_values[PROX_FRONT_LEFT_L] < prox_values[PROX_FRONT_LEFT_F])){
			//turn right
			left_motor_set_speed(300);
			right_motor_set_speed(-300);
			writeLED(1,0,1,1);
		}
		else if(prox_values[PROX_LEFT] > MIN_PROX_THRESHOLD && prox_values[PROX_LEFT] > prox_values[PROX_FRONT_LEFT_L]){
			if(prox_values[PROX_FRONT_LEFT_L] > MAX_PROX_THRESHOLD){
				//turn right
				left_motor_set_speed(300);
				right_motor_set_speed(-300);
				writeLED(1,1,1,0);
			}
			else{
				//continue straight
				left_motor_set_speed(300);
				right_motor_set_speed(300);
				writeLED(0,1,1,1);
			}
		}


		else{
			// Follows the sound source
			if(max_norm_index >= FREQ_MOVE_L && max_norm_index <= FREQ_MOVE_H){
				if(mag_average_left > mag_average_right + MIN_MAG_THRESHOLD_LEFT && phase_average_left < phase_average_right){
					//turn left
					left_motor_set_speed(-300);
					right_motor_set_speed(300);
					writeLED(1,1,1,0);
				}
				else if(mag_average_left < mag_average_right - MIN_MAG_THRESHOLD_RIGHT && phase_average_left > phase_average_right){
					//turn right
					left_motor_set_speed(300);
					right_motor_set_speed(-300);
					writeLED(1,0,1,1);
				}
				else{
					//continue straight
					left_motor_set_speed(300);
					right_motor_set_speed(300);
					writeLED(0,1,1,1);
				}
			}
			else{
				//stop
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				writeLED(1,1,1,1);
			}
		}
	}
	else{
		//stop
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		writeLED(1,1,1,1);
	}
}

// Fonction to turn the LEDs on or off
// 1 = off, 0 = on
void writeLED (uint8_t led1, uint8_t led3, uint8_t led5, uint8_t led7){
	palWritePad(GPIOD, GPIOD_LED1, led1);
	palWritePad(GPIOD, GPIOD_LED3, led3);
	palWritePad(GPIOD, GPIOD_LED5, led5);
	palWritePad(GPIOD, GPIOD_LED7, led7);
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

		//Call the finite state machine
		sound_remote(micBack_output, micFront_output);
	}
}
