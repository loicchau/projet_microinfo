#ifndef DETECTION_H
#define DETECTION_H

#define PROX_FRONT_RIGHT_F		0
#define PROX_FRONT_RIGHT_R		1
#define PROX_RIGHT				2
#define PROX_BACK_RIGHT			3
#define PROX_BACK_LEFT			4
#define PROX_LEFT				5
#define PROX_FRONT_LEFT_L		6
#define PROX_FRONT_LEFT_F		7

#define NB_PROX_SENSOR			8

void sensors_init(void);

void proxthd(void);

void obstacle_detection(float* sens_values);

#endif /* DETECTION_H */
