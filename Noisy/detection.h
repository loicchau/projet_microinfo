#ifndef DETECTION_H
#define DETECTION_H

#define PROX_FRONT_RIGHT_F		0 //IR1
#define PROX_FRONT_RIGHT_R		1 //IR2
#define PROX_RIGHT				2 //IR3
#define PROX_BACK_RIGHT			3 //IR4
#define PROX_BACK_LEFT			4 //IR5
#define PROX_LEFT				5 //IR6
#define PROX_FRONT_LEFT_L		6 //IR7
#define PROX_FRONT_LEFT_F		7 //IR8

#define NB_PROX_SENSOR			8

void sensors_init(void);

void obstacle_detection(int* prox_values);

#endif /* DETECTION_H */
