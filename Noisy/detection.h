#ifndef DETECTION_H
#define DETECTION_H

void sensors_init(void);

void proxthd(void);

uint8_t obstacle_detection(void);
void avoid_obstacle(void);

#endif /* DETECTION_H */
