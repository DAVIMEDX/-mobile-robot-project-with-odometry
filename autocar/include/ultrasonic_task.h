// include/ultrasonic_task.h

#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <stdbool.h>

#define ULTRASONIC_TRIG_PIN 27
#define ULTRASONIC_ECHO_PIN 28


void vStartUltrasonicTask(void);
bool ultrasonic_get_distance_cm(float *distance);

#endif