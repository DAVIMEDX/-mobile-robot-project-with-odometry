// encoder_task.h

#ifndef ENCODER_TASK_H
#define ENCODER_TASK_H

#include <stdbool.h>

#define ENCODER_LEFT_PIN  2
#define ENCODER_RIGHT_PIN 3

#define ENCODER_PULSES_PER_REVOLUTION    20    

#define WHEEL_DIAMETER_CM                6.5f  

#define WHEELBASE_CM                     15.0f 



typedef struct {
    float velocity_left_cm_s;
    float velocity_right_cm_s;
    long long total_pulses_left;
    long long total_pulses_right;
} odometry_data_t;


void vStartEncoderTask(void);
bool encoder_get_odometry(odometry_data_t *data);

#endif