#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_E_PWM_PIN  19 //ENA
#define MOTOR_E_IN1_PIN  7 
#define MOTOR_E_IN2_PIN  8

#define MOTOR_D_PWM_PIN  18 //ENB
#define MOTOR_D_IN3_PIN  10
#define MOTOR_D_IN4_PIN  12  


void motor_control_init(void);
void vStartMotorControlTask(void);

void motor_set_speed(int8_t speed_left, int8_t speed_right); 

#endif 