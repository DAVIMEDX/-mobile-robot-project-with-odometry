#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "encoder_task.h"    
#include "motor_control.h"   
#include "ultrasonic_task.h" 

// --- CONFIGURAÇÕES ---
#define OBSTACLE_DISTANCE_CM 10.0f
#define FORWARD_SPEED 100
#define TURN_SPEED    100  

typedef enum {
    STATE_MOVING_FORWARD,
    STATE_TURNING_180
} RobotState;

// Variáveis para guardar o início do giro das DUAS rodas
static long long g_turn_start_pulse_L = 0;
static long long g_turn_start_pulse_R = 0;
static long long g_turn_target_pulses = 0;

// =======================================================
// CÁLCULO DE PULSOS
// =======================================================
static long long calculate_pulses_for_180_turn() {
    float wheel_circumference = (float)M_PI * WHEEL_DIAMETER_CM;
    float cm_per_pulse = wheel_circumference / (float)ENCODER_PULSES_PER_REVOLUTION;
    
    // Distância de arco para 180 graus
    float turn_distance_cm = ((float)M_PI * WHEELBASE_CM) / 2.0f;
    
    long long pulses = (long long)(turn_distance_cm / cm_per_pulse);
    
    // Proteção: Valor mínimo para evitar erros de cálculo
    if (pulses < 15) pulses = 25; 
    
    printf("[CÉREBRO] Meta de Giro calculada: %lld pulsos.\n", pulses);
    return pulses;
}

void vTaskTest(void *pvParameters) {
    printf("[CÉREBRO] Task 'Vai-e-Vem' (Versão Média + Timeout) iniciada.\n");
    
    const TickType_t loop_delay = pdMS_TO_TICKS(50); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    RobotState state = STATE_MOVING_FORWARD;
    odometry_data_t odometry;
    float distance_cm = 0.0f;
    bool sensor_ok = false;
    
    // Timer de segurança (Timeout) para evitar travamento eterno
    int safety_timer = 0;

    // Calcula meta inicial
    g_turn_target_pulses = calculate_pulses_for_180_turn();

    for (;;) {
        encoder_get_odometry(&odometry);
        sensor_ok = ultrasonic_get_distance_cm(&distance_cm);

        switch (state) {
            
            
            case STATE_MOVING_FORWARD:
                motor_set_speed(FORWARD_SPEED, FORWARD_SPEED); 

                // Lógica de Detecção
                if (sensor_ok && distance_cm > 0.1f && distance_cm < OBSTACLE_DISTANCE_CM) {
                    printf("[CÉREBRO] Parede a %.1f cm! Parando.\n", distance_cm);
                    
                    motor_set_speed(0, 0); 
                    vTaskDelay(pdMS_TO_TICKS(300)); 
                    
                    g_turn_start_pulse_L = odometry.total_pulses_left;
                    g_turn_start_pulse_R = odometry.total_pulses_right;
                    
                    safety_timer = 0;
                    state = STATE_TURNING_180;
                }
                break;

            case STATE_TURNING_180:
                motor_set_speed(-TURN_SPEED, TURN_SPEED);

                long long delta_L = llabs(odometry.total_pulses_left - g_turn_start_pulse_L);
                long long delta_R = llabs(odometry.total_pulses_right - g_turn_start_pulse_R);
                long long pulses_turned_avg = (delta_L + delta_R) / 2;

                safety_timer++;
                if (safety_timer % 10 == 0) {
                    printf("[GIRO] L:%lld R:%lld | Med:%lld / Meta:%lld\n", 
                           delta_L, delta_R, pulses_turned_avg, g_turn_target_pulses);
                }

                if (pulses_turned_avg >= g_turn_target_pulses || safety_timer > 80) {
                    
                    if (safety_timer > 80) {
                        printf("[CÉREBRO] Timeout! Destravando giro forçadamente.\n");
                    } else {
                        printf("[CÉREBRO] Giro concluído com sucesso!\n");
                    }
                    
                    motor_set_speed(0, 0); 
                    vTaskDelay(pdMS_TO_TICKS(300));
                    
                    state = STATE_MOVING_FORWARD;
                }
                break;
        }

        vTaskDelayUntil(&xLastWakeTime, loop_delay);
    }
}