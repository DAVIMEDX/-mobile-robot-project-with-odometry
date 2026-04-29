#include "motor_control.h"


#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h> 
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#define PWM_FREQUENCY_KHZ   5
#define PWM_WRAP            (125000000 / PWM_FREQUENCY_KHZ / 1000)

typedef struct {
    int8_t speed_left;
    int8_t speed_right;
} motor_command_t;

static QueueHandle_t motor_command_queue = NULL;

static uint slice_num_e;
static uint slice_num_d;


// Configura o sentido da rotação do motor
static void motor_set_direction(int8_t speed, uint in1_pin, uint in2_pin) {
    if (speed > 0) { 
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    } else if (speed < 0) { 
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    } else { 
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 1);
    }
}



static void motor_set_duty(uint gpio_pin, uint slice_num, int8_t speed) {
    uint16_t duty = (uint16_t)((float)abs(speed) * PWM_WRAP / 100.0f);
    
    uint chan = pwm_gpio_to_channel(gpio_pin);
    
    pwm_set_chan_level(slice_num, chan, duty);
}


void vMotorControlTask(void *pvParameters) {
    motor_command_t cmd;
    if (motor_command_queue == NULL) {
        printf("[MOTOR] Erro ao criar Queue! Task deletada.\n");
        vTaskDelete(NULL);
    }

    printf("[MOTOR] Task de Controle Iniciada.\n");

    for (;;) {
        // Espera por um novo comando. Receber o comando é o ponto de ativação desta Task.
        if (xQueueReceive(motor_command_queue, &cmd, portMAX_DELAY) == pdPASS) {
            
            // --- Motor Esquerdo ---
            motor_set_direction(cmd.speed_left, MOTOR_E_IN1_PIN, MOTOR_E_IN2_PIN);
            motor_set_duty(MOTOR_E_PWM_PIN, slice_num_e, cmd.speed_left); 

            // --- Motor Direito ---
            motor_set_direction(cmd.speed_right, MOTOR_D_IN3_PIN, MOTOR_D_IN4_PIN);
            motor_set_duty(MOTOR_D_PWM_PIN, slice_num_d, cmd.speed_right); 

            printf("[MOTOR] Comando executado: E: %d%%, D: %d%%\n", cmd.speed_left, cmd.speed_right);
        }
    }
}


void motor_control_init(void) {
    // Configuração dos pinos digitais (IN1, IN2, IN3, IN4) ---
    gpio_init(MOTOR_E_IN1_PIN); gpio_set_dir(MOTOR_E_IN1_PIN, GPIO_OUT);
    gpio_init(MOTOR_E_IN2_PIN); gpio_set_dir(MOTOR_E_IN2_PIN, GPIO_OUT);
    gpio_init(MOTOR_D_IN3_PIN); gpio_set_dir(MOTOR_D_IN3_PIN, GPIO_OUT);
    gpio_init(MOTOR_D_IN4_PIN); gpio_set_dir(MOTOR_D_IN4_PIN, GPIO_OUT);

    //Configuração dos pinos PWM (ENA, ENB) ---
    gpio_set_function(MOTOR_E_PWM_PIN, GPIO_FUNC_PWM);
    slice_num_e = pwm_gpio_to_slice_num(MOTOR_E_PWM_PIN);
    
    gpio_set_function(MOTOR_D_PWM_PIN, GPIO_FUNC_PWM);
    slice_num_d = pwm_gpio_to_slice_num(MOTOR_D_PWM_PIN);

    //Configura a frequência e o contador máximo (WRAP)
    pwm_set_wrap(slice_num_e, PWM_WRAP);
    pwm_set_wrap(slice_num_d, PWM_WRAP);

    //Inicia o PWM e garante que os motores comecem parados
    pwm_set_enabled(slice_num_e, true);
    pwm_set_enabled(slice_num_d, true);

    motor_set_speed(0, 0);
}

// Função pública para enviar comandos de velocidade para o motor (usando a Queue)
void motor_set_speed(int8_t speed_left, int8_t speed_right) {
    motor_command_t cmd;
    cmd.speed_left = speed_left;
    cmd.speed_right = speed_right;
    
    if (motor_command_queue != NULL) {
        xQueueOverwrite(motor_command_queue, &cmd);
    }
}

void vStartMotorControlTask(void) {
    motor_command_queue = xQueueCreate(1, sizeof(motor_command_t));
    motor_control_init();
    
    xTaskCreate(vMotorControlTask,       
                "MotorControl",      
                512,                
                NULL,              
                2,               
                NULL);             
}