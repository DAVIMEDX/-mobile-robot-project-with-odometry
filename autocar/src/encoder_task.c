// encoder_task.c

#include "encoder_task.h"

// Includes do SDK
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

// Includes do FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" 

// =======================================================
// VARIÁVEIS COMPARTILHADAS E RTOS
// =======================================================

static odometry_data_t g_odometry_data = {0};
static SemaphoreHandle_t g_odometry_mutex = NULL;

// Variáveis de Interrupção (Contagem)
static volatile long long g_pulses_left = 0;
static volatile long long g_pulses_right = 0;


// =======================================================
// MANIPULADOR DE INTERRUPÇÃO (ISR)
// =======================================================

// A ISR só deve fazer o mínimo: contar o pulso.
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) { 
        if (gpio == ENCODER_LEFT_PIN) {
            g_pulses_left++;
        } 
        else if (gpio == ENCODER_RIGHT_PIN) {
            g_pulses_right++;
        }
        // Nota: O cálculo de velocidade por pulso individual (g_last_time_left_us)
        // foi removido da ISR, pois a Task (vEncoderTask) fará o cálculo em bloco (a cada 100ms).
    }
}


// =======================================================
// FUNÇÕES AUXILIARES DE INICIALIZAÇÃO
// =======================================================

static void encoder_init_hardware(void) {
    // Inicialização e Pull-up para leitura do sensor
    gpio_init(ENCODER_LEFT_PIN);
    gpio_set_dir(ENCODER_LEFT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_LEFT_PIN); 
    
    gpio_init(ENCODER_RIGHT_PIN);
    gpio_set_dir(ENCODER_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_RIGHT_PIN);
    
    // Configuração das Interrupções com a callback
    gpio_set_irq_enabled_with_callback(
        ENCODER_LEFT_PIN, 
        GPIO_IRQ_EDGE_FALL, 
        true, 
        &gpio_callback
    );
    
    gpio_set_irq_enabled_with_callback(
        ENCODER_RIGHT_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &gpio_callback
    );

    
    g_odometry_mutex = xSemaphoreCreateMutex();
    printf("[ENC] Encoders inicializados e protegidos por Mutex.\n");
}


// =======================================================
// FUNÇÃO DE CÁLCULO DE VELOCIDADE (Auxiliar)
// =======================================================

// Nota: Esta função agora aceita ponteiros separados para o tempo de cálculo,
// permitindo chamadas independentes dentro da vEncoderTask.
static float calculate_velocity_cm_s(long long current_pulses, long long *last_pulses, 
                                     uint64_t current_time_us, uint64_t *last_calc_time_us) {
    
    long long delta_pulses = current_pulses - *last_pulses;
    uint64_t delta_time_us = current_time_us - *last_calc_time_us;
    
    if (delta_time_us == 0 || delta_pulses == 0) {
        // Se o robô está parado ou o tempo de amostragem é zero
        if (delta_pulses == 0) return 0.0f; 
        // Se delta_time_us é zero, evita divisão por zero
        return 0.0f;
    }

    // Fórmulas de Conversão:
    float wheel_circumference = (float)M_PI * WHEEL_DIAMETER_CM;
    float distance_cm = (float)delta_pulses * (wheel_circumference / ENCODER_PULSES_PER_REVOLUTION);
    
    float delta_time_s = (float)delta_time_us / 1000000.0f;
    float velocity = distance_cm / delta_time_s;

    // Atualiza o estado para o próximo cálculo
    *last_pulses = current_pulses;
    *last_calc_time_us = current_time_us; // <<< ESSENCIAL: Atualiza o tempo

    return velocity;
}


// =======================================================
// TAREFA PRINCIPAL (vEncoderTask)
// =======================================================

void vEncoderTask(void *pvParameters) {
    const TickType_t update_delay = pdMS_TO_TICKS(100); // Frequência de atualização de 10 Hz
    
    // Variáveis internas para cálculo de velocidade (AGORA SEPARADAS)
    long long last_pulses_left = 0;
    long long last_pulses_right = 0;
    
    // VARIÁVEIS DE TEMPO SEPARADAS PARA CADA RODA (CORREÇÃO CRÍTICA)
    uint64_t last_calc_time_left_us = time_us_64();
    uint64_t last_calc_time_right_us = time_us_64(); 

    for (;;) {
        // Garante que o Mutex foi inicializado antes de tentar usá-lo
        if (g_odometry_mutex != NULL && xSemaphoreTake(g_odometry_mutex, portMAX_DELAY) == pdPASS) {
            
            uint64_t current_time_us = time_us_64();
            
            // --- CÁLCULO RODA ESQUERDA ---
            g_odometry_data.velocity_left_cm_s = calculate_velocity_cm_s(
                g_pulses_left, &last_pulses_left, current_time_us, &last_calc_time_left_us);
                
            g_odometry_data.total_pulses_left = g_pulses_left;
            
            // --- CÁLCULO RODA DIREITA ---
            g_odometry_data.velocity_right_cm_s = calculate_velocity_cm_s(
                g_pulses_right, &last_pulses_right, current_time_us, &last_calc_time_right_us);
                
            g_odometry_data.total_pulses_right = g_pulses_right;
            
            // Saída de debug (Pode ser removida após o teste)
            printf("[ENC] L: %lld p, %.2f cm/s | R: %lld p, %.2f cm/s\n",
                   g_odometry_data.total_pulses_left, g_odometry_data.velocity_left_cm_s,
                   g_odometry_data.total_pulses_right, g_odometry_data.velocity_right_cm_s);

            xSemaphoreGive(g_odometry_mutex);
        }
        
        vTaskDelay(update_delay);
    }
}


// =======================================================
// FUNÇÕES PÚBLICAS (vStart e GetOdometry)
// =======================================================

void vStartEncoderTask(void) {
    encoder_init_hardware(); // Inicializa hardware e Mutex
    
    xTaskCreate(vEncoderTask,       
                "EncoderTask",      
                1024,                
                NULL,              
                1,                 
                NULL);             
}


bool encoder_get_odometry(odometry_data_t *data) {
    if (g_odometry_mutex != NULL && xSemaphoreTake(g_odometry_mutex, pdMS_TO_TICKS(10)) == pdPASS) {
        *data = g_odometry_data;
        xSemaphoreGive(g_odometry_mutex);
        return true;
    }
    return false;
}