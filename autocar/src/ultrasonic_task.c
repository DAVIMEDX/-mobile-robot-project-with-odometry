#include "ultrasonic_task.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" 


static float g_distance_cm = -1.0f;
static SemaphoreHandle_t g_ultrasonic_mutex = NULL;


#define ECHO_TIMEOUT_US 30000

static float measure_distance_cm(void) {

    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);

   
    uint64_t start_wait_time = time_us_64();
    while (gpio_get(ULTRASONIC_ECHO_PIN) == 0) {
        if (time_us_64() - start_wait_time > ECHO_TIMEOUT_US) {
            return -1.0f; 
        }
    }
    uint64_t pulse_start_time = time_us_64();


    while (gpio_get(ULTRASONIC_ECHO_PIN) == 1) {
        if (time_us_64() - pulse_start_time > ECHO_TIMEOUT_US) {
            return -2.0f;
        }
    }
    uint64_t pulse_end_time = time_us_64();

   
    uint64_t pulse_duration_us = pulse_end_time - pulse_start_time;
    
    // Fórmula: Distância (cm) = (Duração_µs * 0.0343) / 2
    // Ou a fórmula simplificada: Duração_µs / 58.2
    float distance = (float)pulse_duration_us / 58.2f;

    return distance;
}

void vUltrasonicTask(void *pvParameters) {
    const TickType_t update_delay = pdMS_TO_TICKS(40); 

    for (;;) {
        float distance = measure_distance_cm();
        printf("[ULTRA] Distância medida: %.1f cm\n", distance);


        if (g_ultrasonic_mutex != NULL && xSemaphoreTake(g_ultrasonic_mutex, portMAX_DELAY) == pdPASS) {
            g_distance_cm = distance;
            xSemaphoreGive(g_ultrasonic_mutex);
        }
        vTaskDelay(update_delay);
    }
}

static void ultrasonic_init_hardware(void) {
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);   

    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);

    g_ultrasonic_mutex = xSemaphoreCreateMutex();
    printf("[ULTRA] Sensor Ultrassonico (T:%d, E:%d) inicializado.\n", ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
}


void vStartUltrasonicTask(void) {
    ultrasonic_init_hardware();
    
    xTaskCreate(vUltrasonicTask,       
                "UltrasonicTask",      
                512,                
                NULL,              
                1,                 
                NULL);             
}

bool ultrasonic_get_distance_cm(float *distance) {
    if (g_ultrasonic_mutex != NULL && xSemaphoreTake(g_ultrasonic_mutex, pdMS_TO_TICKS(10)) == pdPASS) {
        *distance = g_distance_cm;
        xSemaphoreGive(g_ultrasonic_mutex);
        return true;
    }
    return false;
}