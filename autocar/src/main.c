#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "encoder_task.h"
#include "motor_control.h"
#include "ultrasonic_task.h"


void vTaskTest(void *pvParameters);

int main() {
    stdio_init_all();
    printf("\n--- Robô Vai-e-Vem (FreeRTOS) ---\n");
    printf("[MAIN] A inicializar hardware...\n");

    vStartEncoderTask();
    vStartMotorControlTask();
    vStartUltrasonicTask();
    
    xTaskCreate(vTaskTest,
                "BrainTask", 
                1024,
                NULL,
                3, 
                NULL);
    
    vTaskStartScheduler();

    for (;;) { __wfi(); }
    return 0;
}