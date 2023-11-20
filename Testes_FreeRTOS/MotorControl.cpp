// MotorControl.cpp
#include "MotorControl.h"
#include "PinDefinitions.h"

void taskControleMotores(void *parameter) {
    float outputA;
    float outputB;
    for (;;) {
        if (xQueueReceive(xQueuePWMValues, &outputA, portMAX_DELAY) && 
        xQueueReceive(xQueuePWMValues, &outputB, portMAX_DELAY)) {
            Serial.println("PWMCHegado:");
            Serial.println(outputA);
            ledcWrite(canalMotorB1, outputA);
            ledcWrite(canalMotorB2, 0); 
            ledcWrite(canalMotorA1, outputB);
            ledcWrite(canalMotorA2, 0); 
            Serial.print("Motores ativos com PWM: ");
            Serial.println(outputB);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
