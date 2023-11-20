// SerialControl.cpp
#include "SerialControl.h"


void taskLeituraSerial(void *parameter) {
    // Implementação da taskLeituraSerial
    for (;;) {
        if (Serial.available()) {
            char comando = Serial.read();
            xQueueSend(xQueueComandos, &comando, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay para não saturar o núcleo
    }
}
