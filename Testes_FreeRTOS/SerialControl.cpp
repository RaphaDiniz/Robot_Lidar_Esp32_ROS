// SerialControl.cpp
#include "SerialControl.h"
#define LIMITE_TEMPO_BLOQUEIO 100

void taskLeituraSerial(void *parameter) {
  // Implementação da taskLeituraSerial
  for (;;) {
    if (Serial.available()) {
      char comando = Serial.read();
      xQueueSend(xQueueComandos, &comando, LIMITE_TEMPO_BLOQUEIO);
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay para não saturar o núcleo
  }
}
