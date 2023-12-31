// EncoderControl.cpp
#include "freertos/FreeRTOS.h"
#include "PinDefinitions.h"
#include "freertos/queue.h"
#include "Arduino.h"
#include "EncoderControl.h"
volatile long contadorEncoderA = 0;
volatile long contadorEncoderB = 0;
volatile long ultimoTempoInterrupcaoA = 0;
volatile long ultimoTempoInterrupcaoB = 0;
QueueHandle_t xQueueEncoderAData = 0;
QueueHandle_t xQueueEncoderBData = 0;

// Implementações das funções
void taskEncoderA(void *parameter) {//finalizada
    long contadorA;
    for (;;) {
        contadorA = contadorEncoderA;// Obtendo a contagem atual do encoder A
        Serial.print("Contador Encoder A: "); Serial.println(contadorA); // Para taskEncoderA
        xQueueSend(xQueueEncoderAData, &contadorA, portMAX_DELAY); // Enviando a contagem para a fila
        vTaskDelay(pdMS_TO_TICKS(100)); // Intervalo de 100ms entre cada leitura
    }
}

void taskEncoderB(void *parameter) {//finalizada
    long contadorB;
    for (;;) {
        contadorB = contadorEncoderB;// obtenha a contagem do encoder B
        Serial.print("Contador Encoder B: "); Serial.println(contadorB); 
        xQueueSend(xQueueEncoderBData, &contadorB, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void IRAM_ATTR interrupcaoEncoderA() {
  // Incrementa o contador do encoder A quando uma mudança é detectada
  contadorEncoderA++;
  // Atualiza o tempo da última interrupção
  ultimoTempoInterrupcaoA = millis();
}

void IRAM_ATTR interrupcaoEncoderB() {
  // Incrementa o contador do encoder B quando uma mudança é detectada
  contadorEncoderB++;
  // Atualiza o tempo da última interrupção
  ultimoTempoInterrupcaoB = millis();
}
