// RPMControl.h
#ifndef RPMCONTROL_H
#define RPMCONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern QueueHandle_t xQueueEncoderAData;
extern QueueHandle_t xQueueEncoderBData;
extern QueueHandle_t xQueueRpmResults;

// Protótipos das funções
void taskCalculoRPM(void *parameter);
float calcularRPM(long contadorAtual, long &contadorAnterior, long &tempoUltimaLeitura);

#endif
