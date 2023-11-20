// EncoderControl.h
#ifndef ENCODERCONTROL_H
#define ENCODERCONTROL_H

// Inclua aqui as bibliotecas necessárias
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern QueueHandle_t xQueueEncoderAdata;
extern QueueHandle_t xQueueEncoderBdata;
// Declaração de variáveis globais externas, se necessário
extern volatile long contadorEncoderA;
extern volatile long contadorEncoderB;
extern volatile long ultimoTempoInterrupcaoA;
extern volatile long ultimoTempoInterrupcaoB;

// Protótipos de funções
void taskEncoderA(void *parameter);
void taskEncoderB(void *parameter);
void interrupcaoEncoderA();
void interrupcaoEncoderB();



#endif
