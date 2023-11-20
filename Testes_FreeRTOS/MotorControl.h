// MotorControl.h
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "Arduino.h"

extern const int canalMotorA1;
extern const int canalMotorA2;
extern const int canalMotorB1;
extern const int canalMotorB2;

// Se você tiver variáveis globais relacionadas ao controle dos motores, declare-as aqui
extern QueueHandle_t xQueueComandos;
extern QueueHandle_t xQueuePWMValues;

// Protótipo da função
void taskControleMotores(void *parameter);

#endif
