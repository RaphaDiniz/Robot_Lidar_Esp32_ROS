// PIDControl.h
#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern QueueHandle_t xQueuePWMValues;
extern QueueHandle_t xQueueRpmResults;

// Variáveis globais do PID
extern float Kp;
extern float Ki;
extern float Kd;
extern float ultimoErroA;
extern float ultimoErroB;
extern float integralA;
extern float integralB;

// Protótipos das funções
void taskPID(void *parameter);
float computePID(float setpoint, float valorMedido, float &ultimoErro, float &integral);

#endif
