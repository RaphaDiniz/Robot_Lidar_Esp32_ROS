// SerialControl.h
#ifndef SERIALCONTROL_H
#define SERIALCONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "Arduino.h"

extern QueueHandle_t xQueueComandos;

// Protótipo da função
void taskLeituraSerial(void *parameter);

#endif
