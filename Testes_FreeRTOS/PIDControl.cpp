// PIDControl.cpp
#define LIMITE_TEMPO_BLOQUEIO 100
#include "PIDControl.h"
#include "Arduino.h"

QueueHandle_t xQueuePWMValues = 0;

// Definições das variáveis globais do PID
float Kp = 2.5;
float Ki = 0.75;
float Kd = 0.02;
float ultimoErroA = 0;
float ultimoErroB = 0;
float integralA = 0;
float integralB = 0;

void taskPID(void *parameter) {
    float rpmResults[2];
    float setpointA = 30; // Defina o setpoint para o motor A
    float setpointB = 30; // Defina o setpoint para o motor B
    float outputA, outputB;

    for (;;) {
        if (xQueueReceive(xQueueRpmResults, &rpmResults, LIMITE_TEMPO_BLOQUEIO)) {
            Serial.println("Entrou no PID");
            float erroA = setpointA - rpmResults[0];
            float erroB = setpointB - rpmResults[1];
            Serial.print("PWMValue recebido:");
            Serial.println(rpmResults[0]);
            outputA = computePID(setpointA, rpmResults[0], ultimoErroA, integralA);
            outputB = computePID(setpointB, rpmResults[1], ultimoErroB, integralB);
            Serial.print("Output:" );
            Serial.println(outputA);

            // Suponha que xQueuePWMValues seja a fila para enviar valores de PWM
            xQueueSend(xQueuePWMValues, &outputA, LIMITE_TEMPO_BLOQUEIO);
            xQueueSend(xQueuePWMValues, &outputB, LIMITE_TEMPO_BLOQUEIO);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


float IRAM_ATTR computePID(float setpoint, float valorMedido, float &ultimoErro, float &integral) {//finalizada
    float erro = setpoint - valorMedido;
    integral += erro;
    float derivativo = erro - ultimoErro;

    float saida = Kp * erro + Ki * integral + Kd * derivativo;
    saida = constrain(saida, -255, 255);
    ultimoErro = erro;

    return saida;
}