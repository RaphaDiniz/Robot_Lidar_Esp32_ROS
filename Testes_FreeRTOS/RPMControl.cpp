// RPMControl.cpp
#include "Arduino.h"
#include "RPMControl.h"
#define LIMITE_TEMPO_BLOQUEIO 100
QueueHandle_t xQueueRpmResults = 0;

void taskCalculoRPM(void *parameter) {
    long contadorA, contadorAnteriorA = 0;
    long contadorB, contadorAnteriorB = 0;
    long tempoUltimaLeituraA = 0, tempoUltimaLeituraB = 0;
    float rpmA, rpmB;
    float rpmResults[2];

    for (;;) {
        if(xQueueReceive(xQueueEncoderAData, &contadorA, LIMITE_TEMPO_BLOQUEIO) &&
        xQueueReceive(xQueueEncoderBData, &contadorB, LIMITE_TEMPO_BLOQUEIO)){

            rpmA = calcularRPM(contadorA, contadorAnteriorA, tempoUltimaLeituraA);
            rpmB = calcularRPM(contadorB, contadorAnteriorB, tempoUltimaLeituraB);
            rpmResults[0] = rpmA;

            Serial.println(rpmResults[0]);

            xQueueSend(xQueueRpmResults, &rpmResults, LIMITE_TEMPO_BLOQUEIO);
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}
float calcularRPM(long contadorAtual, long &contadorAnterior, long &tempoUltimaLeitura) {
    const int PULSOS_POR_REVOLUCAO = 20;
    long agora = millis();
    long deltaContagem = contadorAtual - contadorAnterior;
    long deltaTime = agora - tempoUltimaLeitura;

    // Verifica se houve alguma contagem desde a última leitura
    if (deltaContagem == 0) {
        return 0; // Nenhuma revolução detectada desde a última leitura
    }

    // Evitar divisão por zero e garantir intervalo de tempo mínimo
    if (deltaTime < 10) { // 10 milissegundos como intervalo mínimo
        return 0; // Evita valores de RPM anormalmente altos
    }

    float revolucoes = (float)deltaContagem / PULSOS_POR_REVOLUCAO;
    float rpm = (revolucoes / deltaTime) * 60000.0f; // Multiplica por 60000 para converter ms em minutos
    Serial.print("Delta Contagem: ");
    Serial.print(deltaContagem);
    Serial.print(" | Delta Tempo (ms): ");
    Serial.print(deltaTime);
    Serial.print(" | RPM Calculado: ");
    Serial.println(rpm);
    // Atualiza os valores para a próxima leitura
    contadorAnterior = contadorAtual;
    tempoUltimaLeitura = agora;

    return rpm;
}