// MotorControl.cpp
#include "MotorControl.h"
#include "PinDefinitions.h"

void taskControleMotores(void *parameter) {
    char comando;
    for (;;) {
        if (xQueueReceive(xQueueComandos, &comando, portMAX_DELAY)){

              switch (comando) {
                case 'W':
                  pararMotores();
                  andarParaFrente();
                  break;
                case 'S':
                  pararMotores();
                  andarParaTras();
                  break;
                case 'A':
                  pararMotores();
                  virarEsquerda();
                  break;
                case 'D':
                  pararMotores();
                  virarDireita();
                  break;
                default:
                  pararMotores();
                  break;
              }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void andarParaFrente() {

    ledcWrite(canalMotorA1, 120);
    ledcWrite(canalMotorA2, 0);
    ledcWrite(canalMotorB1, 120);
    ledcWrite(canalMotorB2, 0);
}

void andarParaTras() {

    ledcWrite(canalMotorA1, 0);
    ledcWrite(canalMotorA2, 120);
    ledcWrite(canalMotorB1, 0);
    ledcWrite(canalMotorB2, 120);
}

void virarEsquerda() {

    ledcWrite(canalMotorA1, 0);
    ledcWrite(canalMotorA2, 120);
    ledcWrite(canalMotorB1, 120);
    ledcWrite(canalMotorB2, 0);
}

void virarDireita() {
    ledcWrite(canalMotorA1, 120);
    ledcWrite(canalMotorA2, 0);
    ledcWrite(canalMotorB1, 0);
    ledcWrite(canalMotorB2, 120);
}

void pararMotores() {
  ledcWrite(canalMotorA1, 0);
  ledcWrite(canalMotorA2, 0);
  ledcWrite(canalMotorB1, 0);
  ledcWrite(canalMotorB2, 0);
}
