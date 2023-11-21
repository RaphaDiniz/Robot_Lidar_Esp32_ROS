#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "PinDefinitions.h"
#include "bno055.h"
#include "EncoderControl.h"
#include "PIDControl.h"
#include "RPMControl.h"
#include "SerialControl.h"
#include "MotorControl.h"
#include "esp_timer.h"

#define MOTOR_A 1
#define MOTOR_B 2
#define HISTORICO_TAMANHO 10
#define LIMITE_TEMPO_BLOQUEIO 100
const int canalMotorA1 = 0;
const int canalMotorA2 = 1;
const int canalMotorB1 = 2;
const int canalMotorB2 = 3;

const int resolucao = 8;
const int frequencia = 50000;


const long tempoDebounce = 50; // tempo de debounce em ms

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;


TaskHandle_t taskHandleIMU;

String rmpAB = "";

// Declaração de filas
QueueHandle_t xQueueComandos;

// Protótipos das funções que aparecem na imagem
void interrupcaoEncoderA();
void interrupcaoEncoderB();

void taskEncoderA(void *parameter);
void taskEncoderB(void *parameter);
void taskIMUCode(void *parameter);
void taskMotores(void *parameter);
float computePID(float setpoint, float valorMedido, float &ultimoErro);

void setup() {
  // Configura os pinos como saída PWM
  ledcSetup(canalMotorA1, frequencia, resolucao);
  ledcSetup(canalMotorA2, frequencia, resolucao);
  ledcSetup(canalMotorB1, frequencia, resolucao);
  ledcSetup(canalMotorB2, frequencia, resolucao);

  ledcAttachPin(MOTOR_A1, canalMotorA1);
  ledcAttachPin(MOTOR_A2, canalMotorA2);
  ledcAttachPin(MOTOR_B1, canalMotorB1);
  ledcAttachPin(MOTOR_B2, canalMotorB2);

  // Configuração dos encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), interrupcaoEncoderA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), interrupcaoEncoderB, FALLING);

  // Inicia a comunicação Serial
  Serial.begin(115200);
  Serial.println("Aguardando comando...");
  setupBNO055();

  // Inicialização das filas
  xQueueEncoderAData = xQueueCreate(100, sizeof(long)); // Para valores do encoder A
  xQueueEncoderBData = xQueueCreate(100, sizeof(long)); // Para valores do encoder B
  xQueueRpmResults = xQueueCreate(100, sizeof(float) * 2); // Para resultados RPM (A e B)
  xQueueComandos = xQueueCreate(10, sizeof(char));
  xQueuePWMValues = xQueueCreate(100, sizeof(float));

  // Criação das tasks de leitura dos enconders em nucleos separados

  xTaskCreatePinnedToCore(taskEncoderA, "lerEnconderA", 10000, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(taskEncoderB, "lerEnconderB", 10000, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(taskControleMotores, "Tarefa dos motores", 10000, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(taskIMUCode, "Tarefa do IMU", 10000, NULL, 5, &taskHandleIMU, 1);
  xTaskCreatePinnedToCore(taskPID, "Tarefa PID", 10000, NULL, 7, NULL, 0);
  xTaskCreatePinnedToCore(taskCalculoRPM, "Tarefa RPM", 10000, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(taskLeituraSerial, "Tarefa Serial", 10000, NULL, 3, NULL, 1);
}
void loop() {
  // Em FreeRTOS, o loop pode ser deixado vazio.
}






