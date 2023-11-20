#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "esp_timer.h"

#define MOTOR_A1 25
#define MOTOR_A2 26
#define MOTOR_B1 32
#define MOTOR_B2 33
#define ENCODER_A 15
#define ENCODER_B 2
#define HISTORICO_TAMANHO 10
#define LIMITE_TEMPO_BLOQUEIO 100

SemaphoreHandle_t semaforoAcaoCorretiva;
bool acaoCorretivaRealizada = false;
const int pulsosPorRevolucao = 20;
const int frequencia = 5000;
const int canalMotorA1 = 0;
const int canalMotorA2 = 1;
const int canalMotorB1 = 2;
const int canalMotorB2 = 3;
const int resolucao = 8;

volatile int indiceHistoricoA = 0;
volatile int indiceHistoricoB = 0;
volatile long historicoEncoderA[HISTORICO_TAMANHO];
volatile long historicoEncoderB[HISTORICO_TAMANHO];
volatile long contadorEncoderA = 0;
volatile long contadorEncoderB = 0;
volatile long ultimoTempoInterrupcaoA = 0;
volatile long ultimoTempoInterrupcaoB = 0;

const long tempoDebounce = 50; // tempo de debounce em ms

//variaveis do PID
float Kp = 12;
float Ki = 1.5;
float Kd = 0.2;
float ultimoErroA = 0;
float ultimoErroB = 0;
float integralA = 0;
float integralB = 0;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
SemaphoreHandle_t xSemaphore;
TaskHandle_t taskHandleEncoder;
TaskHandle_t taskHandleIMU;
TaskHandle_t taskHandleMonitorEncoders;



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
String msg = "";
String rmpAB = "";

void atualizarHistoricoA(long valor) {
  historicoEncoderA[indiceHistoricoA] = valor;
  indiceHistoricoA = (indiceHistoricoA + 1) % HISTORICO_TAMANHO;
}

void atualizarHistoricoB(long valor) {
  historicoEncoderB[indiceHistoricoB] = valor;
  indiceHistoricoB = (indiceHistoricoB + 1) % HISTORICO_TAMANHO;
}

long lerUltimoValorValidoA() {
  int indiceAnteriorA = (indiceHistoricoA - 1 + HISTORICO_TAMANHO) % HISTORICO_TAMANHO;
  return historicoEncoderA[indiceAnteriorA];
}

long lerUltimoValorValidoB() {
  int indiceAnteriorB = (indiceHistoricoB - 1 + HISTORICO_TAMANHO) % HISTORICO_TAMANHO;
  return historicoEncoderB[indiceAnteriorB];
}

void IRAM_ATTR interrupcaoEncoderA() {
  long tempoInterrupcao = esp_timer_get_time() / 1000; // Tempo em milissegundos
  if (tempoInterrupcao - ultimoTempoInterrupcaoA > tempoDebounce) {
    contadorEncoderA++;
    ultimoTempoInterrupcaoA = tempoInterrupcao;
    atualizarHistoricoA(contadorEncoderA);
  }
}

void IRAM_ATTR interrupcaoEncoderB() {
  long tempoInterrupcao = esp_timer_get_time() / 1000; // Tempo em milissegundos
  if (tempoInterrupcao - ultimoTempoInterrupcaoB > tempoDebounce) {
    contadorEncoderB++;
    ultimoTempoInterrupcaoB = tempoInterrupcao;
    atualizarHistoricoB(contadorEncoderB);
  }
}

long lerEncoderA() {
  noInterrupts();
  long contagemAtual = contadorEncoderA;
  long tempoDesdeUltimaInterrupcaoA = millis() - ultimoTempoInterrupcaoA;
  interrupts();

  if (tempoDesdeUltimaInterrupcaoA > tempoDebounce) {
    return lerUltimoValorValidoA();
  }

  return contagemAtual;
}

long lerEncoderB() {
  noInterrupts();
  long contagemAtual = contadorEncoderB;
  long tempoDesdeUltimaInterrupcaoB = millis() - ultimoTempoInterrupcaoB;
  interrupts();

  if (tempoDesdeUltimaInterrupcaoB > tempoDebounce) {
    return lerUltimoValorValidoB();
  }

  return contagemAtual;
}

void printEvent(sensors_event_t* event, sensors_event_t* event1, sensors_event_t* event2, String rmpAB) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  msg = rmpAB + " Orient: x=" + String(event->orientation.x, 2) + " y=" + String(event->orientation.y, 2) + " z=" + String(event->orientation.z, 2) +
  "; Gyro: x=" + String(event1->gyro.x, 2) + " y=" + String(event1->gyro.y, 2) + " z=" + String(event1->gyro.z, 2) +
  "; Accel: x=" + String(event2->acceleration.x, 2) + " y=" + String(event2->acceleration.y, 2) + " z=" + String(event2->acceleration.z, 2)+";";
  Serial.print(msg);
}

float computePID(float setpoint, float valorMedido, float &ultimoErro, float &integral) {
    float erro = setpoint - valorMedido;
    integral += erro;
    float derivativo = erro - ultimoErro;

    float saida = Kp * erro + Ki * integral + Kd * derivativo;
    ultimoErro = erro;

    return saida;
}

void tarefaEncoder(void * parameter) {
  long ultimoContadorEncoderA = 0;
  long ultimoContadorEncoderB = 0;

  while (1) {
    long contadorAtualEncoderA = lerEncoderA();
    long contadorAtualEncoderB = lerEncoderB();

    float rpmA = ((contadorAtualEncoderA - ultimoContadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float rpmB = ((contadorAtualEncoderB - ultimoContadorEncoderB) / float(pulsosPorRevolucao)) * 60;
    rmpAB = ">> RPM A: " + String(rpmA) + "; RPM B: " + String(rpmB) + ";";

    ultimoContadorEncoderA = contadorAtualEncoderA;
    ultimoContadorEncoderB = contadorAtualEncoderB;

    if (acaoCorretivaRealizada && xSemaphoreTake(semaforoAcaoCorretiva, portMAX_DELAY)) {
      andarParaFrente(5);
      delay(100);
      pararMotores();
      acaoCorretivaRealizada = false;
      xSemaphoreGive(semaforoAcaoCorretiva); // Liberar o semáforo após ação corretiva
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Delay para próxima leitura
  }
}


void taskIMUCode(void * parameter) {
  
  while (1) {
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    printEvent(&orientationData, &angVelocityData, &linearAccelData, rmpAB);

    int8_t boardTemp = bno.getTemp();
    Serial.print(F(" temperature: "));
    Serial.print(boardTemp);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("; Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    //xSemaphoreTake(xSemaphore,portMAX_DELAY);
    //vTaskDelay(pdMS_TO_TICKS(BNO055_SAMPLERATE_DELAY_MS));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
volatile long ultimoValorEncoderA = 0;
volatile long ultimoValorEncoderB = 0;

void taskMonitoramento(void * parameter) {
  while (1) {
    // Verifica se a task 'tarefaEncoder' está suspensa
    if (eTaskGetState(taskHandleEncoder) == eSuspended) {
      vTaskResume(taskHandleEncoder);
    }
    
    // Verifica se a task 'taskIMUCode' está suspensa
    if (eTaskGetState(taskHandleIMU) == eSuspended) {
      vTaskResume(taskHandleIMU);
    }

    // Verifica se a task 'taskMonitoramentoEncoders' está suspensa
    if (eTaskGetState(taskHandleMonitorEncoders) == eSuspended) {
      vTaskResume(taskHandleMonitorEncoders);
    }

    // Aguarda um pouco antes de verificar novamente
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


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
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), interrupcaoEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), interrupcaoEncoderB, CHANGE);

  // Inicia a comunicação Serial
  Serial.begin(115200);
  Serial.println("Aguardando comando...");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  semaforoAcaoCorretiva = xSemaphoreCreateBinary();
  xSemaphoreGive(semaforoAcaoCorretiva);
  // Na função setup()
  xTaskCreatePinnedToCore(tarefaEncoder, "lerEnconderA", 10000, NULL, 1, &taskHandleEncoder, 0);
  xTaskCreatePinnedToCore(taskIMUCode, "taskIMU", 10000, NULL, 2, &taskHandleIMU, 1);
  xTaskCreatePinnedToCore(taskMonitoramentoEncoders, "MonitorEncoders", 10000, NULL, 4, &taskHandleMonitorEncoders, 0);
  xTaskCreatePinnedToCore(taskMonitoramento, "MonitorTask", 10000, NULL, 5, NULL, 0);
}

void loop() {
  float RPM = 10.00;
  if (Serial.available()) {
    char comando = Serial.read();
    switch (comando) {
      case 'W':
        pararMotores();
        andarParaFrente(RPM);
        break;
      case 'S':
        pararMotores();
        andarParaTras(RPM);
        break;
      case 'A':
        pararMotores();
        virarEsquerda(RPM);
        delay(170);
        pararMotores();
        break;
      case 'D':
        pararMotores();
        virarDireita(RPM);
        delay(230);
        pararMotores();
        break;
      default:
        pararMotores();
        break;
    }
    RPM = 0;
  }
}

void andarParaFrente(float targetRpmAB) {
    float rpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float rpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float PID_A = computePID(targetRpmAB, rpmA, ultimoErroA, integralA);
    float PID_B = computePID(targetRpmAB, rpmB, ultimoErroB, integralB);

    float potenciaMotorA = constrain(PID_A, 0, 255);
    float potenciaMotorB = constrain(PID_B, 0, 255);

    ledcWrite(canalMotorA1, potenciaMotorA);
    ledcWrite(canalMotorA2, 0);
    ledcWrite(canalMotorB1, potenciaMotorB);
    ledcWrite(canalMotorB2, 0);
}

void andarParaTras(float targetRpmAB) {
    float RpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float RpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float PID_A = computePID(targetRpmAB, RpmA, ultimoErroA, integralA);
    float PID_B = computePID(targetRpmAB, RpmB, ultimoErroB, integralB);

    // Certifique-se de que a potência esteja no intervalo permitido, por exemplo, 0-255
    float potenciaMotorA = constrain(PID_A, 0, 255);
    float potenciaMotorB = constrain(PID_B, 0, 255);

    ledcWrite(canalMotorA1, 0);
    ledcWrite(canalMotorA2, potenciaMotorA);
    ledcWrite(canalMotorB1, 0);
    ledcWrite(canalMotorB2, potenciaMotorB);
}

void virarEsquerda(float targetRpm) {
    float currentRpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float currentRpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float potenciaMotorA = computePID(targetRpm, currentRpmA, ultimoErroA, integralA);
    float potenciaMotorB = computePID(targetRpm, currentRpmB, ultimoErroB, integralB);  // Invertemos a direção do motor B

    potenciaMotorA = constrain(potenciaMotorA, 0, 255);
    potenciaMotorB = constrain(potenciaMotorB, 0, 255);

    ledcWrite(canalMotorA1, 0);
    ledcWrite(canalMotorA2, potenciaMotorA);
    ledcWrite(canalMotorB1, potenciaMotorB);
    ledcWrite(canalMotorB2, 0);
}

void virarDireita(float targetRpm) {
    float currentRpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float currentRpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float potenciaMotorA = computePID(targetRpm, currentRpmA, ultimoErroA, integralA);
    float potenciaMotorB = computePID(targetRpm, currentRpmB, ultimoErroB, integralB);

    potenciaMotorA = constrain(potenciaMotorA, 0, 255);
    potenciaMotorB = constrain(potenciaMotorB, 0, 255);

    ledcWrite(canalMotorA1, potenciaMotorA);
    ledcWrite(canalMotorA2, 0);
    ledcWrite(canalMotorB1, 0);
    ledcWrite(canalMotorB2, potenciaMotorB);
}

void pararMotores() {
  ledcWrite(canalMotorA1, 0);
  ledcWrite(canalMotorA2, 0);
  ledcWrite(canalMotorB1, 0);
  ledcWrite(canalMotorB2, 0);
}
