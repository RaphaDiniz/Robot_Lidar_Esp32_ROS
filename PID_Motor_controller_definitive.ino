#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define MOTOR_A1 32
#define MOTOR_A2 33
#define MOTOR_B1 25
#define MOTOR_B2 26
#define ENCODER_A 15
#define ENCODER_B 4

const int frequencia = 5000;
const int canalMotorA1 = 0;
const int canalMotorA2 = 1;
const int canalMotorB1 = 2;
const int canalMotorB2 = 3;
const int resolucao = 8;

//***********
//***********
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

TaskHandle_t taskIMU;
SemaphoreHandle_t xSemaphore;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
String msg = "";
String rmpAB = "";
char command = 'F';
void printEvent(sensors_event_t* event, sensors_event_t* event1, sensors_event_t* event2, String rmpAB) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  msg = "ESP32 published data . . . "+rmpAB + " Orient: x=" + String(event->orientation.x, 2) + " y=" + String(event->orientation.y, 2) + " z=" + String(event->orientation.z, 2) +
  "; Gyro: x=" + String(event1->gyro.x, 2) + " y=" + String(event1->gyro.y, 2) + " z=" + String(event1->gyro.z, 2) +
  "; Accel: x=" + String(event2->acceleration.x, 2) + " y=" + String(event2->acceleration.y, 2) + " z=" + String(event2->acceleration.z, 2)+";"+" Command: " + String(command) + ";";
  Serial.println(msg);
}
//***********
//***********
volatile long contadorEncoderA = 0;
volatile long contadorEncoderB = 0;

const long tempoDebounce = 5; // tempo de debounce em ms
volatile long ultimoTempoInterrupcaoA = 0;
volatile long ultimoTempoInterrupcaoB = 0;

const int pulsosPorRevolucao = 20;
const int BACKWARDS = 3;
const int FORWARD = 2;
const int RIGHT = 1;
const int LEFT = 0;
float Kp = 6;
float Ki = 0.045;
float Kd = 0.16;
float ultimoErroA = 0;
float ultimoErroB = 0;
float integralA = 0;
float integralB = 0;

float computePID(float setpoint, float valorMedido, float &ultimoErro, float &integral) {
    float erro = setpoint - valorMedido;
    integral += erro;
    float derivativo = erro - ultimoErro;

    float saida = Kp * erro + Ki * integral + Kd * derivativo;
    ultimoErro = erro;

    return saida;
}

void IRAM_ATTR interrupcaoEncoderA() {
  long tempoInterrupcao = millis();
  if (tempoInterrupcao - ultimoTempoInterrupcaoA > tempoDebounce) {
    contadorEncoderA++;
    ultimoTempoInterrupcaoA = tempoInterrupcao;
  }
}

void IRAM_ATTR interrupcaoEncoderB() {
  long tempoInterrupcao = millis();
  if (tempoInterrupcao - ultimoTempoInterrupcaoB > tempoDebounce) {
    contadorEncoderB++;
    ultimoTempoInterrupcaoB = tempoInterrupcao;
  }
}

void tarefaEncoder(void * parameter) {

  while (1) {

    float rpmA = ((contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float rpmB = ((contadorEncoderB) / float(pulsosPorRevolucao)) * 60;
    rmpAB = ">> RPM A: " + String(rpmA) + "; RPM B: " + String(rpmB)+";";

    contadorEncoderA = 0;
    contadorEncoderB = 0;
    //xSemaphoreGive(xSemaphore);
    vTaskDelay(pdMS_TO_TICKS(500));
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
    /*Serial.print(F(" temperature: "));
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
    Serial.println(mag);*/
    //xSemaphoreTake(xSemaphore,portMAX_DELAY);
    //vTaskDelay(pdMS_TO_TICKS(BNO055_SAMPLERATE_DELAY_MS));
    vTaskDelay(pdMS_TO_TICKS(500));
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

  xTaskCreatePinnedToCore(tarefaEncoder, "tarefaEncoder", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskIMUCode, "taskIMU", 10000, NULL, 2, &taskIMU, 1);
}

void loop() {
  float setPoint = 22.00;
  if (Serial.available()) {
    char comando = Serial.read();
    command = comando;
    stopRPM();
    delay(800);
    switch (comando) {
      case 'W':  
        PID(setPoint, FORWARD);
        break;
      case 'S':
        PID(setPoint, BACKWARDS);
        break;
      case 'A':
        PID(setPoint, LEFT);
        break;
      case 'D':
        PID(setPoint, RIGHT);
        break;
      default:
        break;
    }
  }
  delay(10);
}

void PID(float setPoint, int orientation){ 
  float currentRpmA = ((contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
  float currentRpmB = ((contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

  float potenciaMotorA = computePID(setPoint, currentRpmA, ultimoErroA, integralA);
  float potenciaMotorB = computePID(setPoint, currentRpmB, ultimoErroB, integralB);
  
  potenciaMotorA = constrain(potenciaMotorA, 0, 255);
  potenciaMotorB = constrain(potenciaMotorA, 0, 255);
  
  float pwmA1 = 0, pwmA2 = 0, pwmB1 = 0, pwmB2 = 0;
  
  switch (orientation) {
    case LEFT:
      pwmA2 = potenciaMotorA;
      pwmB1 = potenciaMotorB; 
      break;
    case RIGHT:
      pwmA1 = potenciaMotorA;
      pwmB2 = potenciaMotorB; 
      break;
    case FORWARD:
      pwmA1 = potenciaMotorA;
      pwmB1 = potenciaMotorB; 
      break;
    case BACKWARDS:
      pwmA2 = potenciaMotorA;
      pwmB2 = potenciaMotorB; 
      break;
    default:
      break;
  }
  ledcWrite(canalMotorA1, pwmA1);
  ledcWrite(canalMotorA2, pwmA2);
  ledcWrite(canalMotorB1, pwmB1);
  ledcWrite(canalMotorB2, pwmB2);
}

void stopRPM(){
  ledcWrite(canalMotorA1, 0);
  ledcWrite(canalMotorA2, 0);
  ledcWrite(canalMotorB1, 0);
  ledcWrite(canalMotorB2, 0);
}
