#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "esp_timer.h"

#define MOTOR_A 1
#define MOTOR_B 2

#define MOTOR_A1 25
#define MOTOR_A2 26
#define MOTOR_B1 32
#define MOTOR_B2 33
#define ENCODER_A 15
#define ENCODER_B 4
#define HISTORICO_TAMANHO 10
#define LIMITE_TEMPO_BLOQUEIO 100

const int canalMotorA1 = 0;
const int canalMotorA2 = 1;
const int canalMotorB1 = 2;
const int canalMotorB2 = 3;
String msg = "";
struct ComandoMotor {
    char comando;
    float rpmValue;
};

const int resolucao = 8;
const int frequencia = 5000;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;


TaskHandle_t taskHandleIMU;



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

QueueHandle_t xQueueComandos;


void taskLeituraSerial(void *parameter);
void taskIMUCode(void *parameter);
void taskControleMotores(void *parameter);
void printEvent(sensors_event_t* event, sensors_event_t* event1, sensors_event_t* event2);

void loop() {
  // Em FreeRTOS, o loop pode ser deixado vazio.
}

void taskIMUCode(void * parameter) {// finalizada
  
  while (1) {
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    printEvent(&orientationData, &angVelocityData, &linearAccelData);

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
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void taskControleMotores(void *parameter) {
    char comando;
    for (;;) {
        if (xQueueReceive(xQueueComandos, &comando,portMAX_DELAY)) { // Não bloqueia esperando comandos
            switch (comando) {
                case 'W':
                    Serial.println(comando);
                    ledcWrite(canalMotorA1, 255);
                    ledcWrite(canalMotorA2, 0);
                    ledcWrite(canalMotorB1, 255);
                    ledcWrite(canalMotorB2, 0);
                    break;
                case 'S':
                    // Define pwmA e pwmB para andar para trás
                    Serial.println(comando);
                    ledcWrite(canalMotorA1, 0);
                    ledcWrite(canalMotorA2, 150);
                    ledcWrite(canalMotorB1, 0);
                    ledcWrite(canalMotorB2, 150);
                    break;
                default:
                    ledcWrite(canalMotorA1, 0);
                    ledcWrite(canalMotorA2, 0);
                    ledcWrite(canalMotorB1, 0);
                    ledcWrite(canalMotorB2, 0);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
}
}

void taskLeituraSerial(void *parameter) {//task finalizada
  for (;;) {
    if (Serial.available()) {
      char comando = Serial.read();
      // Envie o comando para a fila de comandos
      xQueueSend(xQueueComandos, &comando, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay para não saturar o núcleo
  }
}
void printEvent(sensors_event_t* event, sensors_event_t* event1, sensors_event_t* event2) {//finalizada
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  msg =  " Orient: x=" + String(event->orientation.x, 2) + " y=" + String(event->orientation.y, 2) + " z=" + String(event->orientation.z, 2) +
  "; Gyro: x=" + String(event1->gyro.x, 2) + " y=" + String(event1->gyro.y, 2) + " z=" + String(event1->gyro.z, 2) +
  "; Accel: x=" + String(event2->acceleration.x, 2) + " y=" + String(event2->acceleration.y, 2) + " z=" + String(event2->acceleration.z, 2)+";";
  Serial.print(msg);
}

void setup(){

  Serial.begin(115200);
  // Configura os pinos como saída PWM
  ledcSetup(canalMotorA1, frequencia, resolucao);
  ledcSetup(canalMotorA2, frequencia, resolucao);
  ledcSetup(canalMotorB1, frequencia, resolucao);
  ledcSetup(canalMotorB2, frequencia, resolucao);

  ledcAttachPin(MOTOR_A1, canalMotorA1);
  ledcAttachPin(MOTOR_A2, canalMotorA2);
  ledcAttachPin(MOTOR_B1, canalMotorB1);
  ledcAttachPin(MOTOR_B2, canalMotorB2);


  // Inicia a comunicação Serial
  Serial.println("Aguardando comando...");
  if (!bno.begin()) {
    Serial.println("Falha na inicialização do BNO055");
  }


  // Inicialização dos semáforos

  // Inicialização das filas
  xQueueComandos = xQueueCreate(10, sizeof(char));

  // Criação das tasks de leitura dos enconders em nucleos separados
  xTaskCreate(taskControleMotores, "Tarefa dos motores", 10000, NULL, 9, NULL);
  xTaskCreate(taskIMUCode, "Tarefa do IMU", 10000, NULL, 5, &taskHandleIMU);
  xTaskCreate(taskLeituraSerial, "Tarefa Serial", 10000, NULL, 3, NULL);

}