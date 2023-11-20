// BNO055Control.cpp
#include "bno055.h"

String msg = "";
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setupBNO055() {
    Serial.begin(115200);
    if (!bno.begin()) {
        Serial.println("Falha na inicialização do BNO055");
    }
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
    /*printEvent(&orientationData, &angVelocityData, &linearAccelData);

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
    //vTaskDelay(pdMS_TO_TICKS(BNO055_SAMPLERATE_DELAY_MS));*/
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void printEvent(sensors_event_t* event, sensors_event_t* event1, sensors_event_t* event2) {//finalizada
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  msg =" Orient: x=" + String(event->orientation.x, 2) + " y=" + String(event->orientation.y, 2) + " z=" + String(event->orientation.z, 2) +
  "; Gyro: x=" + String(event1->gyro.x, 2) + " y=" + String(event1->gyro.y, 2) + " z=" + String(event1->gyro.z, 2) +
  "; Accel: x=" + String(event2->acceleration.x, 2) + " y=" + String(event2->acceleration.y, 2) + " z=" + String(event2->acceleration.z, 2)+";";
  Serial.print(msg);
}