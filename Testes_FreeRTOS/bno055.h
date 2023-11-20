// BNO055Control.h
#ifndef BNO055CONTROL_H
#define BNO055CONTROL_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern Adafruit_BNO055 bno;

void setupBNO055();
void taskIMUCode(void * parameter);
void printEvent(sensors_event_t* event, sensors_event_t* event1, sensors_event_t* event2);

#endif
