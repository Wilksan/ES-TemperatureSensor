#include <Arduino.h>

#include "SensorBox.hpp"

// TTGO Display
TFT_eSPI tft = TFT_eSPI();

SensorBase *doorSensor = new SensorBox(&tft);

void setup() {
  doorSensor->setup();
}

void loop() {
  doorSensor->loop();
}