#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x68
//BMP280_ADDRESS            0x76

void setup() {  
  init_copter();
  delay(100);
}

void loop() {
  loop_400Hz();
}
