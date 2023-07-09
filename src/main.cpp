#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"

//#define PIN_BUTTON 0
//#define PIN_LED    21
//#define NUM_LEDS   1

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
