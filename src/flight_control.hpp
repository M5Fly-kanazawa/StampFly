#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <Arduino.h>
#include <math.h>
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include <FastLED.h>

#define BATTERY_VOLTAGE (3.7)
#define WHITE 0xffffff
#define BLUE 0x0000ff
#define RED 0xff0000
#define GREEN 0x00ff00
#define PERPLE 0xff00ff
#define POWEROFFCOLOR 0x18EBF9

#define PIN_BUTTON 0
#define PIN_LED    21
#define NUM_LEDS   1

#define AVERAGENUM 800

#define INIT_MODE 0
#define AVERAGE_MODE 1
#define FLIGHT_MODE 2
#define STAY_MODE 3
#define LOG_MODE 4

#define POWER_LIMIT 3.48
#define POWER_FLG_MAX 20

#define ANGLECONTROL 0
#define RATECONTROL 1

//グローバル関数の宣言
void init_copter(void);
void loop_400Hz(void);

//グローバル変数
extern uint8_t Mode;

//extern volatile uint8_t LockMode;
//extern volatile uint8_t Logoutputflag;
//extern volatile uint32_t S_time, E_time, D_time, S_time2, E_time2, D_time2;

#endif
