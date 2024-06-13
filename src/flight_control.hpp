#ifndef CONTROL_HPP
#define CONTROL_HPP

//#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <FastLED.h>
#include <vl53lx_platform.h>

#define BATTERY_VOLTAGE (3.7)
#define PIN_BUTTON 0
#define AVERAGENUM 800
#define INIT_MODE 0
#define AVERAGE_MODE 1
#define FLIGHT_MODE 2
#define PARKING_MODE 3
#define LOG_MODE 4

#define POWER_LIMIT 3.34
#define UNDER_VOLTAGE_COUNT 100

#define ANGLECONTROL 0
#define RATECONTROL 1

//グローバル関数の宣言
void init_copter(void);
void loop_400Hz(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

//グローバル変数
extern uint8_t Mode;
extern volatile uint8_t Loop_flag;
extern float Control_period;
extern volatile float Elapsed_time;

//PID Gain
//Rate control PID gain
extern const float Roll_rate_kp;
extern const float Roll_rate_ti;
extern const float Roll_rate_td;
extern const float Roll_rate_eta;

extern const float Pitch_rate_kp;
extern const float Pitch_rate_ti;
extern const float Pitch_rate_td;
extern const float Pitch_rate_eta;

extern const float Yaw_rate_kp;
extern const float Yaw_rate_ti;
extern const float Yaw_rate_td;
extern const float Yaw_rate_eta;

//Angle control PID gain
extern const float Rall_angle_kp;
extern const float Rall_angle_ti;
extern const float Rall_angle_td;
extern const float Rall_angle_eta;

extern const float Pitch_angle_kp;
extern const float Pitch_angle_ti;
extern const float Pitch_angle_td;
extern const float Pitch_angle_eta;

//Altitude control PID gain
extern const float alt_kp;
extern const float alt_ti;
extern const float alt_td;
extern const float alt_eta;
extern const float alt_period;

extern volatile float Interval_time;

//Offset
extern volatile float Roll_angle_offset, Pitch_angle_offset, Yaw_angle_offset;  
extern volatile float Elevator_center, Aileron_center, Rudder_center;

//制御目標
//PID Control reference
//角速度目標値
//Rate reference
extern volatile float Roll_rate_reference, Pitch_rate_reference, Yaw_rate_reference;
//角度目標値
//Angle reference
extern volatile float Roll_angle_reference, Pitch_angle_reference, Yaw_angle_reference;
//舵角指令値
//Commanad
//スロットル指令値
//Throttle
extern volatile float Thrust_command;
//角速度指令値
//Rate command
extern volatile float Roll_rate_command, Pitch_rate_command, Yaw_rate_command;
//角度指令値
//Angle comannd
extern volatile float Roll_angle_command, Pitch_angle_command, Yaw_angle_command;
//高度目標
extern volatile float Alt_ref;
//Motor Duty 
extern volatile float FrontRight_motor_duty;
extern volatile float FrontLeft_motor_duty;
extern volatile float RearRight_motor_duty;
extern volatile float RearLeft_motor_duty;
//速度目標Z
extern float Z_dot_ref;

extern uint8_t Control_mode;
extern uint8_t Flip_flag;
extern uint8_t Alt_flag;

#endif
