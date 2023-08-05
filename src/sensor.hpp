#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
#include <MadgwickAHRS.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6886.h>
#include <stdint.h>
#include "alt_kalman.hpp"

#define SDA_PIN (46)
#define SCL_PIN (41)

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} quat_t;

typedef struct
{
  uint16_t distance;
  uint16_t cnt;  
} distance_t;

//Sensor data
//extern volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
extern volatile float Roll_angle, Pitch_angle, Yaw_angle;
extern volatile float Roll_rate, Pitch_rate, Yaw_rate;
extern volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
extern volatile float Accel_z;
extern volatile float Altitude;
extern volatile float Altitude2;
extern volatile float Alt_velocity;
extern volatile float Voltage;
extern float Acc_norm;
extern quat_t Quat;
extern float Over_g, Over_rate;
extern uint8_t OverG_flag;
extern volatile uint8_t Under_voltage_flag;

void sensor_init(void);
float sensor_read(void);
void sensor_reset_offset(void);
void sensor_calc_offset_avarage(void);
void ahrs_reset(void);
void tof_init(void);


#endif