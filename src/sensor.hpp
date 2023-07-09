#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
#include "MadgwickAHRS.h"
#include <Adafruit_VL53L0X.h>
#include "MPU6886.h"
#include <stdint.h>
#include "alt_kalman.hpp"


//#define I2C_SCL 21
//#define I2C_SDA 25
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
extern volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
extern volatile float Phi, Theta, Psi;
extern volatile float Altitude;
extern volatile float Altitude2;
extern volatile float Voltage;
extern float Acc_norm;
extern quat_t Quat;
extern float Over_g, Over_rate;
extern uint8_t OverG_flag;
extern volatile float Pbias, Qbias, Rbias;
extern volatile uint8_t Power_flag;

void sensor_init(void);
void sensor_read(void);
void ahrs_reset(void);
void tof_init(void);

#endif

#if 0
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
#include "MadgwickAHRS.h"
#include "vl53l0x.hpp"
#include <stdint.h>
#include "MPU6886.h"

#define SDA_PIN (46)
#define SCL_PIN (41)

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} quat_t;

//Sensor data
extern volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
extern volatile float Phi, Theta, Psi;

extern volatile float Voltage;
extern float Acc_norm;
extern quat_t Quat;
extern float Over_g, Over_rate;
extern uint8_t OverG_flag;
extern volatile float Pbias, Qbias, Rbias;
extern volatile uint8_t Power_flag;

void sensor_init(void);
void sensor_read(void);

#endif
#endif