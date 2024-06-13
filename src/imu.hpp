#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h> 
#include "common.h"
#include "bmi2.h"

#define DPS20002RAD 34.90658504
#define DPS10002RAD 17.4532925199

void imu_init(void);
void imu_test(void);
void imu_update(void);
float imu_get_acc_x(void);
float imu_get_acc_y(void);
float imu_get_acc_z(void);
float imu_get_gyro_x(void);
float imu_get_gyro_y(void);
float imu_get_gyro_z(void);


#endif