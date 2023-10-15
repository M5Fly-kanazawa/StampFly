#ifndef ACCEL_GYRO_HPP
#define ACCEL_GYRO_HPP

#include <stdio.h>
#include <math.h>
#include <Wire.h>
#define  BMI270_ADDRESS (0x68)

uint8_t bmi270_byte_read(uint8_t reg_addr);
void bmi270_byte_write(uint8_t reg_addr, uint8_t data);


#endif
