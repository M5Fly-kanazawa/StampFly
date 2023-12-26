#ifndef TOF_HPP
#define TOF_HPP

#include <stdint.h>
#include <vl53lx_api.h>
#include <vl53lx_platform.h>

#define INT_BOTTOM 6
#define XSHUT_BOTTOM 7
#define INT_FRONT 8
#define XSHUT_FRONT 9
#define USER_A 0

void tof_init(void);
uint16_t tof_range_get(VL53LX_DEV dev);
void tof_test_ranging(VL53LX_DEV dev);
uint16_t tof_bottom_get_range();
uint16_t tof_front_get_range();

#endif