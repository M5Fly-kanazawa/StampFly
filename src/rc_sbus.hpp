#ifndef RC_HPP
#define RC_HPP

#include <stdio.h>
#include <stdint.h>

#define RUDDER 0
#define ELEVATOR 1
#define THROTTLE 2
#define AILERON 3
#define MODESWITCH 4
#define DPAD_UP 5
#define DPAD_DOWN 6
#define DPAD_LEFT 7
#define DPAD_RIGHT 8
#define BUTTON 9
#define BUTTON_A 10
#define CONTROLMODE 11

#define LOG_MAX 1
#define LOG_MIN 0
#define BTID "4c:75:25:d5:b2:8e"

#define CH1MAX 1771
#define CH1MIN 278
#define CH2MAX 1756
#define CH2MIN 316
#define CH3MAX 1774
#define CH3MIN 372
#define CH4MAX 1745
#define CH4MIN 291
#define CH5MAX 1904
#define CH5MIN 144
#define CH6MAX 1904
#define CH6MIN 144

#define RUDDER_MAX CH1MAX
#define RUDDER_MIN CH1MIN
#define ELEVATOR_MAX CH2MAX
#define ELEVATOR_MIN CH2MIN
#define THROTTLE_MAX CH3MAX
#define THROTTLE_MIN CH3MIN
#define AILERON_MAX CH4MAX
#define AILERON_MIN CH4MIN



void rc_init(void);
void telemetry_init(void);
void rc_demo(void);
void rc_end(void);
bool rc_isconnected(void);
void telemetry_send(uint8_t* data, uint16_t datalen);
void sbus_dacode(void);

//グローバル変数の宣言
extern volatile float Stick[16];
extern volatile uint8_t Rc_data[1024];
extern volatile uint16_t Rc_length;
extern volatile uint16_t Chdata[18];


#endif