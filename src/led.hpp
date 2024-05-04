#ifndef LED_HPP
#define LED_HPP

#include <FastLED.h>
#include <stdint.h>

extern uint32_t Led_color;


void led_init(void);
void led_show(void);
void led_drive(void);
void onboard_led1(CRGB p, uint8_t state);
void onboard_led2(CRGB p, uint8_t state);
void esp_led(CRGB p, uint8_t state);

#endif