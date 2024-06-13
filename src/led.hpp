#ifndef LED_HPP
#define LED_HPP

#include <FastLED.h>
#include <stdint.h>

#define WHITE 0xffffff
#define BLUE 0x0000ff
#define RED 0xff0000
#define GREEN 0x00ff00
#define PERPLE 0xff00ff
#define POWEROFFCOLOR 0x18EBF9
#define FLIPCOLOR 0xFF9933

#define PIN_LED_ONBORD 39
#define PIN_LED_ESP    21
#define NUM_LEDS   1

extern uint32_t Led_color;

void led_init(void);
void led_show(void);
void led_drive(void);
void onboard_led1(CRGB p, uint8_t state);
void onboard_led2(CRGB p, uint8_t state);
void esp_led(CRGB p, uint8_t state);

#endif