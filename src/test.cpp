#include "test.hpp"
#include "flight_control.hpp"

volatile uint32_t pulse_counter=0;
uint32_t old_pulse_counter = 0;
uint32_t counter=0;
uint8_t flag = 0;
uint16_t interval = 0;
volatile float velocity=0;
volatile uint32_t _time=0, _time_old=0;
volatile uint32_t _time2=0, _time_old2=0;


void IRAM_ATTR pulse_count(void)
{
    pulse_counter++;
    //_time_old = _time;
    //_time = micros();
}

void test_init(void)
{
    pinMode(BLACK_GROVE_PIN2, INPUT);
    attachInterrupt(BLACK_GROVE_PIN2, &pulse_count, FALLING);
}

void test_counter_reset(void)
{
    counter = 0;
}

void test_main(void)
{   
    _time_old2 = _time2;
    _time2 = micros();
    if(flag==0)
    {
        flag = 1;
        test_counter_reset();
    }
    counter++;
    if((counter-1)<400*5){}
    else if((counter-1)<400*(5+3))
    {
        set_duty_fr(0.3);
        set_duty_fl(0.3);
        set_duty_rr(0.3);
        set_duty_rl(0.3);      
    }
    else
    {
        set_duty_fr(0.0);
        set_duty_fl(0.0);
        set_duty_rr(0.0);
        set_duty_rl(0.0);      
    }
    //velocity = 1.0e6*(2*PI/3)/(float)(_time - _time_old);
    USBSerial.printf("%9.4f %9.6f %f\n\r", 
        (float)(counter-1)*0.0025f, 
        1.0e-6*(float)(_time2 - _time_old2),
        (float)pulse_counter);
}


