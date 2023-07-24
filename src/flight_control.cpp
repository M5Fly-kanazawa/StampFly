#include "flight_control.hpp"

//#define DEBUG

//モータPWM出力Pinのアサイン
//Motor PWM Pin
const int pwmFrontLeft  = 7;
const int pwmFrontRight = 1;
const int pwmRearLeft   = 12;
const int pwmRearRight  = 42;

//モータPWM周波数 
//Motor PWM Frequency
const int freq = 150000;

//PWM分解能
//PWM Resolution
const int resolution = 8;

//モータチャンネルのアサイン
//Motor Channel
const int FrontLeft_motor  = 1;
const int FrontRight_motor = 2;
const int RearLeft_motor   = 3;
const int RearRight_motor  = 4;

//制御周期
//Control period
const float Control_period = 0.0025f;//400Hz

//PID Gain
//Rate control PID gain
const float Roll_rate_kp = 0.6f;//0.65
const float Roll_rate_ti = 0.7f;
const float Roll_rate_td = 0.01;//0.03
const float Roll_rate_eta = 0.125f;

const float Pitch_rate_kp = 0.75f;//0.65
const float Pitch_rate_ti = 0.7f;
const float Pitch_rate_td = 0.025f;//0.03
const float Pitch_rate_eta = 0.125f;

const float Yaw_rate_kp = 3.0f;//
const float Yaw_rate_ti = 0.8f;
const float Yaw_rate_td = 0.01f;//0.00
const float Yaw_rate_eta = 0.125f;

//Angle control PID gain
const float Rall_angle_kp = 8.0f;//12
const float Rall_angle_ti = 4.0f;
const float Rall_angle_td = 0.04f;//0.04
const float Rall_angle_eta = 0.125f;

const float Pitch_angle_kp = 8.0f;//12
const float Pitch_angle_ti = 4.0f;
const float Pitch_angle_td = 0.04f;
const float Pitch_angle_eta = 0.125f;

//Times
volatile float Elapsed_time=0.0f;
volatile float Old_Elapsed_time=0.0f;
volatile float Interval_time=0.0f;
volatile uint32_t S_time=0,E_time=0,D_time=0,Dt_time=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t OffsetCounter=0;
uint16_t LedBlinkCounter=0;

//Motor Duty 
volatile float FrontRight_motor_duty=0.0f;
volatile float FrontLeft_motor_duty=0.0f;
volatile float RearRight_motor_duty=0.0f;
volatile float RearLeft_motor_duty=0.0f;

//制御目標
//PID Control reference
//角速度目標値
//Rate reference
volatile float Roll_rate_reference=0.0f, Pitch_rate_reference=0.0f, Yaw_rate_reference=0.0f;
//角度目標値
//Angle reference
volatile float Roll_angle_reference=0.0f, Pitch_angle_reference=0.0f, Yaw_angle_reference=0.0f;
//舵角指令値
//Commanad
//スロットル指令値
//Throttle
volatile float Thrust_command=0.0f;
//角速度指令値
//Rate command
volatile float Roll_rate_command=0.0f, Pitch_rate_command=0.0f, Yaw_rate_command=0.0f;
//角度指令値
//Angle comannd
volatile float Roll_angle_command=0.0f, Pitch_angle_command=0.0f, Yaw_angle_command=0.0f;

//Offset
volatile float Roll_angle_offset=0.0f, Pitch_angle_offset=0.0f, Yaw_angle_offset=0.0f;  
volatile float Elevator_center=0.0f, Aileron_center=0.0f, Rudder_center=0.0f;

//Telemetory
uint8_t Telem_mode=0;
uint8_t Telem_cnt = 0;

//Machine state
float Timevalue=0.0f;
uint8_t Mode = INIT_MODE;
uint8_t Control_mode = ANGLECONTROL;
volatile uint8_t LockMode=0;
float Motor_on_duty_threshold = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
int8_t BtnA_counter = 0;
uint8_t BtnA_on_flag = 0;
uint8_t BtnA_off_flag =1;
volatile uint8_t Loop_flag = 0;
volatile uint8_t Angle_control_flag = 0;
uint32_t Led_color = 0x000000;

//flip
float FliRoll_rate_time = 2.0;
uint8_t Flip_flag = 0;
uint16_t Flip_counter = 0; 
float Flip_time = 2.0;
volatile uint8_t Ahrs_reset_flag=0;
float T_flip;

//PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
CRGB led_esp[NUM_LEDS];
CRGB led_onboard[NUM_LEDS];

//Function declaration
void init_pwm();
void control_init();
void variable_init(void);
void onboard_led(CRGB p, uint8_t state);
void esp_led(CRGB p, uint8_t state);
void get_command(void);
void angle_control(void);
void rate_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void motor_stop(void);
void led_drive(void);
uint8_t judge_mode_change(void);
uint8_t get_arming_button(void);
uint8_t get_flip_button(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len);
void data2log(uint8_t* data_list, float add_data, uint8_t index);
void telemetry(void);
void telemetry_sequence(void);
void make_telemetry_data(uint8_t* senddata);
void make_telemetry_header_data(uint8_t* senddata);

//割り込み関数
//Intrupt function
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() 
{
  Loop_flag = 1;
}

//Initialize Multi copter
void init_copter(void)
{
  //Initialize Mode
  Mode = INIT_MODE;

  //Initialaze LED function
  FastLED.addLeds<WS2812, PIN_LED_ONBORD, GRB>(led_onboard, NUM_LEDS);
  FastLED.addLeds<WS2812, PIN_LED_ESP, GRB>(led_esp, NUM_LEDS);

  led_esp[0]=RED;
  led_onboard[0]=WHITE;

  FastLED.show();

  //Initialize Serial communication
  USBSerial.begin(115200);
  delay(2000);
  USBSerial.printf("Start StampS3FPV!\r\n");
  
  //Initialize PWM
  init_pwm();

  //Initilize Radio control
  rc_init();
  sensor_init();
  control_init();

  //割り込み設定
  //Initialize intrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
}

//Main loop
void loop_400Hz(void)
{
  static uint8_t led=1;
  float sense_time;
  //割り込みにより400Hzで以降のコードが実行
  while(Loop_flag==0);
  Loop_flag = 0;

  E_time = micros();
  Old_Elapsed_time = Elapsed_time;
  Elapsed_time = 1e-6*(E_time - S_time);
  Interval_time = Elapsed_time - Old_Elapsed_time;
  Timevalue+=0.0025f;
  
  //Read Sensor Value
  sense_time = sensor_read();
  uint32_t cs_time = micros();

  led_drive();

  //Begin Mode select
  if (Mode == INIT_MODE)
  {
      motor_stop();
      Elevator_center = 0.0f;
      Aileron_center = 0.0f;
      Rudder_center = 0.0f;
      Roll_angle_offset = 0.0f;
      Pitch_angle_offset = 0.0f;
      Yaw_angle_offset = 0.0f;
      sensor_reset_offset();
      Mode = AVERAGE_MODE;
      return;
  }
  else if (Mode == AVERAGE_MODE)
  {
    motor_stop();
    //Gyro offset Estimate
    if (OffsetCounter < AVERAGENUM)
    {
      sensor_calc_offset_avarage();
      OffsetCounter++;
      return;
    }
    //Mode change
    Mode = PARKING_MODE;
    S_time = micros();
    return;
  }
  else if( Mode == FLIGHT_MODE)
  {
    //Judge Mode change
    if (judge_mode_change() == 1) Mode = PARKING_MODE;
    
    //Get command
    get_command();

    //Angle Control
    angle_control();

    //Rate Control
    rate_control();

  }
  else if(Mode == PARKING_MODE)
  {
    //Judge Mode change
    if( judge_mode_change() == 1)Mode = FLIGHT_MODE;

    //Parking
    motor_stop();
    OverG_flag = 0;
    Angle_control_flag = 0;
    
  }

  //Telemetry
  telemetry();

  uint32_t ce_time = micros();
  //if(Telem_cnt == 1)Dt_time = D_time - E_time;
  Dt_time = ce_time - cs_time;
  #ifdef DEBUG
  USBSerial.printf("Loop time(ms) %5.3f %5.3f %5.3f %5.3f Range %6.1f\n\r", 
    sense_time*1000.0, 
    (ce_time - cs_time)*1.0e-3, 
    (ce_time - cs_time)*1.0e-3 + sense_time*1000.0,
    Interval_time*1000.0,
    Altitude );
  #endif

  //End of Loop_400Hz function
}

uint8_t judge_mode_change(void)
{
  //Ariming Button が押されて離されたかを確認
  uint8_t state;
  state = 0;
  if(LockMode == 0)
  {
    if( get_arming_button()==1)
    {
      LockMode = 1;
    }
  }
  else
  {
    if( get_arming_button()==0)
    {
      LockMode = 0;
      state = 1;
    }
  }
  //USBSerial.printf("%d %d\n\r", state, LockMode);
  return state;
}

void led_drive(void)
{
  if (Mode == AVERAGE_MODE)
  {
    onboard_led(PERPLE, 1);
  }
  else if(Mode == FLIGHT_MODE)
  {
    if(Control_mode == ANGLECONTROL)
    {
      if(Flip_flag==0)Led_color=0xffff00;
      else Led_color = 0xFF9933;
    }
    else Led_color = 0xDC669B;

    if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) onboard_led(Led_color, 1);
    else onboard_led(POWEROFFCOLOR,1);
  }
  else if (Mode == PARKING_MODE)
  {
    if(LedBlinkCounter<10){
      if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) onboard_led(GREEN, 1);
      else onboard_led(POWEROFFCOLOR,1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      if (Under_voltage_flag <UNDER_VOLTAGE_COUNT) onboard_led(GREEN, 0);
      else onboard_led(POWEROFFCOLOR,0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
  }

  //Watch dog LED
  if ( Dt_time>2550u )onboard_led(RED, 1);

  //LED show
  FastLED.show();
  //USBSerial.printf("Time delta %f\n\r", (Elapsed_time - Old_Elapsed_time) );
}



///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//  PID control gain setting
//
//  Sets the gain of PID control.
//  
//  Function usage
//  PID.set_parameter(PGAIN, IGAIN, DGAIN, TC, STEP)
//
//  PGAIN: PID Proportional Gain
//  IGAIN: PID Integral Gain
//   *The larger the value of integral gain, the smaller the effect of integral control.
//  DGAIN: PID Differential Gain
//  TC:    Time constant for Differential control filter
//  STEP:  Control period
//
//  Example
//  Set roll rate control PID gain
//  p_pid.set_parameter(2.5, 10.0, 0.45, 0.01, 0.001); 

void control_init(void)
{
  //Rate control
  p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta, Control_period);//Roll rate control gain
  q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta, Control_period);//Pitch rate control gain
  r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);//Yaw rate control gain
  //Roll P gain を挙げてみて分散が減るかどうか考える
  //Roll Ti を大きくしてみる

  //Angle control
  phi_pid.set_parameter  (Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta, Control_period);//Roll angle control gain
  theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta, Control_period);//Pitch angle control gain

}
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void get_command(void)
{
  Control_mode = Stick[CONTROLMODE];

  //if(OverG_flag == 1){
  //  Thrust_command = 0.0;
  //}

  //Throttle curve conversion　スロットルカーブ補正
  float throttle_limit = 0.7;
  float thlo = Stick[THROTTLE];
  thlo = thlo/throttle_limit;
  if (thlo>1.0f) thlo = 1.0f;
  if (thlo<0.2f) thlo = 0.0f;
  //Thrust_command = (2.95f*thlo-4.8f*thlo*thlo+2.69f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  Thrust_command = (2.97f*thlo-4.94f*thlo*thlo+2.86f*thlo*thlo*thlo)*BATTERY_VOLTAGE;

  //Thrust_command = thlo*BATTERY_VOLTAGE;

  #ifdef MINIJOYC
  Roll_angle_command = 0.6*Stick[AILERON];
  if (Roll_angle_command<-1.0f)Roll_angle_command = -1.0f;
  if (Roll_angle_command> 1.0f)Roll_angle_command =  1.0f;  
  Pitch_angle_command = 0.6*Stick[ELEVATOR];
  if (Pitch_angle_command<-1.0f)Pitch_angle_command = -1.0f;
  if (Pitch_angle_command> 1.0f)Pitch_angle_command =  1.0f;  
  #else
  Roll_angle_command = 0.4*Stick[AILERON];
  if (Roll_angle_command<-1.0f)Roll_angle_command = -1.0f;
  if (Roll_angle_command> 1.0f)Roll_angle_command =  1.0f;  
  Pitch_angle_command = 0.4*Stick[ELEVATOR];
  if (Pitch_angle_command<-1.0f)Pitch_angle_command = -1.0f;
  if (Pitch_angle_command> 1.0f)Pitch_angle_command =  1.0f;  
  #endif
  Yaw_angle_command = Stick[RUDDER];
  if (Yaw_angle_command<-1.0f)Yaw_angle_command = -1.0f;
  if (Yaw_angle_command> 1.0f)Yaw_angle_command =  1.0f;  
  //Yaw control
  Yaw_rate_reference   = 1.5f * PI * (Yaw_angle_command - Rudder_center);

  if (Control_mode == RATECONTROL)
  {
    Roll_rate_reference = 240*PI/180*Roll_angle_command;
    Pitch_rate_reference = 240*PI/180*Pitch_angle_command;
  }

  // flip button check
  if (Flip_flag == 0)
  {
    Flip_flag = get_flip_button();
  }

  //USBSerial.printf("%5.2f %5.2f %5.2f %5.2f \n\r", 
  //  Stick[THROTTLE], Stick[RUDDER], Stick[AILERON], Stick[ELEVATOR]);

}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  //Control main
  if(rc_isconnected())
  {
    if(Thrust_command/BATTERY_VOLTAGE < Motor_on_duty_threshold)
    {      
      FrontRight_motor_duty = 0.0;
      FrontLeft_motor_duty = 0.0;
      RearRight_motor_duty = 0.0;
      RearLeft_motor_duty = 0.0;
      motor_stop();
      p_pid.reset();
      q_pid.reset();
      r_pid.reset();
      Roll_rate_reference = 0.0f;
      Pitch_rate_reference = 0.0f;
      Yaw_rate_reference = 0.0f;
      Rudder_center   = Yaw_angle_command;
    }
    else
    {

      //Control angle velocity
      p_rate = Roll_rate;
      q_rate = Pitch_rate;
      r_rate = Yaw_rate;

      //Get reference
      p_ref = Roll_rate_reference;
      q_ref = Pitch_rate_reference;
      r_ref = Yaw_rate_reference;

      //Error
      p_err = p_ref - p_rate;
      q_err = q_ref - q_rate;
      r_err = r_ref - r_rate;

      //Rate Control PID
      Roll_rate_command = p_pid.update(p_err);
      Pitch_rate_command = q_pid.update(q_err);
      Yaw_rate_command = r_pid.update(r_err);

      //Motor Control
      //正規化Duty
      FrontRight_motor_duty = (Thrust_command +(-Roll_rate_command +Pitch_rate_command +Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
      FrontLeft_motor_duty =  (Thrust_command +( Roll_rate_command +Pitch_rate_command -Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
      RearRight_motor_duty =  (Thrust_command +(-Roll_rate_command -Pitch_rate_command -Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
      RearLeft_motor_duty =   (Thrust_command +( Roll_rate_command -Pitch_rate_command +Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
      
      const float minimum_duty=0.0f;
      const float maximum_duty=0.95f;

      if (FrontRight_motor_duty < minimum_duty) FrontRight_motor_duty = minimum_duty;
      if (FrontRight_motor_duty > maximum_duty) FrontRight_motor_duty = maximum_duty;

      if (FrontLeft_motor_duty < minimum_duty) FrontLeft_motor_duty = minimum_duty;
      if (FrontLeft_motor_duty > maximum_duty) FrontLeft_motor_duty = maximum_duty;

      if (RearRight_motor_duty < minimum_duty) RearRight_motor_duty = minimum_duty;
      if (RearRight_motor_duty > maximum_duty) RearRight_motor_duty = maximum_duty;

      if (RearLeft_motor_duty < minimum_duty) RearLeft_motor_duty = minimum_duty;
      if (RearLeft_motor_duty > maximum_duty) RearLeft_motor_duty = maximum_duty;

      //Duty set
      if (OverG_flag==0){
        set_duty_fr(FrontRight_motor_duty);
        set_duty_fl(FrontLeft_motor_duty);
        set_duty_rr(RearRight_motor_duty);
        set_duty_rl(RearLeft_motor_duty);      
      }
      else 
      {
        FrontRight_motor_duty = 0.0;
        FrontLeft_motor_duty = 0.0;
        RearRight_motor_duty = 0.0;
        RearLeft_motor_duty = 0.0;
        motor_stop();
        OverG_flag=0;
        Mode = PARKING_MODE;
      }
      //USBSerial.printf("%12.5f %12.5f %12.5f %12.5f\n",FrontRight_motor_duty, FrontLeft_motor_duty, RearRight_motor_duty, RearLeft_motor_duty);
    }
  }
  else{
    motor_stop();    
  } 
}

void angle_control(void)
{
  float phi_err,theta_err;
  static uint8_t cnt=0;
  static float timeval=0.0f;
  //flip
  uint16_t flip_delay = 150; 
  uint16_t flip_step;
  float domega;

  if (Control_mode == RATECONTROL) return;

  //USBSerial.printf("On=%d Off=%d Flip=%d Counter=%d\r\n", BtnA_on_flag, BtnA_off_flag, Flip_flag, Flip_counter);

  //PID Control
  if ((Thrust_command/BATTERY_VOLTAGE < Motor_on_duty_threshold))//Angle_control_on_duty_threshold))
  {
    //Initialize
    Roll_rate_reference=0.0f;
    Pitch_rate_reference=0.0f;
    phi_err = 0.0f;
    theta_err = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Flip_flag = 0;
    Flip_counter = 0;

    /////////////////////////////////////
    // 以下の処理で、角度制御が有効になった時に
    // 急激な目標値が発生して機体が不安定になるのを防止する
    Aileron_center  = Roll_angle_command;
    Elevator_center = Pitch_angle_command;

    Roll_angle_offset   = 0;
    Pitch_angle_offset = 0;
    /////////////////////////////////////
  }
  else
  {
    //Flip
    if (Flip_flag == 1)
    { 
      #if 1
      Led_color = 0xFF9933;

      //PID Reset
      phi_pid.reset();
      theta_pid.reset();
    
      //Flip
      Flip_time = 0.4;
      Pitch_rate_reference= 0.0;
      domega = 0.0025f*8.0*PI/Flip_time/Flip_time;
      flip_delay = 150;
      flip_step = (uint16_t)(Flip_time/0.0025f);
      if (Flip_counter < flip_delay)
      {
        Roll_rate_reference = 0.0f;
        Thrust_command = T_flip*1.2;
      }
      else if (Flip_counter < (flip_step/4 + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference + domega;
        Thrust_command = T_flip*1.05;
      }
      else if (Flip_counter < (2*flip_step/4 + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference + domega;
        Thrust_command = T_flip*1.0;
      }
      else if (Flip_counter < (3*flip_step/4 + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference - domega;
        Thrust_command = T_flip*1.0;
      }
      else if (Flip_counter < (flip_step + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference - domega;
        Thrust_command = T_flip*1.4;
      }
      else if (Flip_counter < (flip_step + flip_delay + 100) )
      {
        if(Ahrs_reset_flag == 0) 
        {
          Ahrs_reset_flag = 1;
          ahrs_reset();
        }
        Roll_rate_reference = 0.0;
        Thrust_command=T_flip*1.4;
      }
      else
      {
        Flip_flag = 0;
        Ahrs_reset_flag = 0;
      }
      Flip_counter++;
      #endif  
    }
    else
    {
      //flip reset
      //Flip_flag = 1;
      Roll_rate_reference = 0;
      T_flip = Thrust_command;
      Ahrs_reset_flag = 0;
      Flip_counter = 0;

      //Angle Control
      Led_color = RED;
      //Get Roll and Pitch angle ref 
      Roll_angle_reference  = 0.5f * PI * (Roll_angle_command - Aileron_center);
      Pitch_angle_reference = 0.5f * PI * (Pitch_angle_command - Elevator_center);
      if (Roll_angle_reference > (30.0f*PI/180.0f) ) Roll_angle_reference = 30.0f*PI/180.0f;
      if (Roll_angle_reference <-(30.0f*PI/180.0f) ) Roll_angle_reference =-30.0f*PI/180.0f;
      if (Pitch_angle_reference > (30.0f*PI/180.0f) ) Pitch_angle_reference = 30.0f*PI/180.0f;
      if (Pitch_angle_reference <-(30.0f*PI/180.0f) ) Pitch_angle_reference =-30.0f*PI/180.0f;

      //Error
      phi_err   = Roll_angle_reference   - (Roll_angle - Roll_angle_offset );
      theta_err = Pitch_angle_reference - (Pitch_angle - Pitch_angle_offset);
    
      //PID
      Roll_rate_reference = phi_pid.update(phi_err);
      Pitch_rate_reference = theta_pid.update(theta_err);
    } 
  }
}

void set_duty_fr(float duty){ledcWrite(FrontRight_motor, (uint32_t)(255*duty));}
void set_duty_fl(float duty){ledcWrite(FrontLeft_motor, (uint32_t)(255*duty));}
void set_duty_rr(float duty){ledcWrite(RearRight_motor, (uint32_t)(255*duty));}
void set_duty_rl(float duty){ledcWrite(RearLeft_motor, (uint32_t)(255*duty));}

void onboard_led(CRGB p, uint8_t state)
{
  if (state ==1) led_onboard[0]=p;
  else led_onboard[0]=0;
   //Update LED
  //FastLED.show();
  return;
}

void esp_led(CRGB p, uint8_t state)
{
  if (state ==1) led_esp[0]=p;
  else led_esp[0]=0;
   //Update LED
  //FastLED.show();
  return;
}



void init_pwm(void)
{
  ledcSetup(FrontLeft_motor, freq, resolution);
  ledcSetup(FrontRight_motor, freq, resolution);
  ledcSetup(RearLeft_motor, freq, resolution);
  ledcSetup(RearRight_motor, freq, resolution);
  ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
  ledcAttachPin(pwmFrontRight, FrontRight_motor);
  ledcAttachPin(pwmRearLeft, RearLeft_motor);
  ledcAttachPin(pwmRearRight, RearRight_motor);
}

uint8_t get_arming_button(void)
{
  static int8_t chatta=0;
  static uint8_t state=0;
  if( (int)Stick[BUTTON_ARM] == 1 )
  { 
    chatta++;
    if(chatta>10){
      chatta=10;
      state=1;
    }
  }
  else
  {
    chatta--;
    if(chatta<-10)
    {    
      chatta=-10;
      state=0;
    }
    
  }
  //USBSerial.println(state);
  return state;
}

uint8_t get_flip_button(void)
{
  static int8_t chatta=0;
  static uint8_t state=0;
  if( (int)Stick[BUTTON_FLIP] == 1 )
  { 
    chatta++;
    if(chatta>10){
      chatta=10;
      state=1;
    }
  }
  else
  {
    chatta--;
    if(chatta<-10)
    {    
      chatta=-10;
      state=0;
    }
    
  }
  //USBSerial.println(state);
  return state;
}

const uint8_t MAXINDEX=98;

void telemetry(void)
{
  uint8_t senddata[MAXINDEX]; 

  if(Telem_mode==0)
  {
    Telem_mode = 1;
    make_telemetry_header_data(senddata);

    //Send !
    telemetry_send(senddata, sizeof(senddata));
  }  
  else if(Mode > AVERAGE_MODE)
  {
    if (Telem_cnt == 0)telemetry_sequence();
    Telem_cnt++;
    if (Telem_cnt>10-1)Telem_cnt = 0;
  }
}


void telemetry_sequence(void)
{
  uint8_t senddata[MAXINDEX]; 

  switch (Telem_mode)
  {
    case 1:
      make_telemetry_data(senddata);
      //Send !
      if(telemetry_send(senddata, sizeof(senddata))==1)esp_led(0x110000, 1);//Telemetory Reciver OFF
      else esp_led(0x001100, 1);//Telemetory Reciver ON

      //Telem_mode = 2;
      break;
  }
}


void make_telemetry_header_data(uint8_t* senddata)
{
  float d_float;
  uint8_t d_int[4];
  uint8_t index=0;  

    index=2;
    for (uint8_t i=0;i<(MAXINDEX-2)/4;i++)
    {
      data2log(senddata, 0.0f, index);
      //d_float = 0.0;
      //float2byte(d_float, d_int);
      //append_data(senddata, d_int, index, 4);
      index = index + 4;
    }
    //Telemetry Header
    senddata[0]=99;
    senddata[1]=99;
    index=2;
    //Roll_rate_kp
    data2log(senddata, Roll_rate_kp, index);
    //d_float = Roll_rate_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Roll_rate_ti
    data2log(senddata, Roll_rate_ti, index);
    //d_float = Roll_rate_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Roll_rate_td
    data2log(senddata, Roll_rate_td, index);
    //d_float = Roll_rate_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Roll_rate_eta
    data2log(senddata, Roll_rate_eta, index);
    //d_float = Roll_rate_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Pitch_rate_kp
    data2log(senddata, Pitch_rate_kp, index);
    //d_float = Pitch_rate_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_rate_ti
    data2log(senddata, Pitch_rate_ti, index);
    //d_float = Pitch_rate_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_rate_td
    data2log(senddata, Pitch_rate_td, index);
    //d_float = Pitch_rate_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_rate_eta
    data2log(senddata, Pitch_rate_eta, index);
    //d_float = Pitch_rate_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Yaw_rate_kp
    data2log(senddata, Yaw_rate_kp, index);
    //d_float = Yaw_rate_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Yaw_rate_ti
    data2log(senddata, Yaw_rate_ti, index);
    //d_float = Yaw_rate_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Yaw_rate_td
    data2log(senddata, Yaw_rate_td, index);
    //d_float = Yaw_rate_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Yaw_rate_eta
    data2log(senddata, Yaw_rate_eta, index);
    //d_float = Yaw_rate_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Rall_angle_kp
    data2log(senddata, Rall_angle_kp, index);
    //d_float = Rall_angle_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Rall_angle_ti
    data2log(senddata, Rall_angle_ti, index);
    //d_float = Rall_angle_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Rall_angle_td
    data2log(senddata, Rall_angle_td, index);
    //d_float = Rall_angle_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Rall_angle_eta
    data2log(senddata, Rall_angle_eta, index);
    //d_float = Rall_angle_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_angle_kp
    data2log(senddata, Pitch_angle_kp, index);
    //d_float = Pitch_angle_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_angle_ti
    data2log(senddata, Pitch_angle_ti, index);
    //d_float = Pitch_angle_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_angle_td
    data2log(senddata, Pitch_angle_td, index);
    //d_float = Pitch_angle_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Pitch_angle_eta
    data2log(senddata, Pitch_angle_eta, index);
    //d_float = Pitch_angle_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
}

void make_telemetry_data(uint8_t* senddata)
{
  const uint8_t MAXINDEX=98;
  float d_float;
  uint8_t d_int[4];
  //uint8_t senddata[MAXINDEX]; 
  uint8_t index=0;  

  //Telemetry Header
  senddata[0]=88;
  senddata[1]=88;
  index = 2;
  //1 Time
  data2log(senddata, Elapsed_time, index);
  index = index + 4;
  //2 delta Time
  data2log(senddata, Interval_time, index);
  index = index + 4;
  //3 Roll_angle
  data2log(senddata, (Roll_angle-Roll_angle_offset)*180/PI, index);
  index = index + 4;
  //4 Pitch_angle
  data2log(senddata, (Pitch_angle-Pitch_angle_offset)*180/PI, index);
  index = index + 4;
  //5 Yaw_angle
  data2log(senddata, (Yaw_angle-Yaw_angle_offset)*180/PI, index);
  index = index + 4;
  //6 P
  data2log(senddata, (Roll_rate)*180/PI, index);
  index = index + 4;
  //7 Q
  data2log(senddata, (Pitch_rate)*180/PI, index);
  index = index + 4;
  //8 R
  data2log(senddata, (Yaw_rate)*180/PI, index);
  index = index + 4;
  //9 Roll_angle_reference
  data2log(senddata, Roll_angle_reference*180/PI, index);
  //data2log(senddata, 0.5f * 180.0f *Roll_angle_command, index);
  index = index + 4;
  //10 Pitch_angle_reference
  data2log(senddata, Pitch_angle_reference*180/PI, index);
  //data2log(senddata, 0.5 * 189.0f* Pitch_angle_command, index);
  index = index + 4;
  //11 P ref
  data2log(senddata, Roll_rate_reference*180/PI, index);
  index = index + 4;
  //12 Q ref
  data2log(senddata, Pitch_rate_reference*180/PI, index);
  index = index + 4;
  //13 R ref
  data2log(senddata, Yaw_rate_reference*180/PI, index);
  index = index + 4;
  //14 T ref
  data2log(senddata, Thrust_command/BATTERY_VOLTAGE, index);
  index = index + 4;
  //15 Voltage
  data2log(senddata, Voltage, index);
  index = index + 4;
  //16 Accel_x_raw
  data2log(senddata, Accel_x_raw, index);
  index = index + 4;
  //17 Accel_y_raw
  data2log(senddata, Accel_y_raw, index);
  index = index + 4;
  //18 Accel_z_raw
  data2log(senddata, Accel_z_raw, index);
  index = index + 4;
  //19 Acc Norm
  data2log(senddata, Acc_norm, index);
  index = index + 4;
  //20 FrontRight_motor_duty
  data2log(senddata, FrontRight_motor_duty, index);
  index = index + 4;
  //21 FrontLeft_motor_duty
  data2log(senddata, FrontLeft_motor_duty, index);
  index = index + 4;
  //22 RearRight_motor_duty
  data2log(senddata, RearRight_motor_duty, index);
  index = index + 4;
  //23 RearLeft_motor_duty
  data2log(senddata, RearLeft_motor_duty, index);
  index = index + 4;
  //24 Altitude2
  data2log(senddata, Altitude2, index);
  index = index + 4;
}

void data2log(uint8_t* data_list, float add_data, uint8_t index)
{
    uint8_t d_int[4];
    float d_float = add_data;
    float2byte(d_float, d_int);
    append_data(data_list, d_int, index, 4);
}

void float2byte(float x, uint8_t* dst)
{
  uint8_t* dummy;
  dummy = (uint8_t*)&x;
  dst[0]=dummy[0];
  dst[1]=dummy[1];
  dst[2]=dummy[2];
  dst[3]=dummy[3];
}

void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len)
{
  for(uint8_t i=index;i<index+len;i++)
  {
    data[i]=newdata[i-index];
  }
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

