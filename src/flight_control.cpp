#include "flight_control.hpp"

const int pwmFL = 7;
const int pwmFR = 1;
const int pwmRL = 12;
const int pwmRR = 42;

const int freq = 150000;
const int FL_motor = 1;
const int FR_motor = 2;
const int RL_motor = 3;
const int RR_motor = 4;
const int resolution = 8;

//Control period
const float Control_period = 0.0025f;//400Hz

//PID Gain
//Rate control PID gain
const float P_kp = 0.65f;
const float P_ti = 0.7f;
const float P_td = 0.03f;
const float P_eta = 0.125f;

const float Q_kp = 0.65f;
const float Q_ti = 0.7f;
const float Q_td = 0.03f;
const float Q_eta = 0.125f;

const float R_kp = 3.0f;
const float R_ti = 0.8f;
const float R_td = 0.000f;
const float R_eta = 0.125f;

//Angle control PID gain
const float Phi_kp = 8.0f;//12
const float Phi_ti = 4.0f;
const float Phi_td = 0.04f;
const float Phi_eta = 0.125f;

const float Tht_kp = 8.0f;//12
const float Tht_ti = 4.0f;
const float Tht_td = 0.04f;
const float Tht_eta = 0.125f;

//Times
volatile float Elapsed_time=0.0f;
volatile float Old_Elapsed_time=0.0f;

//
volatile uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,Dt_time=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
volatile float FR_duty=0.0f, FL_duty=0.0f, RR_duty=0.0f, RL_duty=0.0f;
volatile float P_com=0.0f, Q_com=0.0f, R_com=0.0f;
volatile float Phi_com=0.0f, Tht_com=0.0f, Psi_com=0.0f;
volatile float T_ref=0.0f;
//volatile float Pbias=0.0f, Qbias=0.0f, Rbias=0.0f;
volatile float Phi_bias=0.0f, Theta_bias=0.0f, Psi_bias=0.0f;  
volatile float Phi_ref=0.0f, Theta_ref=0.0f, Psi_ref=0.0f;
volatile float Elevator_center=0.0f, Aileron_center=0.0f, Rudder_center=0.0f;
volatile float Pref=0.0f, Qref=0.0f, Rref=0.0f;
volatile float Phi_trim   =  0.8f*PI/180.0f;
volatile float Theta_trim = -0.2f*PI/180.0f;
volatile float Psi_trim   =  0.0f;

//Log
uint8_t Logflag=0;
uint8_t Telem_cnt = 0;

//Machine state
float Timevalue=0.0f;
uint8_t Mode = INIT_MODE;
uint8_t Control_mode = ANGLECONTROL;
volatile uint8_t LockMode=0;
float Motor_on_duty_threshold = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
uint8_t Flip_flag = 0;
uint16_t Flip_counter = 0; 
float Flip_time = 2.0;
int8_t BtnA_counter = 0;
uint8_t BtnA_on_flag = 0;
uint8_t BtnA_off_flag =1;
volatile uint8_t Loop_flag = 0;
volatile uint8_t Angle_control_flag = 0;
uint32_t Led_color = 0x000000;

//PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;

void init_pwm();
void control_init();
void variable_init(void);
void m5_atom_led(CRGB p, uint8_t state);
void get_command(void);
void angle_control(void);
void rate_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void motor_stop(void);
uint8_t lock_com(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);
void telemetry(void);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len);
void data2log(uint8_t* data_list, float add_data, uint8_t index);

CRGB leds[NUM_LEDS];

hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() 
{
  Loop_flag = 1;
}

void init_copter(void)
{
  Mode = INIT_MODE;

  //LED initialaze
  FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);
  leds[0]=0;
  FastLED.show();

  USBSerial.begin(115200);
  delay(2000);
  USBSerial.printf("Start StampS3FPV!\r\n");
  
  init_pwm();
  rc_init();
  leds[0]=WHITE;
  FastLED.show();
  sensor_init();
  control_init();

  //割り込み設定
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
  
  while(!rc_isconnected());
  //Mode = AVERAGE_MODE;
}

//Main loop
void loop_400Hz(void)
{
  static uint8_t led=1;

  while(Loop_flag==0);
  E_time = micros();
  Old_Elapsed_time = Elapsed_time;
  Elapsed_time = 1e-6*(E_time - S_time);
  Loop_flag = 0;
  Timevalue+=0.0025f;

  //Read Sensor Value
  sensor_read();


  //Begin Mode select
  if (Mode == INIT_MODE)
  {
      motor_stop();
      Elevator_center = 0.0f;
      Aileron_center = 0.0f;
      Rudder_center = 0.0f;
      Pbias = 0.0f;
      Qbias = 0.0f;
      Rbias = 0.0f;
      Phi_bias = 0.0f;
      Theta_bias = 0.0f;
      Psi_bias = 0.0f;
      Mode = AVERAGE_MODE;
      return;
  }
  else if (Mode == AVERAGE_MODE)
  {
    motor_stop();
    //Gyro Bias Estimate
    if (BiasCounter < AVERAGENUM)
    {
      //Sensor Read
      m5_atom_led(PERPLE, 1);
      //sensor_read();
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      BiasCounter++;
      return;
    }
    else if(BiasCounter == AVERAGENUM)
    {
      //Average calc
      Pbias = Pbias/AVERAGENUM;
      Qbias = Qbias/AVERAGENUM;
      Rbias = Rbias/AVERAGENUM;

      //Mode change
      Mode = STAY_MODE;
      S_time = micros();
    }
    return;
  }
  else if( Mode == FLIGHT_MODE)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        led=0;
        if(Power_flag==0)m5_atom_led(GREEN,led);
        else m5_atom_led(POWEROFFCOLOR,led);
        //if( (Elapsed_time - Old_Elapsed_time)>0.00251) m5_atom_led(0xffffff,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Mode=STAY_MODE;
      }
      return;
    }
    //LED Blink
    if (Power_flag == 0) m5_atom_led(Led_color, led);
    else m5_atom_led(POWEROFFCOLOR,led);
    //if( (Elapsed_time - Old_Elapsed_time)>0.00251) m5_atom_led(0xffffff,led);
    led=1;

    //Get command
    get_command();

    //Angle Control
    angle_control();

    //Rate Control
    rate_control();
  }
  else if(Mode == STAY_MODE)
  {
    motor_stop();
    //Befor takeoff Voltage Low Check
    if(Voltage<3.7)
    {
      Power_flag = POWER_FLG_MAX;
    }
    OverG_flag = 0;
    Angle_control_flag = 0;
    if(LedBlinkCounter<10){
      if (Power_flag == 0) m5_atom_led(GREEN, 1);
      else m5_atom_led(POWEROFFCOLOR,1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      if (Power_flag == 0) m5_atom_led(GREEN, 0);
      else m5_atom_led(POWEROFFCOLOR,0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
      
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        LockMode=2;//Enable Flight
        Mode=FLIGHT_MODE;
      }
      return;
    }
  }

  //Telemetry
  if (Telem_cnt == 0)telemetry();
  Telem_cnt++;
  if (Telem_cnt>10-1)Telem_cnt = 0;

  D_time = micros();
  if(Telem_cnt == 1)Dt_time = D_time - E_time;
  //End Mode select
  //End of Loop_400Hz function
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
  p_pid.set_parameter(P_kp, P_ti, P_td, P_eta, Control_period);//Roll rate control gain
  q_pid.set_parameter(Q_kp, Q_ti, Q_td, Q_eta, Control_period);//Pitch rate control gain
  r_pid.set_parameter(R_kp, R_ti, R_td, R_eta, Control_period);//Yaw rate control gain
  //Roll P gain を挙げてみて分散が減るかどうか考える
  //Roll Ti を大きくしてみる

  //Angle control
  phi_pid.set_parameter  (Phi_kp, Phi_ti, Phi_td, Phi_eta, Control_period);//Roll angle control gain
  theta_pid.set_parameter(Tht_kp, Tht_ti, Tht_td, Tht_eta, Control_period);//Pitch angle control gain

}
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void get_command(void)
{
  Control_mode = Stick[CONTROLMODE];

  //if(OverG_flag == 1){
  //  T_ref = 0.0;
  //}
  //Throttle curve conversion　スロットルカーブ補正
  float thlo = Stick[THROTTLE];
  if (thlo>1.0f) thlo = 1.0f;
  if (thlo<0.02f) thlo = 0.0f;

  //T_ref = (3.27f*thlo -5.31f*thlo*thlo + 3.04f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (2.92f*thlo -4.90f*thlo*thlo + 2.88f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.01f*thlo -5.20f*thlo*thlo + 3.14f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.46f*thlo -5.74f*thlo*thlo + 3.23f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.42f*thlo -6.00f*thlo*thlo + 3.58f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.32f*thlo -5.40f*thlo*thlo + 3.03f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  T_ref = (3.07f*thlo -3.88f*thlo*thlo + 1.75f*thlo*thlo*thlo)*BATTERY_VOLTAGE;

  #ifdef MINIJOYC
  Phi_com = 0.6*Stick[AILERON];
  if (Phi_com<-1.0f)Phi_com = -1.0f;
  if (Phi_com> 1.0f)Phi_com =  1.0f;  
  Tht_com = 0.6*Stick[ELEVATOR];
  if (Tht_com<-1.0f)Tht_com = -1.0f;
  if (Tht_com> 1.0f)Tht_com =  1.0f;  
  #else
  Phi_com = 0.4*Stick[AILERON];
  if (Phi_com<-1.0f)Phi_com = -1.0f;
  if (Phi_com> 1.0f)Phi_com =  1.0f;  
  Tht_com = 0.4*Stick[ELEVATOR];
  if (Tht_com<-1.0f)Tht_com = -1.0f;
  if (Tht_com> 1.0f)Tht_com =  1.0f;  
  #endif
  Psi_com = Stick[RUDDER];
  if (Psi_com<-1.0f)Psi_com = -1.0f;
  if (Psi_com> 1.0f)Psi_com =  1.0f;  
  //Yaw control
  Rref   = 1.5f * PI * (Psi_com - Rudder_center);

  if (Control_mode == RATECONTROL)
  {
    Pref = 240*PI/180*Phi_com;
    Qref = 240*PI/180*Tht_com;
  }

  // A button
  if (Stick[BUTTON_A]==1) BtnA_counter ++;
  else BtnA_counter --;
  if (BtnA_counter>20)
  {
    BtnA_counter=20;
    if(BtnA_off_flag==1)
    {
      BtnA_on_flag = 1;
      BtnA_off_flag = 0;
    }
  }
  if (BtnA_counter<-20)
  {
    BtnA_counter=-20;
    BtnA_on_flag = 0;
    BtnA_off_flag = 1;
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
    if(T_ref/BATTERY_VOLTAGE < Motor_on_duty_threshold)
    {
      if(Control_mode == ANGLECONTROL)
      {
        if(Flip_flag==0)Led_color=0xffff00;
        else Led_color = 0xFF9933;
      }
      else Led_color = 0xDC669B;
      
      FR_duty = 0.0;
      FL_duty = 0.0;
      RR_duty = 0.0;
      RL_duty = 0.0;
      motor_stop();
      p_pid.reset();
      q_pid.reset();
      r_pid.reset();
      Pref=0.0f;
      Qref=0.0f;
      Rref=0.0f;
      Rudder_center   = Psi_com;
    }
    else
    {

      //Control angle velocity
      p_rate = Wp - Pbias;
      q_rate = Wq - Qbias;
      r_rate = Wr - Rbias;

      //Get reference
      p_ref = Pref;
      q_ref = Qref;
      r_ref = Rref;

      //Error
      p_err = p_ref - p_rate;
      q_err = q_ref - q_rate;
      r_err = r_ref - r_rate;

      //Rate Control PID
      P_com = p_pid.update(p_err);
      Q_com = q_pid.update(q_err);
      R_com = r_pid.update(r_err);

      //Motor Control
      //正規化Duty
      FR_duty = (T_ref +(-P_com +Q_com +R_com)*0.25f)/BATTERY_VOLTAGE;
      FL_duty = (T_ref +( P_com +Q_com -R_com)*0.25f)/BATTERY_VOLTAGE;
      RR_duty = (T_ref +(-P_com -Q_com -R_com)*0.25f)/BATTERY_VOLTAGE;
      RL_duty = (T_ref +( P_com -Q_com +R_com)*0.25f)/BATTERY_VOLTAGE;
      
      const float minimum_duty=0.0f;
      const float maximum_duty=0.95f;

      if (FR_duty < minimum_duty) FR_duty = minimum_duty;
      if (FR_duty > maximum_duty) FR_duty = maximum_duty;

      if (FL_duty < minimum_duty) FL_duty = minimum_duty;
      if (FL_duty > maximum_duty) FL_duty = maximum_duty;

      if (RR_duty < minimum_duty) RR_duty = minimum_duty;
      if (RR_duty > maximum_duty) RR_duty = maximum_duty;

      if (RL_duty < minimum_duty) RL_duty = minimum_duty;
      if (RL_duty > maximum_duty) RL_duty = maximum_duty;

      //Duty set
      if (OverG_flag==0){
        set_duty_fr(FR_duty);
        set_duty_fl(FL_duty);
        set_duty_rr(RR_duty);
        set_duty_rl(RL_duty);      
      }
      else 
      {
        FR_duty = 0.0;
        FL_duty = 0.0;
        RR_duty = 0.0;
        RL_duty = 0.0;
        motor_stop();
        OverG_flag=0;
        LockMode = 0;
        Mode = STAY_MODE;
      }
      //USBSerial.printf("%12.5f %12.5f %12.5f %12.5f\n",FR_duty, FL_duty, RR_duty, RL_duty);
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

  if (Control_mode == RATECONTROL) return;

  //USBSerial.printf("On=%d Off=%d Flip=%d Counter=%d\r\n", BtnA_on_flag, BtnA_off_flag, Flip_flag, Flip_counter);

  //PID Control
  if ((T_ref/BATTERY_VOLTAGE < Motor_on_duty_threshold))//Angle_control_on_duty_threshold))
  {
    Pref=0.0f;
    Qref=0.0f;
    phi_err = 0.0f;
    theta_err = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Phi_ref);
    theta_pid.set_error(Theta_ref);
    Flip_flag = 0;
    Flip_counter = 0;

    /////////////////////////////////////
    // 以下の処理で、角度制御が有効になった時に
    // 急激な目標値が発生して機体が不安定になるのを防止する
    Aileron_center  = Phi_com;
    Elevator_center = Tht_com;

    Phi_bias   = 0;
    Theta_bias = 0;
    /////////////////////////////////////
  
  }
  else
  {
    
    //Flip
    if (0)// (BtnA_on_flag == 1) || (Flip_flag == 1))
    { 
      #if 0
      Led_color = 0xFF9933;

      BtnA_on_flag = 0;
      Flip_flag = 1;

      //PID Reset
      phi_pid.reset();
      theta_pid.reset();
    
      Flip_time = 0.26;
      Pref = 2.0*PI/Flip_time;
      Qref = 0.0;
      if (Flip_counter > (uint16_t)(Flip_time/0.0025) )
      {
        Flip_counter = (uint16_t)(Flip_time/0.0025);
        Flip_flag = 0;
        Flip_counter = 0;
      }
      Flip_counter++;
      #endif  
    }
    
    //Angle Control
    else
    {
      Led_color = RED;
      //Get Roll and Pitch angle ref 
      Phi_ref   = 0.5f * PI * (Phi_com - Aileron_center);
      Theta_ref = 0.5f * PI * (Tht_com - Elevator_center);
      if (Phi_ref > (30.0f*PI/180.0f) ) Phi_ref = 30.0f*PI/180.0f;
      if (Phi_ref <-(30.0f*PI/180.0f) ) Phi_ref =-30.0f*PI/180.0f;
      if (Theta_ref > (30.0f*PI/180.0f) ) Theta_ref = 30.0f*PI/180.0f;
      if (Theta_ref <-(30.0f*PI/180.0f) ) Theta_ref =-30.0f*PI/180.0f;

      //Error
      phi_err   = Phi_ref   - (Phi   - Phi_bias );
      theta_err = Theta_ref - (Theta - Theta_bias);
    
      //PID
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
    } 
  }
}

void set_duty_fr(float duty){ledcWrite(FR_motor, (uint32_t)(255*duty));}
void set_duty_fl(float duty){ledcWrite(FL_motor, (uint32_t)(255*duty));}
void set_duty_rr(float duty){ledcWrite(RR_motor, (uint32_t)(255*duty));}
void set_duty_rl(float duty){ledcWrite(RL_motor, (uint32_t)(255*duty));}


void m5_atom_led(CRGB p, uint8_t state)
{
  if (state ==1) leds[0]=p;
  else leds[0]=0;
   //Update LED
  FastLED.show();
  return;
}


void init_pwm(void)
{
  ledcSetup(FL_motor, freq, resolution);
  ledcSetup(FR_motor, freq, resolution);
  ledcSetup(RL_motor, freq, resolution);
  ledcSetup(RR_motor, freq, resolution);
  ledcAttachPin(pwmFL, FL_motor);
  ledcAttachPin(pwmFR, FR_motor);
  ledcAttachPin(pwmRL, RL_motor);
  ledcAttachPin(pwmRR, RR_motor);
}


void telemetry(void)
{
  //Telemetry
  const uint8_t MAXINDEX=98;
  float d_float;
  uint8_t d_int[4];
  uint8_t senddata[MAXINDEX]; 
  uint8_t index=0;  

  if(Logflag==0)
  {
    Logflag = 1;
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
    //P_kp
    data2log(senddata, P_kp, index);
    //d_float = P_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //P_ti
    data2log(senddata, P_ti, index);
    //d_float = P_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //P_td
    data2log(senddata, P_td, index);
    //d_float = P_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //P_eta
    data2log(senddata, P_eta, index);
    //d_float = P_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Q_kp
    data2log(senddata, Q_kp, index);
    //d_float = Q_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Q_ti
    data2log(senddata, Q_ti, index);
    //d_float = Q_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Q_td
    data2log(senddata, Q_td, index);
    //d_float = Q_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Q_eta
    data2log(senddata, Q_eta, index);
    //d_float = Q_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //R_kp
    data2log(senddata, R_kp, index);
    //d_float = R_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //R_ti
    data2log(senddata, R_ti, index);
    //d_float = R_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //R_td
    data2log(senddata, R_td, index);
    //d_float = R_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //R_eta
    data2log(senddata, R_eta, index);
    //d_float = R_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Phi_kp
    data2log(senddata, Phi_kp, index);
    //d_float = Phi_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Phi_ti
    data2log(senddata, Phi_ti, index);
    //d_float = Phi_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Phi_td
    data2log(senddata, Phi_td, index);
    //d_float = Phi_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Phi_eta
    data2log(senddata, Phi_eta, index);
    //d_float = Phi_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Tht_kp
    data2log(senddata, Tht_kp, index);
    //d_float = Tht_kp;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Tht_ti
    data2log(senddata, Tht_ti, index);
    //d_float = Tht_ti;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Tht_td
    data2log(senddata, Tht_td, index);
    //d_float = Tht_td;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;
    //Tht_eta
    data2log(senddata, Tht_eta, index);
    //d_float = Tht_eta;
    //float2byte(d_float, d_int);
    //append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Send !
    telemetry_send(senddata, sizeof(senddata));
  }  
  else if(Mode > AVERAGE_MODE)
  {
    //Telemetry Header
    senddata[0]=88;
    senddata[1]=88;
    index = 2;
    //1 Time
    data2log(senddata, Elapsed_time, index);
    index = index + 4;
    //2 delta Time
    data2log(senddata, 1e-6*Dt_time, index);
    index = index + 4;
    //3 Phi
    data2log(senddata, (Phi-Phi_bias)*180/PI, index);
    index = index + 4;
    //4 Theta
    data2log(senddata, (Theta-Theta_bias)*180/PI, index);
    index = index + 4;
    //5 Psi
    data2log(senddata, (Psi-Psi_bias)*180/PI, index);
    index = index + 4;
    //6 P
    data2log(senddata, (Wp-Pbias)*180/PI, index);
    index = index + 4;
    //7 Q
    data2log(senddata, (Wq-Qbias)*180/PI, index);
    index = index + 4;
    //8 R
    data2log(senddata, (Wr-Rbias)*180/PI, index);
    index = index + 4;
    //9 Phi_ref
    data2log(senddata, Phi_ref*180/PI, index);
    //data2log(senddata, 0.5f * 180.0f *Phi_com, index);
    index = index + 4;
    //10 Theta_ref
    data2log(senddata, Theta_ref*180/PI, index);
    //data2log(senddata, 0.5 * 189.0f* Tht_com, index);
    index = index + 4;
    //11 P ref
    data2log(senddata, Pref*180/PI, index);
    index = index + 4;
    //12 Q ref
    data2log(senddata, Qref*180/PI, index);
    index = index + 4;
    //13 R ref
    data2log(senddata, Rref*180/PI, index);
    index = index + 4;
    //14 T ref
    data2log(senddata, T_ref/BATTERY_VOLTAGE, index);
    index = index + 4;
    //15 Voltage
    data2log(senddata, Voltage, index);
    index = index + 4;
    //16 Ax
    data2log(senddata, Ax, index);
    index = index + 4;
    //17 Ay
    data2log(senddata, Ay, index);
    index = index + 4;
    //18 Az
    data2log(senddata, Az, index);
    index = index + 4;
    //19 Acc Norm
    data2log(senddata, Acc_norm, index);
    index = index + 4;
    //20 FR_duty
    data2log(senddata, FR_duty, index);
    index = index + 4;
    //21 FL_duty
    data2log(senddata, FL_duty, index);
    index = index + 4;
    //22 RR_duty
    data2log(senddata, RR_duty, index);
    index = index + 4;
    //23 RL_duty
    data2log(senddata, RL_duty, index);
    index = index + 4;
    //24 Altitude2
    data2log(senddata, Altitude2, index);
    index = index + 4;

    //Send !
    telemetry_send(senddata, sizeof(senddata));
  }
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

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( (int)Stick[BUTTON] == 1 )
  { 
    chatta++;
    if(chatta>20){
      chatta=20;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }
  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

