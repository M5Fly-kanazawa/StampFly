#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp" 
#include "flight_control.hpp"

/************ BEEP ************/
//BeepPWM出力Pinのアサイン
#define BEEP 40

//モータPWM周波数 
//Beep PWM Frequency [Hz]
int32_t beep_freq = 330;

//PWM分解能
//PWM Resolution
const int beep_resolution = 12;

//モータチャンネルのアサイン
//BEEP Channel
const int beep_channel  = 7;
/************ BEEP ************/


Madgwick Drone_ahrs;
Alt_kalman EstimatedAltitude;

INA3221 ina3221(INA3221_ADDR40_GND);// Set I2C address to 0x40 (A0 pin -> GND)
Filter acc_filter;
Filter az_filter;
Filter voltage_filter;

//Sensor data
volatile float Roll_angle=0.0f, Pitch_angle=0.0f, Yaw_angle=0.0f;
volatile float Roll_rate, Pitch_rate, Yaw_rate;
volatile float Roll_rate_offset=0.0f, Pitch_rate_offset=0.0f, Yaw_rate_offset=0.0f;
volatile float Accel_z;
volatile float Accel_z_offset=0.0f;
volatile float Accel_x_raw,Accel_y_raw,Accel_z_raw;
volatile float Roll_rate_raw,Pitch_rate_raw,Yaw_rate_raw;
volatile float Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
volatile float Altitude = 0.0f;
volatile float Altitude2 = 0.0f;
volatile float Alt_velocity = 0.0f;
volatile uint8_t Alt_control_ok = 0;
volatile float Az=0.0;
volatile uint16_t Offset_counter = 0;

volatile float Voltage;
float Acc_norm=0.0f;
//quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;
uint8_t OverG_flag = 0;
volatile uint8_t Under_voltage_flag = 0;
//volatile uint8_t ToF_bottom_data_ready_flag;
volatile uint16_t Range=1000;

void beep_init(void);


void beep_init(void)
{
  #if 0
  ledcSetup(beep_channel, (uint32_t)beep_freq, beep_resolution);
  ledcAttachPin(BEEP, beep_channel);
  ledcWrite(beep_channel, (uint32_t)(0));
  #endif
}

uint8_t scan_i2c()
{
  USBSerial.println ("I2C scanner. Scanning ...");
  delay(50);
  byte count = 0;
  for (uint8_t i = 1; i < 127; i++)
  {
    Wire1.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire1.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      USBSerial.print ("Found address: ");
      USBSerial.print (i, DEC);
      USBSerial.print (" (0x");
      USBSerial.print (i, HEX); 
      USBSerial.println (")");
      count++;
    }
  }
  USBSerial.print ("Found ");      
  USBSerial.print (count, DEC);        // numbers of devices
  USBSerial.println (" device(s).");
  return count;
}

void sensor_reset_offset(void)
{
  Roll_rate_offset = 0.0f;
  Pitch_rate_offset = 0.0f;
  Yaw_rate_offset = 0.0f;
  Accel_z_offset = 0.0f;
  Offset_counter = 0;
}

void sensor_calc_offset_avarage(void)
{
  Roll_rate_offset = (Offset_counter * Roll_rate_offset + Roll_rate_raw) / (Offset_counter + 1);
  Pitch_rate_offset = (Offset_counter * Pitch_rate_offset + Pitch_rate_raw) / (Offset_counter + 1);
  Yaw_rate_offset = (Offset_counter * Yaw_rate_offset + Yaw_rate_raw) / (Offset_counter + 1);
  Accel_z_offset = (Offset_counter * Accel_z_offset + Accel_z_raw) / (Offset_counter + 1);

  Offset_counter++;
}


void pipo(void)
{  
  ledcChangeFrequency(beep_channel, 2000, beep_resolution);
  ledcWrite(beep_channel, 127);
  ets_delay_us(200000);
  ledcWrite(beep_channel, 0);
  ets_delay_us(5000);
  ledcChangeFrequency(beep_channel, 1000, beep_resolution);
  ledcWrite(beep_channel, 127);
  ets_delay_us(200000);
  ledcWrite(beep_channel, 0);
}

void termin(void)
{
  while(1)
  {
    if(ToF_bottom_data_ready_flag)
    {
      ToF_bottom_data_ready_flag = 0;
      Range = tof_bottom_get_range();

      //Change Beep freqency
      beep_freq = ((int32_t)Range - 23)*5 + 100;
      beep_freq = beep_freq>>2;
      beep_freq = beep_freq<<2;
      if(beep_freq>8000)beep_freq = 8000;

      if(beep_freq<0)
      {
        ledcWrite(beep_channel, 0);
      }
      else if(beep_freq>50)
      {
        ledcChangeFrequency(beep_channel, (uint32_t)beep_freq, beep_resolution);
        ledcWrite(beep_channel, 127);
      }
      //else ledcWrite(beep_channel, 127);

      USBSerial.printf("%d %d\r\n", Range, beep_freq);
    }
  }
}

void test_voltage(void)
{
  for (uint16_t i=0; i<1000; i++)
  {
    USBSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
  }
}

void ahrs_reset(void)
{
  Drone_ahrs.reset();
}

void sensor_init()
{
  //beep_init();
 
  Wire1.begin(SDA_PIN, SCL_PIN,400000UL);
  if(scan_i2c()==0)
  {
    USBSerial.printf("No I2C device!\r\n");
    USBSerial.printf("Can not boot AtomFly2.\r\n");
    while(1);
  }

  tof_init();
  imu_init();
  Drone_ahrs.begin(400.0);
  ina3221.begin(&Wire1);
  ina3221.reset();  
  voltage_filter.set_parameter(0.005, 0.0025);
  
  uint16_t cnt=0;
  while(cnt<10)
  {
    if(ToF_bottom_data_ready_flag)
    {
      ToF_bottom_data_ready_flag = 0;
      cnt++;
      USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
    }
  }
  
  //pipo();
  delay(500);
  //test_imu();
  //termin();
  //test_voltage();  

  //Acceleration filter
  //acc_filter.set_parameter(0.005, 0.0025);
  az_filter.set_parameter(0.15, 0.0025);
  //while(1);
}

float sensor_read(void)
{
  float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
  float filterd_v;
  static float dp, dq, dr; 
  static uint16_t dcnt=0u;
  uint16_t dist;
  static float alt_time = 0.0f;
  static float sensor_time = 0.0f;
  static float old_alt_time = 0.0f;
  static uint8_t first_flag = 0;
  const uint8_t interval = 400/30+1;
  float old_alt=0.0;
  float old_sensor_time = 0.0;
  uint32_t st;
  float sens_interval;

  st = micros();
  old_sensor_time = sensor_time;
  sensor_time = (float)st*1.0e-6;
  sens_interval = sensor_time - old_sensor_time;

  //以下では航空工学の座標軸の取り方に従って
  //X軸：前後（前が正）左肩上がりが回転の正
  //Y軸：右左（右が正）頭上げが回転の正
  //Z軸：下上（下が正）右回りが回転の正
  //となる様に軸の変換を施しています
  //BMI270の座標軸の撮り方は
  //X軸：右左（右が正）頭上げが回転の正
  //Y軸：前後（前が正）左肩上がりが回転の正
  //Z軸：上下（上が正）左回りが回転の正

  imu_update();//IMUの値を読む前に必ず実行

  //Get Acc Data
  acc_x = imu_get_acc_x();
  acc_y = imu_get_acc_y();
  acc_z = imu_get_acc_z();
  //Get Gyro Data
  gyro_x = imu_get_gyro_x();
  gyro_y = imu_get_gyro_y();
  gyro_z = imu_get_gyro_z();

  Accel_x_raw =  acc_y;
  Accel_y_raw =  acc_x;
  Accel_z_raw = -acc_z;
  Roll_rate_raw  =  gyro_y;
  Pitch_rate_raw =  gyro_x;
  Yaw_rate_raw   = -gyro_z;

  if(Mode > AVERAGE_MODE)
  {
    Drone_ahrs.updateIMU( (Pitch_rate_raw-Pitch_rate_offset)*(float)RAD_TO_DEG, 
                          (Roll_rate_raw-Roll_rate_offset)*(float)RAD_TO_DEG,
                         -(Yaw_rate_raw-Yaw_rate_offset)*(float)RAD_TO_DEG,
                            Accel_y_raw, Accel_x_raw, -Accel_z_raw);
    Roll_angle  =  Drone_ahrs.getPitch()*(float)DEG_TO_RAD;
    Pitch_angle =  Drone_ahrs.getRoll()*(float)DEG_TO_RAD;
    Yaw_angle   = -Drone_ahrs.getYaw()*(float)DEG_TO_RAD;
  }

  Roll_rate  = Roll_rate_raw - Roll_rate_offset;
  Pitch_rate = Pitch_rate_raw - Pitch_rate_offset;
  Yaw_rate   = Yaw_rate_raw - Yaw_rate_offset;
  Accel_z = Accel_z_raw - Accel_z_offset;

  #if 1
  acc_norm = sqrt(Accel_x_raw*Accel_x_raw + Accel_y_raw*Accel_y_raw + Accel_z_raw*Accel_z_raw);
  Acc_norm = acc_filter.update(acc_norm ,Control_period);
  if (Acc_norm>3.8) 
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;
  }
  #endif

  //Battery voltage check 
  Voltage = ina3221.getVoltage(INA3221_CH2);
  filterd_v = voltage_filter.update(Voltage, Control_period);

  if(Under_voltage_flag != UNDER_VOLTAGE_COUNT){
    if (filterd_v < POWER_LIMIT) Under_voltage_flag ++;
    else Under_voltage_flag = 0;
    if ( Under_voltage_flag > UNDER_VOLTAGE_COUNT) Under_voltage_flag = UNDER_VOLTAGE_COUNT;
  }
  uint32_t mt=micros();

  //Altitude
  if(Mode>AVERAGE_MODE)
  Az = az_filter.update(-(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset), sens_interval);
  //USBSerial.printf("Sens_interval=%f,Az=%f, rawdata=%f\n\r", sens_interval, Az, -(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset));

  #if 1
  //Get Altitude (30Hz)
  if (dcnt>interval)
  {
    if(ToF_bottom_data_ready_flag)
    {
      ToF_bottom_data_ready_flag = 0;
      dist=tof_bottom_get_range();
      old_alt = Altitude;
      Altitude = (float)dist/1000.0;

      //外れ値除去
      if(dist==9999)Altitude = old_alt;
      else if(dist==8191)Altitude = old_alt;
      else if(dist==0)Altitude = old_alt;
      //else if (Altitude - old_alt >  0.1)Altitude = old_alt;
      //else if (Altitude - old_alt < -0.1)Altitude = old_alt;
      
      Alt_control_ok = 1;
      dcnt=0u;

      old_alt_time = alt_time;
      alt_time = micros()*1.0e-6;
      float h = alt_time - old_alt_time;
      if(first_flag == 1) EstimatedAltitude.update(Altitude, Az, h);
      else first_flag = 1;
      Altitude2 = EstimatedAltitude.Altitude;
      Alt_velocity = EstimatedAltitude.Velocity;
      //USBSerial.printf("%9.6f, %9.6f\r\n",Elapsed_time, -(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset));
      //USBSerial.printf("%9.6f, %9.6f, %9.6f, %9.6f, %9.6f\r\n",Elapsed_time,Altitude/1000.0,  Altitude2, Alt_velocity,-(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset));
    }
  }
  else dcnt++;
  #endif

  //float Roll_angle = Roll_angle;
  //float tht = Pitch_angle;
  //float Yaw_angle = Yaw_angle;
  //float sRoll_angle = sin(Roll_angle);
  //float cRoll = cos(Roll_angle);
 // float stht = sin(tht);
  //float cPitch = cos(Pitch_angle);
  //float sYaw_angle = sin(Yaw_angle);
  //float sYaw_angle = cos(Yaw_angle);

  //float r33 =  cRoll*cPitch;
  //Altitude2 = r33 * Altitude;
  //EstimatedAltitude.update(Altitude2, r33*Accel_z_raw)

  uint32_t et =micros();
  //USBSerial.printf("Sensor read %f %f %f\n\r", (mt-st)*1.0e-6, (et-mt)*1e-6, (et-st)*1.0e-6);

  return (et-st)*1.0e-6;
}

#if 0

float range = 1.0f;

float Roll_angle = 0.0f;
float tht = 0.0f;
float Yaw_angle = 0.0f;
float sRoll_angle = sin(Roll_angle);
float cRoll_angle = cos(Roll_angle);
float stht = sin(tht);
float ctht = cos(tht);
float sYaw_angle = sin(Yaw_angle);
float sYaw_angle = cos(Yaw_angle);

float r11 =  ctht*cYaw_angle;
float r12 =  sRoll_angle*stht*cYaw_angle - cRoll_angle*sYaw_angle;
float r13 =  cRoll_angle*stht*cYaw_angle + sRoll_angle*sYaw_angle;

float r21 =  ctht*sYaw_angle;
float r22 =  sRoll_angle*stht*sYaw_angle + cRoll_angle*cYaw_angle;
float r23 =  cRoll_angle*stht*sYaw_angle - sRoll_angle*cYaw_angle;

float r31 = -stht;
float r32 =  sRoll_angle*ctht;
float r33 =  cRoll_angle*ctht;

float x = r13*range;
float y = r23*range;
float z = r33*range;
#endif
