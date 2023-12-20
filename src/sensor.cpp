#include "sensor.hpp"
#include "common.h"
#include "bmi2.h"

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
volatile uint16_t Offset_counter = 0;

volatile float Voltage;
float Acc_norm=0.0f;
//quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;
uint8_t OverG_flag = 0;
volatile uint8_t Under_voltage_flag = 0; 

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
#define INT_BOTTOM 6
#define XSHUT_BOTTOM 7
#define INT_FRONT 8
#define XSHUT_FRONT 9
#define USER_A 0

VL53LX_Dev_t tof_front;
VL53LX_Dev_t tof_bottom;

VL53LX_DEV ToF_front=&tof_front;
VL53LX_DEV ToF_bottom=&tof_bottom;


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

void IRAM_ATTR tof_int()
{
  ToF_bottom_data_ready_flag = 1;
}

void tof_range_get(VL53LX_DEV dev)
{
  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData=&MultiRangingData;

  VL53LX_GetMultiRangingData(dev, pMultiRangingData);
  uint8_t no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
  if(no_of_object_found!=0)
  {
    Range = MultiRangingData.RangeData[0].RangeMilliMeter;
    #if 0
    for(uint8_t j=0;j<no_of_object_found;j++){
          if(j!=0)USBSerial.printf("\n\r                     ");
          USBSerial.printf("%d %5d %2.2f %2.2f ",
                  MultiRangingData.RangeData[j].RangeStatus,
                  MultiRangingData.RangeData[j].RangeMilliMeter,
                  MultiRangingData.RangeData[j].SignalRateRtnMegaCps/65536.0,
                  MultiRangingData.RangeData[j].AmbientRateRtnMegaCps/65536.0);
    }
    #endif
        
  }
  VL53LX_ClearInterruptAndStartMeasurement(dev);
}

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

void imu_init(void)
{
  USBSerial.printf("Start IMU Initialize!\n\r");
  pinMode(46, OUTPUT);//CSを設定
  digitalWrite(46, 0);//CSをHIGH
  bmi270_dev_init();
  USBSerial.printf("SPI Initilize status:%d\n\r",spi_init());
  usleep(1000*10);
  uint8_t data=0;

  //BMI270 Init
  USBSerial.printf("#INIT Status:%d\n\r", bmi270_init(pBmi270));
  USBSerial.printf("#Chip ID DEV:%02X\n\r", Bmi270.chip_id);
  USBSerial.printf("#APP_STATUS:%02X\n\r", Bmi270.aps_status);
  
  USBSerial.printf("#INIT_STATUS Read:%d\n\r",bmi2_get_regs(0x21, &data, 1, pBmi270));  
  USBSerial.printf("#INIT_STATUS:%02X\n\r", data);
  //IMU Config
  USBSerial.printf("#Config Status:%d\n\r", set_accel_gyro_config(pBmi270));
  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
  USBSerial.printf("#Sensor enable Status:%d\n\r", bmi2_sensor_enable(sensor_list, 2, pBmi270));

  #if 0
  //BMI270使用の試行錯誤の跡
  uint8_t w_data[2]={3,5};
  USBSerial.printf("#Read status:%d\n\r",bmi2_get_regs(0x00, &data, 1, pBmi270));  
  USBSerial.printf("#Chicp ID:%02X\n\r", data);

  USBSerial.printf("#Read status:%d\n\r",bmi2_get_regs(0x7C, &data, 1, pBmi270));  
  USBSerial.printf("#ADR 0x7C:%02X=0x03\n\r", data);
  USBSerial.printf("#Read status:%d\n\r",bmi2_get_regs(0x7D, &data, 1, pBmi270));  
  USBSerial.printf("#ADR 0x7D:%02X=0x00\n\r", data);

  USBSerial.printf("#Write status:%d\n\r",bmi2_set_regs(0x7C, w_data, 2, pBmi270));  
  
  USBSerial.printf("#Read status:%d\n\r",bmi2_get_regs(0x7C, &data, 1, pBmi270));  
  USBSerial.printf("#ADR 0x7C:%02X=0x03\n\r", data);
  USBSerial.printf("#Read status:%d\n\r",bmi2_get_regs(0x7D, &data, 1, pBmi270));  
  USBSerial.printf("#ADR 0x7D:%02X=0x00\n\r", data);
  #endif
}

#define DPS20002RAD 34.90658504
#define DPS10002RAD 17.4532925199

void test_imu(void)
{
  u_long st, now, old, end;
  uint16_t count;
  uint8_t ret;
  st=micros();
  now = st;
  old = st;
  struct bmi2_sens_data imu_data;
  usleep(1000*5000);


  while(1)
  {
    old = now;
    now = micros();
    ret = bmi2_get_sensor_data(&imu_data, pBmi270);
    //USBSerial.printf("%d\n\r", ret);
    acc_x = lsb_to_mps2(imu_data.acc.x, 8.0, 16);
    acc_y = lsb_to_mps2(imu_data.acc.y, 8.0, 16);
    acc_z = lsb_to_mps2(imu_data.acc.z, 8.0, 16);
    gyro_x = lsb_to_rps(imu_data.gyr.x, DPS10002RAD, 16);
    gyro_y = lsb_to_rps(imu_data.gyr.y, DPS10002RAD, 16);
    gyro_z = lsb_to_rps(imu_data.gyr.z, DPS10002RAD, 16);
    #if 1
    USBSerial.printf("%8.4f %7.5f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %d\n\r", 
      (float)(now-st)*1.0e-6,
      (float)(now - old)*1.0e-6,
      acc_x,
      acc_y,
      acc_z,
      gyro_x,
      gyro_y,
      gyro_z,
      ret);
  #endif
  }
  
}

void termin(void)
{
  while(1)
  {
    if(ToF_bottom_data_ready_flag)
    {
      ToF_bottom_data_ready_flag = 0;
      tof_range_get(ToF_bottom);

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


void tof_init(void)
{
  uint8_t byteData;
  uint16_t wordData;

  ToF_bottom->comms_speed_khz = 400;
  ToF_bottom->i2c_slave_address = 0x29;

  ToF_front->comms_speed_khz = 400;
  ToF_front->i2c_slave_address = 0x29;


  //USBSerial.printf("#tof_i2c_init_status:%d\r\n",vl53lx_i2c_init());  

  //ToF Pin Initialize
  pinMode(XSHUT_BOTTOM, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(INT_BOTTOM, INPUT);
  pinMode(INT_FRONT, INPUT);
  pinMode(USER_A, INPUT_PULLUP);
  
  //ToF Disable
  digitalWrite(XSHUT_BOTTOM, LOW);
  digitalWrite(XSHUT_FRONT, LOW);

  //Front ToF I2C address to 0x54  
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  VL53LX_SetDeviceAddress(ToF_front, 0x54);
  ToF_front->i2c_slave_address = 0x2A;

  delay(100);
  digitalWrite(XSHUT_BOTTOM, HIGH);

  //Bttom ToF setting
  USBSerial.printf("#1 WaitDeviceBooted Status:%d\n\r",VL53LX_WaitDeviceBooted(ToF_bottom));
  USBSerial.printf("#1 DataInit Status:%d\n\r",VL53LX_DataInit(ToF_bottom));
  USBSerial.printf("#1 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_bottom, VL53LX_DISTANCEMODE_LONG));
  USBSerial.printf("#1 SetMeasurementTimingBuget Status:%d\n\r",VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_bottom, 20000));
  USBSerial.printf("#1 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_bottom, 0x010F, &byteData));
  USBSerial.printf("#1 VL53LX Model_ID: %02X\n\r", byteData);  
  USBSerial.printf("#1 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_bottom, 0x0110, &byteData));
  USBSerial.printf("#1 VL53LX Module_Type: %02X\n\r", byteData);
  USBSerial.printf("#1 RdWord Status:%d\n\r", VL53LX_RdWord(ToF_bottom, 0x010F, &wordData));
  USBSerial.printf("#1 VL53LX: %04X\n\r", wordData);
  
  //Front ToF Setting
  USBSerial.printf("#2 WaitDeviceBooted Status:%d\n\r",VL53LX_WaitDeviceBooted(ToF_front));
  USBSerial.printf("#2 DataInit Status:%d\n\r",VL53LX_DataInit(ToF_front));
  USBSerial.printf("#1 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_front, VL53LX_DISTANCEMODE_LONG));
  USBSerial.printf("#2 SetMeasurementTimingBuget Status:%d\n\r",VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_front, 20000));
  USBSerial.printf("#2 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_front, 0x010F, &byteData));
  USBSerial.printf("#2 VL53LX Model_ID: %02X\n\r", byteData);  
  USBSerial.printf("#2 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_front, 0x0110, &byteData));
  USBSerial.printf("#2 VL53LX Module_Type: %02X\n\r", byteData);
  USBSerial.printf("#2 RdWord Status:%d\n\r", VL53LX_RdWord(ToF_front, 0x010F, &wordData));
  USBSerial.printf("#2 VL53LX: %04X\n\r", wordData);

  attachInterrupt(INT_BOTTOM, &tof_int, FALLING);

  VL53LX_ClearInterruptAndStartMeasurement(ToF_bottom);
  delay(100);
  USBSerial.printf("#Start Measurement Status:%d\n\r", VL53LX_StartMeasurement(ToF_bottom));

}

void test_ranging(VL53LX_DEV dev)
{
  uint8_t status=0;
  uint8_t data_ready=0;
  int16_t range;
  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData=&MultiRangingData;

  if (status==0){
    status = VL53LX_ClearInterruptAndStartMeasurement(dev);
  }
  delay(100);
  USBSerial.printf("#Start Measurement Status:%d\n\r", VL53LX_StartMeasurement(dev));

  USBSerial.printf("#Count ObjNo Status Range Signal(Mcps) Ambient(Mcps)\n\r");

  u_long st, now, old, end;
  uint16_t count;
  st=micros();
  now = st;
  old = st;

  count = 0;
  USBSerial.printf("Start!\n\r");
  while(count<500)
  {
    //VL53LX_WaitMeasurementDataReady(dev);
    //if(digitalRead(INT_BOTTOM)==0)

    VL53LX_GetMeasurementDataReady(dev, &data_ready);
    if( data_ready==1 )
    {
      data_ready = 0;
      count++;      
      VL53LX_GetMultiRangingData(dev, pMultiRangingData);
      old = now;
      now=micros();
      uint8_t no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      USBSerial.printf("%7.4f %7.4f ", (float)(now-st)*1e-6, (float)(now - old)*1e-6);
      USBSerial.printf("%5d ", pMultiRangingData->StreamCount);
      USBSerial.printf("%1d ", no_of_object_found);
      for(uint8_t j=0;j<no_of_object_found;j++){
        if(j!=0)USBSerial.printf("\n\r                     ");
        USBSerial.printf("%d %5d %2.2f %2.2f ",
                MultiRangingData.RangeData[j].RangeStatus,
                MultiRangingData.RangeData[j].RangeMilliMeter,
                MultiRangingData.RangeData[j].SignalRateRtnMegaCps/65536.0,
                MultiRangingData.RangeData[j].AmbientRateRtnMegaCps/65536.0);        
      }
      
      VL53LX_ClearInterruptAndStartMeasurement(dev);
      end=micros();
      USBSerial.printf ("%8.6f", (float)(end-now)*1.0e-6);
      USBSerial.printf ("\n\r");

    }
  }
  USBSerial.printf("End!\n\r");

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
  beep_init();
 
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
  
  //test_ranging();

  //pipo();

  delay(500);
  //test_imu();
  //termin();
  //test_voltage();  


  //Acceleration filter
  //acc_filter.set_parameter(0.005, 0.0025);
  //while(1);
}

float sensor_read(void)
{
  struct bmi2_sens_data imu_data;
  float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
  float filterd_v;
  static float dp, dq, dr; 
  static uint16_t dcnt=0u;
  uint16_t dist;
  const uint8_t interval = 400/30+1;

  uint32_t st = micros();
  //以下では航空工学の座標軸の取り方に従って
  //X軸：前後（前が正）左肩上がりが回転の正
  //Y軸：右左（右が正）頭上げが回転の正
  //Z軸：下上（下が正）右回りが回転の正
  //となる様に軸の変換を施しています
  //MPU6886の座標軸の撮り方は
  //X軸：右左（右が正）頭上げが回転の正
  //Y軸：前後（前が正）左肩上がりが回転の正
  //Z軸：上下（上が正）左回りが回転の正

  bmi2_get_sensor_data(&imu_data, pBmi270);

  acc_x = lsb_to_mps2(imu_data.acc.x, 8.0, 16)/GRAVITY_EARTH;
  acc_y = lsb_to_mps2(imu_data.acc.y, 8.0, 16)/GRAVITY_EARTH;
  acc_z = lsb_to_mps2(imu_data.acc.z, 8.0, 16)/GRAVITY_EARTH;
  gyro_x = lsb_to_rps(imu_data.gyr.x, DPS20002RAD, 16);
  gyro_y = lsb_to_rps(imu_data.gyr.y, DPS20002RAD, 16);
  gyro_z = lsb_to_rps(imu_data.gyr.z, DPS20002RAD, 16);

  Accel_x_raw =  acc_y;
  Accel_y_raw =  acc_x;
  Accel_z_raw = -acc_z;
  Roll_rate_raw  =  gyro_y;
  Pitch_rate_raw =  gyro_x;
  Yaw_rate_raw   = -gyro_z;

  if(Mode > AVERAGE_MODE)
  {
    Drone_ahrs.updateIMU((Pitch_rate_raw-Pitch_rate_offset)*(float)RAD_TO_DEG, (Roll_rate_raw-Roll_rate_offset)*(float)RAD_TO_DEG, -(Yaw_rate_raw-Yaw_rate_offset)*(float)RAD_TO_DEG, Accel_y_raw, Accel_x_raw, -Accel_z_raw);
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
  Acc_norm = acc_filter.update(acc_norm);
  if (Acc_norm>3.8) 
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;
  }
  #endif

  //Battery voltage check 
  Voltage = ina3221.getVoltage(INA3221_CH2);
  filterd_v = voltage_filter.update(Voltage);

  if(Under_voltage_flag != UNDER_VOLTAGE_COUNT){
    if (filterd_v < POWER_LIMIT) Under_voltage_flag ++;
    else Under_voltage_flag = 0;
    if ( Under_voltage_flag > UNDER_VOLTAGE_COUNT) Under_voltage_flag = UNDER_VOLTAGE_COUNT;
  }
  uint32_t mt=micros();

  //Altitude
  #if 1
  //Get Altitude (30Hz)
  if (dcnt == 0) tof.startMeasurement();
  if (dcnt>interval)
  {
    if(is_finish_ranging())
    {
      dist = tof.readRangeResult();
      if(dist>2000)dist = (uint16_t)Altitude;
      Altitude = (float)dist;
      dcnt=0u;
      Alt_control_ok = 1;//距離データが得られたら制御をしても良いフラグを立てる
      EstimatedAltitude.update(Altitude/1000.0, -(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset) );
      Altitude2 = EstimatedAltitude.Altitude;
      Alt_velocity = EstimatedAltitude.Velocity;
    }
    else dcnt++;
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
