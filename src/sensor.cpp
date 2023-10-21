#include "sensor.hpp"

Madgwick Drone_ahrs;
Alt_kalman EstimatedAltitude;

INA3221 ina3221(INA3221_ADDR40_GND);// Set I2C address to 0x40 (A0 pin -> GND)
Filter acc_filter;
Filter voltage_filter;
//VL53L3C tof;

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
volatile uint16_t Offset_counter = 0;

volatile float Voltage;
float Acc_norm=0.0f;
//quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;
uint8_t OverG_flag = 0;
volatile uint8_t Under_voltage_flag = 0; 

uint8_t init_i2c()
{
  Wire1.begin(SDA_PIN, SCL_PIN,400000UL);
  USBSerial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (short i = 0; i < 256; i++)
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

void imu_init(void)
{
  IMU.begin();
 #if 0
  //Cutoff frequency
  //filter_config Gyro Accel
  //0 250    218.1 log140　Bad
  //1 176    218.1 log141　Bad
  //2 92     99.0  log142 Bad これはヨーガカクカクする log256
  //3 41     44.8  log143 log188　Good! log257
  //4 20     21.2
  //5 10     10.2
  //6 5      5.1
  //7 3281   420.0
  uint8_t data;
  const uint8_t filter_config = 2;//(今の所2はノイズが多くてダメ、log188は3)

  //Mdgwick filter 実験
  // filter_config=0において実施
  //beta =0 次第に角度増大（角速度の積分のみに相当する）
  //beta=0.5

  
  
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き

  //MPU6886 imu;

  //Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);

 //F_CHOICE_B
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  USBSerial.printf("GYRO_CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_GYRO_CONFIG, data & 0b11111100);
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  USBSerial.printf("Update GYRO_CONFIG %d\r\n", data);

  //Gyro
  //DLPG_CFG
  data = mpu6886_byte_read(MPU6886_CONFIG);
  USBSerial.printf("CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_CONFIG, (data&0b11111100)|filter_config);
  data = mpu6886_byte_read(MPU6886_CONFIG);
  USBSerial.printf("Update CONFIG %d\r\n", data);

  //Accel
  //ACCEL_FCHOCE_B & A_DLPF_CFG
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  USBSerial.printf("ACCEL_CONFIG2 %d\r\n", data);
  mpu6886_byte_write(MPU6886_ACCEL_CONFIG2, (data & 0b11110111) | filter_config);
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  USBSerial.printf("Update ACCEL_CONFIG2 %d\r\n", data);
#endif

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

void sensor_init()
{
  if(init_i2c()==0)
  {
    USBSerial.printf("No I2C device!\r\n");
    USBSerial.printf("Can not boot AtomFly2.\r\n");
    while(1);
  }

  imu_init();
  tof_init();
  //test_rangefinder();
  //Drone_ahrs.begin(400.0);
  ina3221.begin(&Wire1);
  ina3221.reset();  
  //voltage_filter.set_parameter(0.005, 0.0025);
  //Acceleration filter
  acc_filter.set_parameter(0.005, 0.0025);

}

void tof_init(void)
{
  u_long st=micros();




}


void ahrs_reset(void)
{
  Drone_ahrs.reset();
}

float sensor_read(void)
{
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

  //IMU.getAccelData(&ax, &ay, &az);
  //IMU.getGyroData(&gx, &gy, &gz);

  Accel_x_raw = ay;
  Accel_y_raw = ax;
  Accel_z_raw =-az;
  Roll_rate_raw  =  gy*(float)DEG_TO_RAD;
  Pitch_rate_raw =  gx*(float)DEG_TO_RAD;
  Yaw_rate_raw   = -gz*(float)DEG_TO_RAD;

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
  if (Acc_norm>9.0) 
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
  
  #if 0
  //Get Altitude (30Hz)
  if (dcnt>interval)
  {
    if(tof.dataIsReady())
    {
      dist = get_distance();
      Altitude = (float)dist;
      dcnt=0u;
    }
    EstimatedAltitude.update(Altitude/1000.0, -(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset) );
    Altitude2 = EstimatedAltitude.Altitude;
    Alt_velocity = EstimatedAltitude.Velocity;
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


