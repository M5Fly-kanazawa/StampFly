#include "sensor.hpp"

MPU6886 IMU;
Madgwick Drone_ahrs;
Alt_kalman EstimatedAltitude;

// Set I2C address to 0x40 (A0 pin -> GND)
INA3221 ina3221(INA3221_ADDR40_GND);
Filter acc_filter;
Filter voltage_filter;
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

//Sensor data
volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
volatile float Phi=0.0f, Theta=0.0f, Psi=0.0f;
volatile float Altitude = 0.0f;
volatile float Altitude2 = 0.0f;

volatile float Voltage;
float Acc_norm=0.0f;
//quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;
uint8_t OverG_flag = 0;
volatile float Pbias=0.0f, Qbias=0.0f, Rbias=0.0f;
volatile uint8_t Power_flag = 0;

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
      USBSerial.print (i, HEX);     // PCF8574 7 bit address
      USBSerial.println (")");
      count++;
    }
  }
  USBSerial.print ("Found ");      
  USBSerial.print (count, DEC);        // numbers of devices
  USBSerial.println (" device(s).");
  return count;
}

uint8_t mpu6886_byte_read(uint8_t reg_addr)
{
  uint8_t data;
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.endTransmission();
  Wire1.requestFrom(MPU6886_ADDRESS, 1);
  data = Wire1.read();
  return data;
}

void mpu6886_byte_write(uint8_t reg_addr, uint8_t data)
{
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.write(data);
  Wire1.endTransmission();
}

void imu_init(void)
{
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

  IMU.Init();
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き

  MPU6886 imu;

  Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);

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

}




void test_rangefinder(void)
{
}

void sensor_init()
{
  if(init_i2c()==0)
  {
    Serial.printf("No I2C device!\r\n");
    Serial.printf("Can not boot AtomFly2.\r\n");
    while(1);
  }

  imu_init();
  tof_init();
  //test_rangefinder();
  Drone_ahrs.begin(400.0);
  ina3221.begin(&Wire1);
  ina3221.reset();  
  //voltage_filter.set_parameter(0.005, 0.0025);
  //Acceleration filter
  acc_filter.set_parameter(0.005, 0.0025);

}

void tof_init(void)
{
    tof.begin(0x29, false, &Wire1);
    tof.setDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING);
    tof.setMeasurementTimingBudgetMicroSeconds(33000);
    tof.startMeasurement();
    while(tof.isRangeComplete()==0);
    Serial.printf("Distance=%d\n", tof.readRangeResult());
}

void start_mesure_distance(void)
{
  tof.startMeasurement();
}

uint8_t is_finish_ranging(void)
{
  return tof.isRangeComplete();
}

uint16_t get_distance(void)
{
  return  tof.readRangeResult();
}

void ahrs_reset(void)
{
  Drone_ahrs.reset();
}

void sensor_read(void)
{
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
  float filterd_v;
  static float dp, dq, dr; 
  static uint16_t dcnt=0u;
  uint16_t dist;

  //Get Altitude (20Hz)
  if (dcnt==0)
  {
    start_mesure_distance();
    dcnt = 1u;
  }
  else if (dcnt>14)
  {
    dcnt++;
    if(is_finish_ranging())
    {
      dist = get_distance();
      Altitude = dist;
      dcnt=0u;
    }
  }
  else dcnt++;

  //以下では航空工学の座標軸の取り方に従って
  //X軸：前後（前が正）左肩上がりが回転の正
  //Y軸：右左（右が正）頭上げが回転の正
  //Z軸：下上（下が正）右回りが回転の正
  //となる様に軸の変換を施しています
  //MPU6886の座標軸の撮り方は
  //X軸：右左（右が正）頭上げが回転の正
  //Y軸：前後（前が正）左肩上がりが回転の正
  //Z軸：上下（上が正）左回りが回転の正

  IMU.getAccelData(&ax, &ay, &az);
  IMU.getGyroData(&gx, &gy, &gz);

  Ax = ay;
  Ay = ax;
  Az =-az;
  Wp = gy*(float)DEG_TO_RAD;
  Wq = gx*(float)DEG_TO_RAD;
  Wr =-gz*(float)DEG_TO_RAD;

  if(Mode > AVERAGE_MODE)
  {
    Drone_ahrs.updateIMU((Wq-Qbias)*(float)RAD_TO_DEG, (Wp-Pbias)*(float)RAD_TO_DEG, -(Wr-Rbias)*(float)RAD_TO_DEG, Ay, Ax, -Az);
    Phi =   Drone_ahrs.getPitch()*(float)DEG_TO_RAD;
    Theta = Drone_ahrs.getRoll()*(float)DEG_TO_RAD;
    Psi =  -Drone_ahrs.getYaw()*(float)DEG_TO_RAD;
  }


  #if 1
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  Acc_norm = acc_filter.update(acc_norm);
  if (Acc_norm>9.0) 
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;
  }
  #endif
  
  #if 1
  Voltage = ina3221.getVoltage(INA3221_CH2);
  filterd_v = voltage_filter.update(Voltage);

  if(Power_flag != POWER_FLG_MAX){
    if (filterd_v < POWER_LIMIT) Power_flag ++;
    else Power_flag = 0;
    if ( Power_flag > POWER_FLG_MAX) Power_flag = POWER_FLG_MAX;
  }
  #endif

  float phi = Phi;
  float tht = Theta;
  //float psi = Psi;
  //float sphi = sin(phi);
  float cphi = cos(phi);
 // float stht = sin(tht);
  float ctht = cos(tht);
  //float spsi = sin(psi);
  //float spsi = cos(psi);

  float r33 =  cphi*ctht;
  Altitude2 = r33*Altitude;
  //EstimatedAltitude.update(Altitude2, r33*Az)

}

#if 0

float range = 1.0f;

float phi = 0.0f;
float tht = 0.0f;
float psi = 0.0f;
float sphi = sin(phi);
float cphi = cos(phi);
float stht = sin(tht);
float ctht = cos(tht);
float spsi = sin(psi);
float spsi = cos(psi);

float r11 =  ctht*cpsi;
float r12 =  sphi*stht*cpsi - cphi*spsi;
float r13 =  cphi*stht*cpsi + sphi*spsi;

float r21 =  ctht*spsi;
float r22 =  sphi*stht*spsi + cphi*cpsi;
float r23 =  cphi*stht*spsi - sphi*cpsi;

float r31 = -stht;
float r32 =  sphi*ctht;
float r33 =  cphi*ctht;

float x = r13*range;
float y = r23*range;
float z = r33*range;
#endif


