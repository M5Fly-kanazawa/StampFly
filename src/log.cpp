#include "log.hpp"

#if 0
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
  //19 Alt Velocity
  data2log(senddata, Alt_velocity, index);
  index = index + 4;
  //20 Z_dot_ref
  data2log(senddata, Z_dot_ref, index);
  index = index + 4;
  //20 FrontRight_motor_duty
  //data2log(senddata, FrontRight_motor_duty, index);
  //index = index + 4;
  //21 FrontLeft_motor_duty
  data2log(senddata, FrontLeft_motor_duty, index);
  index = index + 4;
  //22 RearRight_motor_duty
  data2log(senddata, RearRight_motor_duty, index);
  index = index + 4;
  //23 RearLeft_motor_duty
  //data2log(senddata, RearLeft_motor_duty, index);
  //23 Alt_ref
  data2log(senddata, Alt_ref, index);
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

#endif
