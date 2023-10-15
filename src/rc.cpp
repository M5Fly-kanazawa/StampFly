#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;

//Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
//ATOM Lite (C): 4C:75:25:AE:27:FC
//4C:75:25:AD:8B:20
const uint8_t addr[6] = {0x4C, 0x75, 0x25, 0xAD, 0x8B, 0x20};

esp_now_peer_info_t peerInfo;

//RC
volatile float Stick[16];

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) 
{
  Connect_flag=0;

  uint8_t* d_int;
  int16_t d_short;
  float d_float;

#ifdef MINIJOYC
  d_int = (uint8_t*)&d_short;
  d_int[0]=recv_data[0];
  d_int[1]=recv_data[1];
  Stick[RUDDER]=(float)d_short;

  d_int[0]=recv_data[2];
  d_int[1]=recv_data[3];
  Stick[THROTTLE]=(float)d_short;

  d_int = (uint8_t*)&d_float;
  d_int[0] = recv_data[4];
  d_int[1] = recv_data[5];
  d_int[2] = recv_data[6];
  d_int[3] = recv_data[7];
  Stick[AILERON]  = d_float;

  d_int[0] = recv_data[8];
  d_int[1] = recv_data[9];
  d_int[2] = recv_data[10];
  d_int[3] = recv_data[11];
  Stick[ELEVATOR]  = d_float;

  Stick[BUTTON_ARM] = recv_data[12];
  Stick[BUTTON_FLIP] = recv_data[13];
  Stick[CONTROLMODE] = recv_data[14];
  
  Stick[LOG] = 0.0;

  //Normalize
  Stick[RUDDER] /= RUDDER_MAX;
  Stick[THROTTLE] /= THROTTLE_MAX;
  Stick[AILERON] /= (0.5*3.14159);
  Stick[ELEVATOR] /= (0.5*3.14159);
  if(Stick[THROTTLE]<0.0) Stick[THROTTLE]=0.0;

#else
  d_int = (uint8_t*)&d_float;
  
  d_int[0] = recv_data[0];
  d_int[1] = recv_data[1];
  d_int[2] = recv_data[2];
  d_int[3] = recv_data[3];
  Stick[RUDDER]=d_float;

  d_int[0] = recv_data[4];
  d_int[1] = recv_data[5];
  d_int[2] = recv_data[6];
  d_int[3] = recv_data[7];
  Stick[THROTTLE]=d_float;

  d_int[0] = recv_data[8];
  d_int[1] = recv_data[9];
  d_int[2] = recv_data[10];
  d_int[3] = recv_data[11];
  Stick[AILERON]  = d_float;

  d_int[0] = recv_data[12];
  d_int[1] = recv_data[13];
  d_int[2] = recv_data[14];
  d_int[3] = recv_data[15];
  Stick[ELEVATOR]  = d_float;

  Stick[BUTTON_ARM] = recv_data[16];
  Stick[BUTTON_FLIP] = recv_data[17];
  Stick[CONTROLMODE] = recv_data[18];
  
  Stick[LOG] = 0.0;

  //Normalize
  //Stick[RUDDER] /= -RUDDER_MAX_JOYC;
  //Stick[THROTTLE] /= THROTTLE_MAX_JOYC;
  //Stick[AILERON] /= (0.5*3.14159);
  //Stick[ELEVATOR] /= (0.5*3.14159);
  if(Stick[THROTTLE]<0.0) Stick[THROTTLE]=0.0;
#endif
  
#if 0
  USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n\r", 
                                            Stick[THROTTLE],
                                            Stick[AILERON],
                                            Stick[ELEVATOR],
                                            Stick[RUDDER],
                                            Stick[BUTTON],
                                            Stick[BUTTON_A],
                                            Stick[CONTROLMODE],
                                            Stick[LOG]);
#endif
}

// 送信コールバック
uint8_t esp_now_send_status;
void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  esp_now_send_status = status;
}

void rc_init(void)
{
  //Initialize Stick list 
  for (uint8_t i = 0;i<16;i++)Stick[i]=0.0;

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  USBSerial.printf("MAC ADDRESS: %s\r\n", (WiFi.macAddress()).c_str());

  if (esp_now_init() == ESP_OK) {
    USBSerial.println("ESPNow Init Success");
  } else {
    USBSerial.println("ESPNow Init Failed");
    ESP.restart();
  }

  //ペアリング
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 8;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        USBSerial.println("Failed to add peer");
        return;
  }

  // ESP-NOWコールバック登録
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(on_esp_now_sent);
  USBSerial.println("ESP-NOW Ready.");
  USBSerial.println("Wait Contoroller ready....");
  //while(Connect_flag==0);
  USBSerial.println("Contoroller ready !");
  esp_wifi_set_channel(8, WIFI_SECOND_CHAN_NONE);
}

uint8_t telemetry_send(uint8_t* data, uint16_t datalen)
{
  static uint32_t cnt=0;
  static uint8_t error_flag = 0;
  static uint8_t state=0;

  esp_err_t result;

  if ((error_flag == 0)&&(state==0))
  {
    result = esp_now_send(peerInfo.peer_addr, data, datalen);
    cnt=0;
  }
  else cnt++;
  
  if (esp_now_send_status == 0)
  {
    error_flag = 0;
    //state = 0;
  }
  else
  {
    //送信でエラーが一度でも出た場合はそれ以降stateは１になる。
    error_flag = 1;
    state = 1;
  }
  if (cnt>100)
  {
    error_flag = 0;
    cnt = 0;
  }
  cnt++;
  //USBSerial.printf("%6d %d %d\r\n", cnt, error_flag, esp_now_send_status);

  return state;
}

void rc_end(void)
{
    // Ps3.end();
}

uint8_t rc_isconnected(void)
{
    bool status;
    Connect_flag++;
    if (Connect_flag<10)status = 1;
    else status = 0;
    //USBSerial.printf("%d \n\r", Connect_flag);

    return status;
}

void rc_demo()
{
}

