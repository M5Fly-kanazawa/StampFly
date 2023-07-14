// #include <Ps3Controller.h>
#include "rc_sbus.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "driver/uart.h"
#if CONFIG_IDF_TARGET_ESP32
    #include "esp32/rom/uart.h"
#elif CONFIG_IDF_TARGET_ESP32S2
    #include "esp32s2/rom/uart.h"
#endif

//esp_now_peer_info_t slave;


int player = 0;
int battery = 0;
float rctime=0.0;
volatile uint8_t Connect_flag = 0;

//Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
const uint8_t addr[6] = {0x4C, 0x75, 0x25, 0xAD, 0xB6, 0x6C};

esp_now_peer_info_t peerInfo;

//RC Stick Init
volatile float Stick[16];

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle UART interrupt.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

const int uart_buffer_size = (BUF_SIZE);
static QueueHandle_t uart_queue;

// Both definition are same and valid
//static uart_isr_handle_t *handle_console;
static intr_handle_t handle_console;



// Receive buffer to collect incoming data
uint8_t rxbuf[256];
// Register to collect data length
uint16_t urxlen;

volatile uint8_t Rc_data[1024];
volatile uint16_t Rc_length = 999;

//グローバル変数の宣言
volatile uint16_t Chdata[18];

// 受信コールバック  
//static void IRAM_ATTR uart_intr_handle(void *arg)
//static void uart_intr_handle(void *arg)
void sbus_dacode(void)
{
  uint16_t rx_fifo_len, status;
  uint16_t i;

  // Read data from UART.
  const uart_port_t uart_num = UART_NUM_1;

  uart_get_buffered_data_len(uart_num, (size_t*)&Rc_length);
  Rc_length = uart_read_bytes(uart_num, (void*)Rc_data, Rc_length, 100);

  //Rc_length = 777;
  //Serial.printf("Receive! %d\n", length);

  // after reading bytes from buffer clear UART interrupt status
  //uart_clear_intr_status(UART_NUM_1, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

  //ローカル変数の宣言
  uint16_t chars_rxed = 0;
  uint16_t data_num=0;
  uint8_t sbus_data[25];
  uint8_t ch;

  while (Rc_length) {
        ch = Rc_data[data_num];
        data_num++;
        Rc_length--;


        if(ch==0x0f&&chars_rxed==0){
            sbus_data[chars_rxed]=ch;
            //printf("%02X ",ch);
            chars_rxed++;
        }
        else if(chars_rxed>0){
            sbus_data[chars_rxed]=ch;
            //printf("%02X ",ch);
            chars_rxed++;            
        }
        
        switch(chars_rxed){
            case 3:
                Chdata[0]=(sbus_data[1]|(sbus_data[2]<<8)&0x07ff);
                //printf("%04d ",Chdata[0]);
                break;
            case 4:
                Chdata[1]=(sbus_data[3]<<5|sbus_data[2]>>3)&0x07ff;
                //printf("%04d ",Chdata[1]);
                break;
            case 6:
                Chdata[2]=(sbus_data[3]>>6|sbus_data[4]<<2|sbus_data[5]<<10)&0x07ff;
                //printf("%04d ",Chdata[2]);
                break;
            case 7:
                Chdata[3]=(sbus_data[6]<<7|sbus_data[5]>>1)&0x07ff;
                //printf("%04d ",Chdata[3]);
                break;
            case 8:
                Chdata[4]=(sbus_data[7]<<4|sbus_data[6]>>4)&0x07ff;
                //printf("%04d ",Chdata[4]);
                break;
            case 10:
                Chdata[5]=(sbus_data[7]>>7|sbus_data[8]<<1|sbus_data[9]<<9)&0x07ff;
                //printf("%04d ",Chdata[5]);
                break;
            case 11:
                Chdata[6]  = ((sbus_data[9]>>2|sbus_data[10]<<6) & 0x07FF);
                //printf("%04d ",Chdata[6]);
                break;
            case 12:
                Chdata[7]  = ((sbus_data[10]>>5|sbus_data[11]<<3) & 0x07FF);
                //printf("%04d ",Chdata[7]);
                break;
            case 14:
                Chdata[8]  = ((sbus_data[12]|sbus_data[13]<< 8) & 0x07FF);
                //printf("%04d ",Chdata[8]);
                break;
            case 15:
                Chdata[9]  = ((sbus_data[13]>>3|sbus_data[14]<<5) & 0x07FF);
                //printf("%04d ",Chdata[9]);
                break;
            case 16:
                Chdata[10] = ((sbus_data[14]>>6|sbus_data[15]<<2|sbus_data[16]<<10) & 0x07FF);
                //printf("%04d ",Chdata[10]);
                break;
            case 17:
                Chdata[11] = ((sbus_data[16]>>1|sbus_data[17]<<7) & 0x07FF);
                //printf("%04d ",Chdata[11]);
                break;
            case 19:
                Chdata[12] = ((sbus_data[17]>>4|sbus_data[18]<<4) & 0x07FF);
                //printf("%04d ",Chdata[12]);
                break;
            case 21:
                Chdata[13] = ((sbus_data[18]>>7|sbus_data[19]<<1|sbus_data[20]<<9) & 0x07FF);
                //printf("%04d ",Chdata[13]);
                break;
            case 22:
                Chdata[14] = ((sbus_data[20]>>2|sbus_data[21]<<6) & 0x07FF);
                //printf("%04d ",Chdata[14]);
                break;
            case 23:
                Chdata[15] = ((sbus_data[21]>>5|sbus_data[22]<<3) & 0x07FF);
                //printf("%04d ",Chdata[15]);
                break;
            case 24:
                Chdata[16] = sbus_data[23];
                //printf("%04x ",Chdata[16]);
                break;
        }

        if(chars_rxed==25){
            Chdata[17]=sbus_data[24];
            //printf("%04d ",Chdata[17]);
            //printf("\n");
            chars_rxed=0;
        }
    }

    Stick[RUDDER]   = (float)(2*Chdata[RUDDER]  - (RUDDER_MAX   + RUDDER_MIN))    /(float)(RUDDER_MAX   - RUDDER_MIN);
    Stick[THROTTLE] = (float)(  Chdata[THROTTLE]- THROTTLE_MIN)                   /(float)(THROTTLE_MAX - THROTTLE_MIN);
    Stick[AILERON]  = (float)(2*Chdata[AILERON] - (AILERON_MAX  + AILERON_MIN))   /(float)(AILERON_MAX  - AILERON_MIN);
    Stick[ELEVATOR] = (float)(2*Chdata[ELEVATOR]- (ELEVATOR_MAX + ELEVATOR_MIN))  /(float)(ELEVATOR_MAX - ELEVATOR_MIN);
    if (Chdata[MODESWITCH]>0.5*(CH5MAX+CH5MIN)) Stick[BUTTON] = 1.0;
    else Stick[BUTTON] = 0.0f;
    USBSerial.printf("%4d %4d %4d %4d %4d\n\r", Chdata[THROTTLE], Chdata[AILERON], Chdata[ELEVATOR], Chdata[RUDDER], Chdata[MODESWITCH] );
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) 
{

#if 0
  Stick[RUDDER]=0;
  Stick[THROTTLE]=0;
  Stick[AILERON]  = 0;
  Stick[ELEVATOR]  = 0;
  Stick[BUTTON] = 0;
  Stick[BUTTON_A] = 0;
  Stick[CONTROLMODE] = 0;  
  Stick[LOG] = 0.0;

  //Normalize
  Stick[RUDDER] /= -RUDDER_MAX;
  Stick[THROTTLE] /= THROTTLE_MAX;
  Stick[AILERON] /= (0.5*3.14159);
  Stick[ELEVATOR] /= (0.5*3.14159);
  if(Stick[THROTTLE]<0.0) Stick[THROTTLE]=0.0;
#endif
}

void rc_init(void)
{
  telemetry_init();

  //Initialize Stick list 
  for (uint8_t i = 0;i<16;i++)Stick[i]=0.0;

  /* Configure parameters of an UART driver,
	* communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = 100000,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_EVEN,
		.stop_bits = UART_STOP_BITS_2,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
  // Configure UART parameters
  uart_param_config(UART_NUM_1, &uart_config);
    //Set pin
  uart_set_pin(UART_NUM_1, 15, 13, -1, -1);
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
  // Install UART driver using an event queue here
  uart_driver_install(UART_NUM_1, uart_buffer_size, 0, 10, &uart_queue, 0);
  //uart_isr_free(UART_NUM_1);
  //uart_isr_register(UART_NUM_1, uart_intr_handle, NULL, 1, &handle_console);
  //uart_enable_rx_intr(UART_NUM_1);

}

void telemetry_init(void)
{

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("MAC ADDRESS: %s\r\n", (WiFi.macAddress()).c_str());

  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  //ペアリング
  
  //telemetry 相手とペアリング
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 5;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        Serial.println("Failed to add peer");
        return;
  }

  // ESP-NOWコールバック登録
  //esp_now_register_recv_cb(OnDataRecv);
  //Serial.println("ESP-NOW Ready.");
  //Serial.println("Wait Contoroller ready....");
  //while(Connect_flag==0);
  //Serial.println("Contoroller ready !");
  esp_wifi_set_channel(5, WIFI_SECOND_CHAN_NONE);

}

void telemetry_send(uint8_t* data, uint16_t datalen)
{
  //uint8_t data[1];
  //data[0]=0xff;
  esp_err_t result = esp_now_send(peerInfo.peer_addr, data, datalen);
  //Serial.printf("%d\r\n", sizeof(data));
}

void rc_end(void)
{
}

bool rc_isconnected(void)
{
    return 1;
}

void rc_demo()
{
}












