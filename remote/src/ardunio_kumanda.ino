

//Kütüphaneler.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG_MODE true

// NRF24L01 tanımlamaları
#define CE_PIN 9//anten pinleri sck= 13 mosi = 11  miso = 12 digital
#define CSN_PIN 10

//Joystick tanımlamaları.
#define VRX_PIN_GAS 32//joystick pinleri analog
#define VRY_PIN_GAS 33
#define VRX_PIN_STR 34  
#define VRY_PIN_STR 35


//Joystick değişkenleri.
int xValueGas = 0;
int yValueGas = 0;
int xValueStr = 0;
int yValueStr = 0;

uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xBD, 0x32, 0x0C};



//Objeler.
RF24 radio(CE_PIN,CSN_PIN);

const byte address[6] = "00031";

byte data[4];

typedef struct JoystickData{
  int received_xValueGas = 0, received_yValueGas = 0, received_xValueStr = 0, received_yValueStr = 0;
} JoystickData;

JoystickData joystickData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

    analogReadResolution(10);

  

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop()
{
  sendData();
}


void startRadio()
{
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setPALevel(1);
  radio.stopListening();
}




void sendData()
{
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);

    //Verileri sıkıştır ve değişkenlere ata.
    joystickData.received_xValueGas = map(xValueGas, 0, 1023, 0, 255) + 16;
    joystickData.received_yValueGas = map(yValueGas, 0, 1023, 0, 255) + 16;
    joystickData.received_xValueStr = map(xValueStr, 0, 1023, 0, 255) + 16;
    joystickData.received_yValueStr = map(yValueStr, 0, 1023, 0, 255) + 16;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystickData, sizeof(joystickData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    
    #if DEBUG_MODE
      Serial.print("xg = ");
      Serial.println(joystickData.received_xValueGas);
      Serial.print("yg = ");
      Serial.println(joystickData.received_yValueGas);
      Serial.print("xs = ");
      Serial.println(joystickData.received_xValueStr);
      Serial.print("ys = ");
      Serial.println(joystickData.received_yValueStr);
      Serial.println("\n");
    #endif
    
}
