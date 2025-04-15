/*---------------------------------------------------------------------*/


//Kütüphaneler.
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>


#define DEBUG_MODE true


//Joystick tanımlamaları.
#define VRX_PIN_GAS 32//joystick pinleri analog
#define VRY_PIN_GAS 33
#define VRX_PIN_STR 34  
#define VRY_PIN_STR 35



/*---------------------------------------------------------------------*/


//Joystick değişkenleri.
int xValueGas = 0;
int yValueGas = 0;
int xValueStr = 0;
int yValueStr = 0;

uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xBD, 0x32, 0x0C};


typedef struct JoystickData{
  int sent_xValueGas = 0, sent_yValueGas = 0, sent_xValueStr = 0, sent_yValueStr = 0;
} JoystickData;

JoystickData joystickData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/*---------------------------------------------------------------------*/


// prototipler

void sendData();



// Başlancıç fonksiyonları


/*---------------------------------------------------------------------*/


//Görev fonksiyonları

void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

    analogReadResolution(10);

  

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    while (true)
    {
      Serial.println("Error initializing ESP-NOW");
      digitalWrite(2, 0);
      delay(500);
      digitalWrite(2, 1);
      delay(500);
    }
  }

  esp_wifi_set_max_tx_power(78);
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



/*---------------------------------------------------------------------*/


//Döngü fonksiyonları


void sendData()
{
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);

    //Verileri sıkıştır ve değişkenlere ata.
    joystickData.sent_xValueGas = map(xValueGas, 0, 1023, 0, 1023) + 57;
    joystickData.sent_yValueGas = map(yValueGas, 0, 1023, 0, 1023) + 57;
    joystickData.sent_xValueStr = map(xValueStr, 0, 1023, 0, 1023) + 57;
    joystickData.sent_yValueStr = map(yValueStr, 0, 1023, 0, 1023) + 57;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystickData, sizeof(joystickData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    
    #if DEBUG_MODE
      Serial.print("xg = ");
      Serial.println(joystickData.sent_xValueGas);
      Serial.print("yg = ");
      Serial.println(joystickData.sent_yValueGas);
      Serial.print("xs = ");
      Serial.println(joystickData.sent_xValueStr);
      Serial.print("ys = ");
      Serial.println(joystickData.sent_yValueStr);
      Serial.println("\n");
    #endif
    
}


/*---------------------------------------------------------------------*/
