/*---------------------------------------------------------------------*/


//Kütüphaneler.
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Ticker.h>

#define DEBUG_MODE true


//Joystick tanımlamaları.
#define VRX_PIN_GAS 32//joystick pinleri analog
#define VRY_PIN_GAS 33
#define VRX_PIN_STR 34  
#define VRY_PIN_STR 35


#define AD_BUTTON 23
#define PP_BUTTON 22

#define AD_LED 4
#define PP_LED 13

void checkButtonCB();
Ticker checkButtonTicker(checkButtonCB, 100);

void printConsole();
Ticker printConsoleTiker(printConsole, 1000);

void sendData();
Ticker sendDataTicker(sendData, 50);


bool assistedDrivingBool = false;
bool assistedDrivingBoolReleased = false;
bool parallelParkingBool = false;
bool parallelParkingBoolReleased = false;


/*---------------------------------------------------------------------*/


//Joystick değişkenleri.
int xValueGas = 0;
int yValueGas = 0;
int xValueStr = 0;
int yValueStr = 0;

uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xBD, 0x32, 0x0C};


typedef struct JoystickData{
  int sent_xValueGas = 0, sent_yValueGas = 0, sent_xValueStr = 0, sent_yValueStr = 0;
  bool sent_parallelParking = false;
  bool sent_assistedDriving = false;
} JoystickData;

JoystickData joystickData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/*---------------------------------------------------------------------*/


// prototipler

void sendData();



// Başlancıç fonksiyonları

void initESPNow() 
{
  // Init ESP-NOW
  WiFi.mode(WIFI_STA);
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


void setPins()
{
  analogReadResolution(10);
  pinMode(VRX_PIN_GAS, INPUT);
  pinMode(VRY_PIN_GAS, INPUT);
  pinMode(VRX_PIN_STR, INPUT);
  pinMode(VRY_PIN_STR, INPUT);

  pinMode(AD_BUTTON, INPUT);
  pinMode(PP_BUTTON, INPUT);

  pinMode(PP_LED, OUTPUT);
  pinMode(AD_LED, OUTPUT);
}

/*---------------------------------------------------------------------*/


//Görev fonksiyonları

void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

  setPins();
  initESPNow();

  checkButtonTicker.start();
  printConsoleTiker.start();
  sendDataTicker.start();
}

void loop()
{
  sendDataTicker.update();
  checkButtonTicker.update();
  printConsoleTiker.update();
}



/*---------------------------------------------------------------------*/


//Döngü fonksiyonları


void checkButtonCB()
{
  if (digitalRead(AD_BUTTON) == 0 && !assistedDrivingBoolReleased)
  {
    assistedDrivingBoolReleased = true;
  }

  if (digitalRead(AD_BUTTON) == 1 && !assistedDrivingBool && assistedDrivingBoolReleased)
  {
    assistedDrivingBoolReleased = false;
    assistedDrivingBool = true;
    digitalWrite(AD_LED, assistedDrivingBool);
  }
  else if (digitalRead(AD_BUTTON) == 1 && assistedDrivingBool && assistedDrivingBoolReleased)
  {
    assistedDrivingBoolReleased = false;
    assistedDrivingBool = false;
    digitalWrite(AD_LED, assistedDrivingBool);
  }



  if (digitalRead(PP_BUTTON) == 0 && !parallelParkingBoolReleased)
  {
    parallelParkingBoolReleased = true;
  }

  if (digitalRead(PP_BUTTON) == 1 && !parallelParkingBool && parallelParkingBoolReleased)
  {
    parallelParkingBoolReleased = false;
    parallelParkingBool = true;
    digitalWrite(PP_LED, parallelParkingBool);
  }
  else if (digitalRead(PP_BUTTON) == 1 && parallelParkingBool && parallelParkingBoolReleased)
  {
    parallelParkingBoolReleased = false;
    parallelParkingBool = false;
    digitalWrite(PP_LED, parallelParkingBool);
  }

}

void sendData()
{
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);

    //Verileri sıkıştır ve değişkenlere ata.
    joystickData.sent_xValueGas = xValueGas + 52;
    joystickData.sent_yValueGas = yValueGas + 52;
    joystickData.sent_xValueStr = xValueStr + 52;
    joystickData.sent_yValueStr = yValueStr + 52;

    joystickData.sent_assistedDriving = assistedDrivingBool;
    joystickData.sent_parallelParking = parallelParkingBool;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystickData, sizeof(joystickData));
    
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    }
    else {
      //Serial.println("Error sending the data");
    }

    


    
}


void printConsole()
{
  #if DEBUG_MODE
    Serial.print("xg = ");
    Serial.println(joystickData.sent_xValueGas);
    Serial.print("yg = ");
    Serial.println(joystickData.sent_yValueGas);
    Serial.print("xs = ");
    Serial.println(joystickData.sent_xValueStr);
    Serial.print("ys = ");
    Serial.println(joystickData.sent_yValueStr);
    Serial.print("AD = ");
    Serial.println(joystickData.sent_assistedDriving);
    Serial.print("PP = ");
    Serial.println(joystickData.sent_parallelParking);
    Serial.println("\n");
  #endif
}


/*---------------------------------------------------------------------*/
