# 1 "C:\\Users\\YUSUFE~1\\AppData\\Local\\Temp\\tmpy6uaqb9o"
#include <Arduino.h>
# 1 "C:/Users/yusufemrebilir/Documents/GitHub/okabot/remote/src/ardunio_kumanda.ino"



#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LCD_I2C.h>
#include <Ticker.h>

#define DEBUG_MODE true


#define CE_PIN 9
#define CSN_PIN 10


#define VRX_PIN_GAS 0
#define VRY_PIN_GAS 1
#define VRX_PIN_STR 2
#define VRY_PIN_STR 3



int xValueGas = 0;
int yValueGas = 0;
int xValueStr = 0;
int yValueStr = 0;

byte prev_xValueGas = 0;
byte prev_yValueGas = 0;
byte prev_xValueStr = 0;
byte prev_yValueStr = 0;



int lcdUpdateFreq = 200;



RF24 radio(CE_PIN,CSN_PIN);
LCD_I2C lcd(0x27, 16, 2);

const byte address[6] = "00001";

byte data[4];
void setup();
void loop();
void startRadio();
void sendData();
#line 48 "C:/Users/yusufemrebilir/Documents/GitHub/okabot/remote/src/ardunio_kumanda.ino"
void setup()
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif


  startRadio();
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
  radio.setPALevel(3);
  radio.stopListening();
}




void sendData()
{
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);

    data[0] = map(xValueGas, 0, 1023, 0, 255);
    data[1] = map(yValueGas, 0, 1023, 0, 255);
    data[2] = map(xValueStr, 0, 1023, 0, 255);
    data[3] = map(yValueStr, 0, 1023, 0, 255);

    radio.write(&data, sizeof(data));


    #if false
      Serial.print("xg = ");
      Serial.println(data[0]);
      Serial.print("yg = ");
      Serial.println(data[1]);
      Serial.print("xs = ");
      Serial.println(data[2]);
      Serial.print("ys = ");
      Serial.println(data[3]);
      Serial.println("\n");
    #endif

}