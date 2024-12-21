

//NOT: Kod şu anlık küçük test arabası için optimizedir.


//Kütüphaneler.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LCD_I2C.h>
#include <Ticker.h>

#define DEBUG_MODE true

// NRF24L01 tanımlamaları
#define CE_PIN 9//anten pinleri sck= 13 mosi = 11  miso = 12 digital
#define CSN_PIN 10

//Joystick tanımlamaları.
#define VRX_PIN_GAS 0//joystick pinleri analog
#define VRY_PIN_GAS 1
#define VRX_PIN_STR 2   
#define VRY_PIN_STR 3
#define DEADZONE 10


//Joystick değişkenleri.
byte xValueGas = 0;
byte yValueGas = 0;
byte xValueStr = 0;
byte yValueStr = 0;

byte prev_xValueGas = 0;
byte prev_yValueGas = 0;
byte prev_xValueStr = 0;
byte prev_yValueStr = 0;


//Ticker
int lcdUpdateFreq = 200;
void updateLcd();
Ticker lcdTask(updateLcd, lcdUpdateFreq);


//Objeler.
RF24 radio(CE_PIN,CSN_PIN);
LCD_I2C lcd(0x27, 16, 2);

const byte address[6] = "00001";

byte data[4];


void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

  lcd.begin();               
  lcd.backlight();
  lcdTask.start();

  startRadio();
}

void loop()
{
  sendData();
  lcdTask.update();
}


void startRadio()
{
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setPALevel(3);
  radio.stopListening();
}


void updateLcd()
{
  lcd.setCursor(0, 0);
  lcd.print ("xg= ");
  lcd.print(xValueGas);
  lcd.setCursor(0, 1); 
  lcd.print("yg= ");
  lcd.print(yValueGas);
  lcd.setCursor(9, 0);
  lcd.print ("xs= ");
  lcd.print(xValueStr);
  lcd.setCursor(9, 1); 
  lcd.print ("ys= ");
  lcd.print(yValueStr);
}


void sendData()
{
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);



    if (abs(xValueGas - prev_xValueGas) > DEADZONE || abs(yValueGas - prev_yValueGas) > DEADZONE ||
        abs(xValueStr - prev_xValueStr)  > DEADZONE|| abs(yValueGas - prev_yValueGas) > DEADZONE)
    {
      data[0] = xValueGas;
      data[1] = yValueGas;
      data[2] = xValueStr;
      data[3] = yValueStr;

      radio.write(&data, sizeof(data));

      
      #if DEBUG_MODE
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

    prev_xValueGas = xValueGas;
    prev_yValueGas = yValueGas;
    prev_xValueStr = xValueStr;
    prev_yValueStr = yValueStr;
}
