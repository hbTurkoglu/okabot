/*---------------------------------------------------------------------*/


//Kütüphaneler.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


/*---------------------------------------------------------------------*/


#define DEBUG_MODE false

// NRF24L01 tanımlamaları
//anten pinleri sck= 13 mosi = 11  miso = 12 digital
#define CE_PIN 9
#define CSN_PIN 10

//Joystick (analog pin) tanımlamaları.
#define VRX_PIN_GAS 0 
#define VRY_PIN_GAS 1
#define VRX_PIN_STR 2   
#define VRY_PIN_STR 3


/*---------------------------------------------------------------------*/


//Joystick değişkenleri.
int xValueGas = 0;
int yValueGas = 0;
int xValueStr = 0;
int yValueStr = 0;



//Objeler.
RF24 radio(CE_PIN,CSN_PIN);

const byte address[6] = "00031";

byte data[4];


/*---------------------------------------------------------------------*/


// prototipler

void startRadio()
void sendData()



// Başlancıç fonksiyonları
void startRadio()
{
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setPALevel(1);
  radio.stopListening();
}


/*---------------------------------------------------------------------*/


//Görev fonksiyonları

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


/*---------------------------------------------------------------------*/


//Döngü fonksiyonları


void sendData()
{
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);

    //Verileri sıkıştır ve değişkenlere ata.
    data[0] = map(xValueGas, 0, 1023, 0, 255);
    data[1] = map(yValueGas, 0, 1023, 0, 255);
    data[2] = map(xValueStr, 0, 1023, 0, 255);
    data[3] = map(yValueStr, 0, 1023, 0, 255);

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


/*---------------------------------------------------------------------*/
