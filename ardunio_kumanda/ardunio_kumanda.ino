#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN 9
#define CSN_PIN 10
#define VRX_PIN_GAS 0
#define VRY_PIN_GAS 1
#define VRX_PIN_STR 2 
#define VRY_PIN_STR 3

#define DEADZONE 15

int xValueGas = 0;
int yValueGas = 0;
int xValueStr = 0;
int yValueStr = 0;

int prev_xValueGas = 0;
int prev_yValueGas = 0;
int prev_xValueStr = 0;
int prev_yValueStr = 0;

RF24 radio(CE_PIN,CSN_PIN);
const byte address[6] = "00001";


void setup() 
{
  Serial.begin(9600);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setPALevel(3);
  radio.stopListening();
}

void loop()
 {
    xValueGas = analogRead(VRX_PIN_GAS);
    yValueGas = analogRead(VRY_PIN_GAS);
    xValueStr = analogRead(VRX_PIN_STR);
    yValueStr = analogRead(VRY_PIN_STR);
 
    int message[4] = {};
    message[0] = xValueGas;
    message[1] = yValueGas;
    message[2] = xValueStr;
    message[3] = yValueStr;

    if (abs(xValueGas - prev_xValueGas) > DEADZONE || abs(yValueGas -prev_yValueGas) > DEADZONE ||
       abs(xValueStr - prev_xValueStr)  > DEADZONE|| abs(yValueGas - prev_yValueGas) > DEADZONE) 
      {
      Serial.print("xg =");
      Serial.println(message[0]);
      Serial.print("yg=");
      Serial.println(message[1]);
      Serial.print("xs =");
      Serial.println(message[2]);
      Serial.print("ys=");
      Serial.println(message[3]);
      radio.write(&message,sizeof(message));
      }

    prev_xValueGas = xValueGas;
    prev_yValueGas = yValueGas;
    prev_xValueStr = xValueStr;
    prev_yValueStr = yValueStr;

    delay(1000);
  

}
