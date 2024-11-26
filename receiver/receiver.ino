//NOT: Kod şu anlık küçük test arabası için optimizedir.



//Kütüphaneler.
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define DEBUG_MODE true

// NRF24L01 tanımlamaları
#define CE_PIN 9
#define CSN_PIN 10

//Diğer tanımlamalar.
#define STEERING_PIN 3
#define MOTORS_ENABLE tbd
#define MOTORS_PWM tbd

#define ENGINE_SIM true


//Objeler.
RF24 radio(CE_PIN, CSN_PIN);
Servo steeringServo;

const byte address[6] = "00001"; // Haberleşme adresi
byte data[4];

void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

  steeringServo.attach(STEERING_PIN);
  startRadio();
}


void loop() 
{
  getValuesFromRadio();

  simulateEngine();
}


void startRadio()
{
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_250KBPS);  //iletim hızı.
  radio.setPALevel(3);  //güç seviyesi, 1-min, 3-max.
  radio.startListening(); // Alıcı moduna geçiş.
}


void simulateEngine()
{
  if (!ENGINE_SIM) {return;}

}


void getValuesFromRadio()
{
  if (radio.available()) 
  {
    radio.read(&data, sizeof(data));  //Verileri değişkenlere çek.

    #if DEBUG_MODE  //Seri monitöre yazdır.
      Serial.print("Gelen veri 1: ");
      Serial.println(data[0]);
      Serial.print("Gelen veri 2: ");
      Serial.println(data[1]);
      Serial.print("Gelen veri 3: ");
      Serial.println(data[2]);
      Serial.print("Gelen veri 4: ");
      Serial.println(data[3]);
      Serial.print("\n");
    #endif
  }
}