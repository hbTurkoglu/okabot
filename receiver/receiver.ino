#include <Servo.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// NRF24L01 tanımlamaları
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
Servo servo;
const byte address[6] = "00001"; // Haberleşme adresi


bool onOff = false;

void setup() {
  servo.write(0);
  Serial.begin(9600);
  pinMode(3, OUTPUT);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(3);
  radio.startListening(); // Alıcı moduna geçiş
}

void loop() 
{
  if (radio.available()) 
  {
    int data[4] = {};
    radio.read(&data, sizeof(data));
    analogWrite(3, map(data[0], 0, 1024, 0, 255));
    Serial.print("Gelen veri 1: ");
    Serial.println(data[0]);
    Serial.print("Gelen veri 2: ");
    Serial.println(data[1]);
    Serial.print("Gelen veri 3: ");
    Serial.println(data[2]);
    Serial.print("Gelen veri 4: ");
    Serial.println(data[3]);
    Serial.print("\n");
  }
}