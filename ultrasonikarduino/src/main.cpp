#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ticker.h>

#define TRIG 2

#define SOUND_SPEED 0.034

#define SENSOR_COUNT 4

byte echoes[SENSOR_COUNT] = {3, 4, 5, 6};
byte distances[SENSOR_COUNT] = {12, 13, 14, 15};



int readDistance(int echo);
void scanSensors(int freq);

void setup() 
{
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(echoes[0], INPUT);
  pinMode(echoes[1], INPUT);
  pinMode(echoes[2], INPUT);
  pinMode(echoes[3], INPUT);
}

void loop() 
{
  scanSensors(1000);
  Serial.write(distances, sizeof(distances));
}

void scanSensors(int freq)
{
  distances[0] = readDistance(echoes[0]);
  delay(freq/SENSOR_COUNT);
  distances[1] = readDistance(echoes[1]);
  delay(freq/SENSOR_COUNT);
  distances[2] = readDistance(echoes[2]);
  delay(freq/SENSOR_COUNT);
  distances[3] = readDistance(echoes[3]);
  delay(freq/SENSOR_COUNT);
  
}

int readDistance(int echo)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(echo, HIGH);
  int distance = constrain(((duration * SOUND_SPEED) / 2), 0, 40);
  return distance;
}



