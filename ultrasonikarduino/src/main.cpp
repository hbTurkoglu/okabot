#include <Arduino.h>

#define TRIG 2

#define SOUND_SPEED 0.034

#define SENSOR_COUNT 6

byte echoes[SENSOR_COUNT] = {3, 4, 5, 6, 7, 8};
byte distances[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0};



byte readDistance(int echo);
void scanSensors(int freq);

void setup() 
{
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);

  for(int i = 0; i < SENSOR_COUNT; i++)
  {
    pinMode(echoes[i], OUTPUT);
  }
}

void loop() 
{
  scanSensors(1000);
  Serial.write(distances, sizeof(distances));
}

void scanSensors(int freq)
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    distances[i] = readDistance(echoes[i]);
    delay(freq/SENSOR_COUNT);
  }
}

byte readDistance(int echo)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(echo, HIGH);
  byte distance = constrain(((duration * SOUND_SPEED) / 2), 0, 40);
  return distance;
}



