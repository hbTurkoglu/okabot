#include <Arduino.h>
#include <Ticker.h>

#define TRIG 2

#define SOUND_SPEED 0.034

#define SENSOR_COUNT 6

byte echoes[SENSOR_COUNT] = {3, 4, 5, 6, 7, 9};
byte distances[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0};



void printConsole();
void readDistance(int index);
void scanSensors(int freq);

Ticker printConsoleTask(printConsole, 1000);

void setup() 
{
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);

  for(int i = 0; i < SENSOR_COUNT; i++)
  {
    pinMode(echoes[i], INPUT);
  }

  //printConsoleTask.start();
}

void loop() 
{
  scanSensors(100);
  Serial.write(distances, sizeof(distances));
  //printConsoleTask.update();
}

void scanSensors(int freq)
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    readDistance(i);
    delay(freq/SENSOR_COUNT);
  }
}

void readDistance(int index)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(echoes[index], HIGH);
  byte distance = constrain(((duration * SOUND_SPEED) / 2), 0, 40);
  //int distance = ((duration * SOUND_SPEED) / 2);
  distances[index] = distance;
}


void printConsole()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(i+1);
    Serial.print(". Sensor Veri: ");
    Serial.print(distances[i]);
    Serial.println("cm");
  }
  Serial.println("---------------------------");
}