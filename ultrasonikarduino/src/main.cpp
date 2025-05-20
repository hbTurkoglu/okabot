#include <Arduino.h>
#include <Ticker.h>

#define TRIG 2

#define SOUND_SPEED 0.034

#define SENSOR_COUNT 8

#define DEBUG_MODE false

#define SCAN_FREQ 500

byte echoes[SENSOR_COUNT] = {10, 3, 4, 5, 6, 7, 8, 9};
byte distances[SENSOR_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};



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


  #if DEBUG_MODE
  printConsoleTask.start();
  #endif
}

void loop() 
{
  scanSensors(SCAN_FREQ);
  Serial.write(distances, sizeof(distances));


  #if DEBUG_MODE
  printConsoleTask.update();
  #endif
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