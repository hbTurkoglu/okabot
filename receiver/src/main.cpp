
//NOT: Kod şu anlık küçük test arabası için optimizedir.



//Kütüphaneler.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ticker.h>


#define DEBUG_MODE true


// NRF24L01 tanımlamaları
#define CE_PIN 2
#define CSN_PIN 13


//Motor pin tanımlamaları.
#define MOTOR_1_R_PWM 12
#define MOTOR_1_L_PWM 14

#define MOTOR_2_R_PWM 27
#define MOTOR_2_L_PWM 26

#define MOTOR_3_R_PWM 25
#define MOTOR_3_L_PWM 33

#define MOTOR_4_R_PWM 32
#define MOTOR_4_L_PWM 15


//parametreler
#define ADJUSTING_FREQ 30
#define ACCELERATION 1


//Görevler
void adjustInputs();
Ticker inputAdjustTask(adjustInputs, ADJUSTING_FREQ); //Girişleri oku

void emergencyLockdown();
Ticker emergencyLockdownTask(emergencyLockdown, 1000);
bool dataReceived = false;
bool atLockdown = false;


//pwm değerleri
const int pwmFreq = 15000;
const int pwmResolution = 8;

const int pwmChannel_1R = 0;
const int pwmChannel_1L = 1;

const int pwmChannel_2R = 2;
const int pwmChannel_2L = 3;

const int pwmChannel_3R = 4;
const int pwmChannel_3L = 5;

const int pwmChannel_4R = 6;
const int pwmChannel_4L = 7;

//Gelen veriler
byte xValueGas = 0;
byte yValueGas = 0;
byte xValueStr = 0;
byte yValueStr = 0;

byte determined_xValueGas = 0;
byte determined_yValueGas = 0;
byte determined_xValueStr = 0;
byte determined_yValueStr = 0;


byte JIV_xValueGas = 160; //Joystick Idle Value
byte JIV_yValueGas = 160; //Joystick Idle Value
byte JIV_xValueStr = 160; //Joystick Idle Value
byte JIV_yValueStr = 160; //Joystick Idle Value

//Objeler.
RF24 radio(CE_PIN, CSN_PIN);


const byte address[6] = "00001"; // Haberleşme adresi
byte data[4];


//prototipler
void startRadio();
void setPins();
void getValuesFromRadio();
void driveMotors();
void calibrateJoysticks();


void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

  setPins();
  startRadio();
  calibrateJoysticks();
  inputAdjustTask.start();
  emergencyLockdownTask.start();
}


void loop() 
{
  getValuesFromRadio();
  emergencyLockdownTask.update();
  inputAdjustTask.update();
  if (!atLockdown) {driveMotors();}
}


void startRadio()
{
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_250KBPS);  //iletim hızı, 250kbps en hızlı.
  radio.setPALevel(3);  //güç seviyesi, 1-min, 3-max.
  radio.startListening(); // Alıcı moduna geçiş.
}


void calibrateJoysticks()
{
  for (int i = 0; i<5; i++) //5 kere dene
  {
    if (radio.available())
    {
      radio.read(&data, sizeof(data));
      JIV_xValueGas = data[0];
      JIV_yValueGas = data[1];
      JIV_xValueStr = data[2];
      JIV_yValueStr = data[3];
      break;
    }
    delay(100);
  }
}


void setPins()
{
  ledcSetup(pwmChannel_1R, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_1L, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_2R, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_2L, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_3R, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_3L, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_4R, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_4L, pwmFreq, pwmResolution);

  ledcAttachPin(MOTOR_1_R_PWM, pwmChannel_1R);
  ledcAttachPin(MOTOR_1_L_PWM, pwmChannel_1L);
  ledcAttachPin(MOTOR_2_R_PWM, pwmChannel_2R);
  ledcAttachPin(MOTOR_2_L_PWM, pwmChannel_2L);
  ledcAttachPin(MOTOR_3_R_PWM, pwmChannel_3R);
  ledcAttachPin(MOTOR_3_L_PWM, pwmChannel_3L);
  ledcAttachPin(MOTOR_4_R_PWM, pwmChannel_4R);
  ledcAttachPin(MOTOR_4_L_PWM, pwmChannel_4L);
}


void getValuesFromRadio()
{
  if (radio.available()) 
  {
    radio.read(&data, sizeof(data));  //Verileri değişkenlere çek.
    dataReceived = true; //Veri geldiğini kaydet.
    atLockdown = false; //Kilidi kapat.

    //verileri değişkenlere ata
    xValueGas = data[0];
    yValueGas = data[1];
    xValueStr = data[2];
    yValueStr = data[3];

    #if DEBUG_MODE  //Seri monitöre yazdır.
      Serial.print("Gelen veri x: ");
      Serial.println(data[0]);
      Serial.print("Gelen veri x: ");
      Serial.println(data[1]);
      Serial.print("Gelen veri 3: ");
      Serial.println(data[2]);
      Serial.print("Gelen veri 4: ");
      Serial.println(data[3]);
      Serial.print("\n");
    #endif
  }
}


void smoothlyAdjust(byte& determinedValue, byte currentValue, byte acceleration)
{
  if (abs(currentValue - determinedValue) > acceleration) 
  {
    if (currentValue > determinedValue) 
    {
      determinedValue += acceleration;
    } 
    else 
    {
      determinedValue -= acceleration;
    }
  }
}


void adjustInputs()
{
  smoothlyAdjust(determined_xValueGas, xValueGas, ACCELERATION);
  smoothlyAdjust(determined_yValueGas, yValueGas, ACCELERATION);
  smoothlyAdjust(determined_xValueStr, xValueStr, ACCELERATION);
  smoothlyAdjust(determined_yValueStr, yValueStr, ACCELERATION);
}


void determineDirection(byte input, int channel_R, int channel_L, int middlePoint)
{
  if (input > middlePoint)
  {
    ledcWrite(channel_L, 0);
    ledcWrite(channel_R, map(input, middlePoint, 256, 0, 256));
  }
  else
  {
    ledcWrite(channel_R, 0);
    ledcWrite(channel_L, map(input, 0, middlePoint, 0, 256));
  }
}


void driveMotors()
{
  determineDirection(determined_xValueGas, pwmChannel_1R, pwmChannel_1L, JIV_xValueGas);
  determineDirection(determined_yValueGas, pwmChannel_2R, pwmChannel_2L, JIV_yValueGas);
  determineDirection(determined_xValueStr, pwmChannel_3R, pwmChannel_3L, JIV_xValueStr);
  determineDirection(determined_yValueStr, pwmChannel_4R, pwmChannel_4L, JIV_yValueStr);
}


void lockMotor(int channel_R, int channel_L)
{
  determineDirection(0, channel_R, channel_L, JIV_xValueGas);
}

void emergencyLockdown()
{
  if (!dataReceived)
  {
    atLockdown = true;
    lockMotor(pwmChannel_1R, pwmChannel_1L);
    lockMotor(pwmChannel_2R, pwmChannel_2L);
    lockMotor(pwmChannel_3R, pwmChannel_3L);
    lockMotor(pwmChannel_4R, pwmChannel_4L);
  }
  dataReceived = false; //kontrol bool'unu sıfırla  
}

