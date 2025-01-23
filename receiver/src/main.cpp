
/*---------------------------------------------------------------------*/

//Kütüphaneler.

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ticker.h>

/*---------------------------------------------------------------------*/

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
#define SPEED_ADJUSTING_FREQ 30
#define FORWARD_ACCELERATION 50
#define BACKWARD_ACCELERATION 10

/*---------------------------------------------------------------------*/

//Görevler

void adjustInputs();
Ticker driveMotorsTask(adjustInputs, SPEED_ADJUSTING_FREQ); //Girişleri oku

void emergencyLockdown();
Ticker emergencyLockdownTask(emergencyLockdown, 1000);
bool dataReceived = false;
bool atLockdown = false;

/*---------------------------------------------------------------------*/

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

byte xValueGas = 0, yValueGas = 0, xValueStr = 0, yValueStr = 0;

byte determined_xValueGas = 0, determined_yValueGas = 0, determined_xValueStr = 0, determined_yValueStr = 0;

byte JIV_xValueGas = 160; //Joystick Idle Value
byte JIV_yValueGas = 160; //Joystick Idle Value
byte JIV_xValueStr = 160; //Joystick Idle Value
byte JIV_yValueStr = 160; //Joystick Idle Value

/*---------------------------------------------------------------------*/

//Objeler.
RF24 radio(CE_PIN, CSN_PIN);


const byte address[6] = "00001"; // Haberleşme adresi
byte data[4];

/*---------------------------------------------------------------------*/

//prototipler
void startRadio();
void setPins();
void getValuesFromRadio();
void sendPWM();

/*---------------------------------------------------------------------*/


//Başlancıç fonksiyonları

void startRadio()
{
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_250KBPS);  //iletim hızı, 250kbps en hızlı.
  radio.setPALevel(3);  //güç seviyesi, 1-min, 3-max.
  radio.startListening(); // Alıcı moduna geçiş.
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


/*---------------------------------------------------------------------*/



//Görev fonksiyonları

void setup() 
{
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

  setPins();
  startRadio();
  driveMotorsTask.start();
  //emergencyLockdownTask.start();    geçici olarak devre dışı bırakıldı.
}

void loop() 
{
  getValuesFromRadio();
  //emergencyLockdownTask.update();   geçici olarak devre dışı bırakıldı.
  driveMotorsTask.update();
}


/*---------------------------------------------------------------------*/


//Döngü fonksiyonları

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


void smoothInputs(byte& determinedValue, byte currentValue, byte forwardAccel, byte backwardAccel)
{
  if (abs(currentValue - determinedValue) > backwardAccel) 
  {
    if (currentValue > determinedValue) 
    {
      determinedValue += forwardAccel;
    } 
    else 
    {
      determinedValue -= backwardAccel;
    }
  }
}

void determineDirectionAndSpeed(byte input, int channel_R, int channel_L, int middlePoint)
{
  if (input > middlePoint)
  {
    ledcWrite(channel_L, 0); // Sol kanalı kapat
    byte pwmValue = map(input, middlePoint, 255, 0, 255);
    ledcWrite(channel_R, map(pow(pwmValue, 2), pow(0, 2), pow(255, 2), 0, 255));
  }
  else
  {
    ledcWrite(channel_R, 0); // Sağ kanalı kapat
    byte mirroredInput = 2 * middlePoint - input;
    byte pwmValue = map(mirroredInput, middlePoint, 255, 0, 255);
    ledcWrite(channel_L, map(pow(pwmValue, 2), pow(0, 2), pow(255, 2), 0, 255));
  }
}


void adjustInputs()
{
  smoothInputs(determined_xValueGas, xValueGas, FORWARD_ACCELERATION, BACKWARD_ACCELERATION);
  smoothInputs(determined_yValueGas, yValueGas, FORWARD_ACCELERATION, BACKWARD_ACCELERATION);
  smoothInputs(determined_xValueStr, xValueStr, FORWARD_ACCELERATION, BACKWARD_ACCELERATION);
  smoothInputs(determined_yValueStr, yValueStr, FORWARD_ACCELERATION, BACKWARD_ACCELERATION);
}

void sendPWM()
{
  determineDirectionAndSpeed(determined_xValueGas, pwmChannel_1R, pwmChannel_1L, JIV_xValueGas);
  determineDirectionAndSpeed(determined_yValueGas, pwmChannel_2R, pwmChannel_2L, JIV_yValueGas);
  determineDirectionAndSpeed(determined_xValueStr, pwmChannel_3R, pwmChannel_3L, JIV_xValueStr);
  determineDirectionAndSpeed(determined_yValueStr, pwmChannel_4R, pwmChannel_4L, JIV_yValueStr);
}



void driveMotors()
{
  adjustInputs();
  sendPWM();
}




void lockMotor(int channel_R, int channel_L)
{
  determineDirectionAndSpeed(0, channel_R, channel_L, JIV_xValueGas);
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

