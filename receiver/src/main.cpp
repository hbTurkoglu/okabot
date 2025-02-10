
/*---------------------------------------------------------------------*/

//Kütüphaneler.

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ticker.h>
#include <math.h>

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
#define SPEED_ADJUSTING_FREQ 10
#define ACCELERATION 2
#define SLOWDOWNZONE 30

#define OMNIDEADZONE 50

/*---------------------------------------------------------------------*/

//Görevler

void adjustInputs();
Ticker adjustInputsTask(adjustInputs, SPEED_ADJUSTING_FREQ, 0, MICROS); //Girişleri oku

void emergencyLockdown();
Ticker emergencyLockdownTask(emergencyLockdown, 1000);
bool dataReceived = false;
bool atLockdown = false;

/*---------------------------------------------------------------------*/

//pwm değerleri

const int pwmFreq = 15000;
const int pwmResolution = 8;


//pwm kanalları

const int pwmChannel_1R = 0;
const int pwmChannel_1L = 1;

const int pwmChannel_2R = 2;
const int pwmChannel_2L = 3;

const int pwmChannel_3R = 4;
const int pwmChannel_3L = 5;

const int pwmChannel_4R = 6;
const int pwmChannel_4L = 7;

byte xValueGas = 0, yValueGas = 0, xValueStr = 0, yValueStr = 0;

byte det_xValueGas = 0, det_yValueGas = 0, det_xValueStr = 0, det_yValueStr = 0;

byte joystickIdleValue = 125;

int R_value;
int L_value;
int power;

float teta;
float sin_value;
float cos_value;
float max_value;
float omni_x;
float omni_y;
float turn;
float left_front;
float right_front;
float left_rear;
float right_rear;


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
void omni_X();
void omni_Y();
void omni_Diagonal();
void omni_Turn();
void simpleOmniDrive();
void power_direction_calculation();
void omni_move();
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
  adjustInputsTask.start();
  emergencyLockdownTask.start();

}

void loop() 
{
  getValuesFromRadio();
  emergencyLockdownTask.update();
  adjustInputsTask.update();
  //sendPWM();
  /*
  if (abs(xValueGas - joystickIdleValue) > OMNIDEADZONE ||  //Dönüş yada ilerlemeye karar ver, dönüşe öncelik ver.
      abs(yValueGas - joystickIdleValue) > OMNIDEADZONE)
  {
    omni_Turn();
  }
  else
  {
    simpleOmniDrive();
  }*/
  omni_move();
}


/*---------------------------------------------------------------------*/


//Döngü fonksiyonları

void getValuesFromRadio()
{
  if (radio.available())
  {
    radio.read(&data, sizeof(data));  //Verileri değişkenlere çek.
    dataReceived = true; //Verinin geldiğini kaydet.

    //verileri değişkenlere ata
    xValueStr = data[0];
    yValueStr = data[1];
    xValueGas = data[2];
    yValueGas = data[3];

    #if false //DEBUG_MODE  //Seri monitöre yazdır.
      Serial.print("Gelen veri 1: ");
      Serial.println(data[0]);
      Serial.print("Gelen veri 2: ");
      Serial.println(data[1]);
      Serial.print("Gelen veri 3: ");
      Serial.println(data[2]);
      Serial.print("Gelen veri 4: ");
      Serial.println(data[3]);
      Serial.print("\n");
      delay(500);
    #endif
  }
}


void smoothInputs(byte& determinedValue, byte currentValue, byte Accel)
{
  if (abs(currentValue - determinedValue) > Accel)
  {
    if (currentValue > determinedValue)
    {
      determinedValue += Accel;
    } 
    else 
    {
      determinedValue -= Accel;
    }
  }
}

void adjustInputs()
{
  smoothInputs(det_xValueGas, xValueGas, ACCELERATION);
  smoothInputs(det_yValueGas, yValueGas, ACCELERATION);
  smoothInputs(det_xValueStr, xValueStr, ACCELERATION);
  smoothInputs(det_yValueStr, yValueStr, ACCELERATION);
}


void determinePwmValues(byte input, int channel_R, int channel_L, int middlePoint)
{
  if (input > middlePoint)
  {
    if (L_value != 0)
    {
      L_value = 0;
      ledcWrite(channel_L, L_value); // Sol kanalı kapat
    }

    if (input > middlePoint + SLOWDOWNZONE)
    {
      R_value = map(input, middlePoint + SLOWDOWNZONE, 255, 2*SLOWDOWNZONE, 255);
    }
    else
    {
      R_value = map((input - middlePoint) * (input - middlePoint), 0, SLOWDOWNZONE * SLOWDOWNZONE, 0, 2*SLOWDOWNZONE);
    }
    ledcWrite(channel_R, R_value);
  }
  else
  {
    if (R_value != 0)
    {
      R_value = 0;
      ledcWrite(channel_R, R_value); // Sağ kanalı kapat
    }

    if (input < middlePoint - SLOWDOWNZONE)
    {
      L_value = map(input, 0, middlePoint - SLOWDOWNZONE, 255, 2*SLOWDOWNZONE);
    }
    else
    {
      L_value = map((input - middlePoint) * (input - middlePoint), 0, SLOWDOWNZONE * SLOWDOWNZONE, 0, 2*SLOWDOWNZONE);
    }
    ledcWrite(channel_L, L_value);
  }
}

void sendPWM()
{
  determinePwmValues(det_xValueGas, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  determinePwmValues(det_yValueGas, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  determinePwmValues(det_xValueStr, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  determinePwmValues(det_yValueStr, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}


void emergencyLockdown()
{
  if (!dataReceived)
  {
    xValueGas = joystickIdleValue;
    yValueGas = joystickIdleValue;
    xValueStr = joystickIdleValue;
    yValueStr = joystickIdleValue;
  }
  dataReceived = false; //kontrol bool'unu sıfırla  
}

/*
void simpleOmniDrive()
{
  //en basit şekilde yöne karar ver ve ilgili fonksiyona yönlendir. Geçici, çok ilkel bir yöntem. Biraz kafa patlatmak lazım...
  if(abs(xValueGas - joystickIdleValue) > OMNIDEADZONE &&
      abs(yValueGas - joystickIdleValue) > OMNIDEADZONE)
  {
    omni_Diagonal();
  }
  if ((abs(xValueGas - joystickIdleValue) < OMNIDEADZONE) &&
      (abs(yValueGas - joystickIdleValue) > OMNIDEADZONE))
  {
    omni_Y();
  }
  if((abs(xValueGas - joystickIdleValue) > OMNIDEADZONE) &&
      (abs(yValueGas - joystickIdleValue) < OMNIDEADZONE))
  {
    omni_X();
  }
}

void omni_Y()
{
  determinePwmValues(det_yValueGas, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  determinePwmValues(det_yValueGas, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  determinePwmValues(det_yValueGas, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  determinePwmValues(det_yValueGas, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}

void omni_X()
{
  determinePwmValues(255-det_xValueGas, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  determinePwmValues(255-det_xValueGas, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  determinePwmValues(det_xValueGas, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  determinePwmValues(det_xValueGas, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}

void omni_Diagonal()
{
  if ((det_xValueGas > joystickIdleValue && det_yValueGas > joystickIdleValue) ||
      (det_xValueGas < joystickIdleValue && det_yValueGas < joystickIdleValue))
  {
    determinePwmValues(det_yValueGas, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
    determinePwmValues(det_yValueGas, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
    determinePwmValues(joystickIdleValue, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
    determinePwmValues(joystickIdleValue, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
  }
  else
  {
    determinePwmValues(joystickIdleValue, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
    determinePwmValues(joystickIdleValue, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
    determinePwmValues(det_yValueGas, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
    determinePwmValues(det_yValueGas, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
  }
}

void omni_Turn()
{
  determinePwmValues(255-det_yValueStr, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  determinePwmValues(det_yValueStr, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  determinePwmValues(det_yValueStr, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  determinePwmValues(255-det_yValueStr, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}*/
void power_direction_calculation()
{
  omni_x = det_xValueGas /255,0;
  omni_y = -det_yValueGas /255,0;
  turn = det_xValueStr /255,0;
  

  teta = atan2(omni_y,omni_x);
  power = hypot(omni_x,omni_y);
}



void omni_move()
{
  power_direction_calculation();
  sin_value = sin(teta-PI/4);
  cos_value = cos(teta-PI/4);
  max_value = max(abs(sin_value),abs(cos_value));

  left_front = power * cos_value/max_value + turn;
  right_front = power * sin_value/max_value - turn;
  left_rear = power * sin_value/max_value + turn;
  right_rear = power * cos_value/max_value - turn;

  if ((power + abs(turn)) > 1)
  {
    left_front /= power + turn; 
    right_front /= power + turn; 
    left_rear /= power + turn; 
    right_rear /= power + turn; 
  }
  left_front *= 255;
  right_front *= 255;
  left_rear *= 255;
  right_rear *=255;
  determinePwmValues(left_front, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  determinePwmValues(right_front, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  determinePwmValues(right_rear, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  determinePwmValues(left_rear, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}




