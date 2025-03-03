
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

#define MOTOR_3_L_PWM 32
#define MOTOR_3_R_PWM 15

#define MOTOR_4_L_PWM 25
#define MOTOR_4_R_PWM 33


//parametreler

#define MAX_POWER 80
#define SLOWDOWNZONE 40 //MAX_POWER 'dan küçük olmak zorunda. Yoksa... öngörülemeyen sonuçlar ortaya çıkabilir.

#define SPEED_ADJUSTING_FREQ 2
#define ACCELERATION 1

#define LOCKDOWN_TIME 500


/*---------------------------------------------------------------------*/

//Görevler

void adjustInputs();
Ticker adjustInputsTask(adjustInputs, SPEED_ADJUSTING_FREQ, 0, MILLIS); //Girişleri oku

void emergencyLockdown();
Ticker emergencyLockdownTask(emergencyLockdown, LOCKDOWN_TIME, 0, MILLIS); //Acil durum kapatma
bool dataReceived = false;

void printConsole();
Ticker printConsoleTask(printConsole, 500);

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

int xValueGas = 0, yValueGas = 0, xValueStr = 0, yValueStr = 0;

int det_xValueGas = 0, det_yValueGas = 0, det_xValueStr = 0, det_yValueStr = 0;

int joystickIdleValue = 0;

int R_value;
int L_value;

int omniX;
int omniY;

/*---------------------------------------------------------------------*/

//Objeler.
RF24 radio(CE_PIN, CSN_PIN);


const byte address[6] = "00031"; // Haberleşme adresi
byte data[4];

/*---------------------------------------------------------------------*/

//prototipler
void startRadio();
void setPins();
void getRadio();
void omniDrive();
void omniTurn();

/*---------------------------------------------------------------------*/


//Başlancıç fonksiyonları

void startRadio()
{
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_250KBPS);  //iletim hızı, 250kbps en hızlı.
  radio.setPALevel(1);  //güç seviyesi, 1-min, 3-max.
  radio.startListening(); // Alıcı moduna geçiş.
}

void setPins()
{
  pinMode(4, OUTPUT);
  digitalWrite(4, 0);

  pinMode(12, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(15, OUTPUT);


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
    printConsoleTask.start();
  #endif

  setPins();
  startRadio();
  adjustInputsTask.start();

}

void loop() 
{
  getRadio();

  if (!dataReceived)
  {
    if (emergencyLockdownTask.state() == STOPPED) {emergencyLockdownTask.start();}
    emergencyLockdownTask.update();
  } 
  else {if (emergencyLockdownTask.state() == RUNNING) {emergencyLockdownTask.stop();}}

  adjustInputsTask.update();
  printConsoleTask.update();
  
  if (abs(det_yValueStr) > 10)
  {
    omniTurn();
  }
  else
  {
    omniDrive();
  }
}


/*---------------------------------------------------------------------*/


//Döngü fonksiyonları

void getRadio()
{
  if (radio.available())
  {
    dataReceived = true; //Verinin geldiğini kaydet.
    radio.read(&data, sizeof(data));  //Verileri değişkenlere çek.
    digitalWrite(4, 1);

    //verileri değişkenlere ata ve sıkıştırmayı aç.
    xValueStr = map(data[0], 0, 255, -255, 255);
    yValueStr = map(data[1], 0, 255, -255, 255);
    xValueGas = map(data[2], 0, 255, -255, 255);
    yValueGas = map(data[3], 0, 255, -255, 255);
  }
  else
  {dataReceived = false;} 
  //Veri gelmediyse, veri alındı mı değişkenini false yap.
  //Bu değişken, veri alınmadığında acil durum kapatma işlemi yapar.
  //Kumanda sürekli veri göndermediğinden dolayı runtime'da bu değer sürekli değer değiştirir.
  //Ancak acil durum fonksiyonu sadece belirli bir süre bu değer false olduğunda çalışır.
  //Bu sayede kumanda uzun süre veri göndermeyi keserse, araç acil durumda kapanır.
}


void printConsole()
{
  Serial.println("-----------------------------");

  Serial.print("det_xValueGas: ");
  Serial.println(det_xValueGas);
  Serial.print("det_xValueStr: ");
  Serial.println(det_xValueStr);
  Serial.print("det_yValueGas: ");
  Serial.println(det_yValueGas);
  Serial.print("det_yValueStr: ");
  Serial.println(det_yValueStr);

  Serial.print("\n\n");

  Serial.println("omniX: ");
  Serial.println(omniX);
  Serial.println("omniY: ");
  Serial.println(omniY);

  Serial.println("-----------------------------");
}


void linearlyRefineInputs(int& determinedValue, int currentValue, int Accel)
{
  if (abs(currentValue - determinedValue) > 0)
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
  linearlyRefineInputs(det_xValueGas, xValueGas, ACCELERATION);
  linearlyRefineInputs(det_yValueGas, yValueGas, ACCELERATION);
  linearlyRefineInputs(det_xValueStr, xValueStr, ACCELERATION);
  linearlyRefineInputs(det_yValueStr, yValueStr, ACCELERATION);
}


void outputPwmValues(int input, int channel_R, int channel_L, int middlePoint)
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
      R_value = map(input, middlePoint + SLOWDOWNZONE, 255, SLOWDOWNZONE, MAX_POWER);
    }
    else
    {
      R_value = map((input - middlePoint) * (input - middlePoint), 0, SLOWDOWNZONE * SLOWDOWNZONE, 0, SLOWDOWNZONE);
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
      L_value = map(input, -255, middlePoint - SLOWDOWNZONE, MAX_POWER, SLOWDOWNZONE);
    }
    else
    {
      L_value = map((input - middlePoint) * (input - middlePoint), 0, -SLOWDOWNZONE * SLOWDOWNZONE, 0, SLOWDOWNZONE);
    }
    ledcWrite(channel_L, L_value);
  }
}


void emergencyLockdown()
{
  xValueGas = joystickIdleValue;
  yValueGas = joystickIdleValue;
  xValueStr = joystickIdleValue;
  yValueStr = joystickIdleValue;
  digitalWrite(4, 0);
  Serial.println("\n-----------------------------");
  Serial.println("Emergency lockdown activated.");
  Serial.println("-----------------------------");
}

void omniDrive()
{
  int power = sqrt(det_xValueGas*det_xValueGas + det_yValueGas*det_yValueGas);
  int angle = (atan2(det_yValueGas, det_xValueGas) * 180 / PI) - 45;

  if (power > 255) {power = 255;}

  omniX = map((power * cos(angle * PI / 180)), -180, 180, -255, 255);
  omniY = map((-power * sin(angle * PI / 180)), -180, 180, -255, 255);

  outputPwmValues(omniX, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  outputPwmValues(omniY, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  outputPwmValues(omniX, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  outputPwmValues(omniY, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}


void omniTurn()
{
  outputPwmValues(det_yValueStr, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  outputPwmValues(-det_yValueStr, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  outputPwmValues(-det_yValueStr, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  outputPwmValues(det_yValueStr, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}




