
/*---------------------------------------------------------------------*/

// Kütüphaneler.

#include <Ticker.h>
#include <esp_now.h>
#include <WiFi.h>

/*---------------------------------------------------------------------*/

#define DEBUG_MODE true

// NRF24L01 tanımlamaları

#define CE_PIN 2
#define CSN_PIN 13

// Motor pin tanımlamaları.

#define MOTOR_1_R_PWM 12
#define MOTOR_1_L_PWM 14

#define MOTOR_2_R_PWM 27
#define MOTOR_2_L_PWM 26

#define MOTOR_3_L_PWM 32
#define MOTOR_3_R_PWM 15

#define MOTOR_4_L_PWM 25
#define MOTOR_4_R_PWM 33

#define ENABLE_PIN 21

#define CHARGE_PIN 34

// parametreler

#define MAX_POWER 80
#define SLOWDOWNZONE 40 // MAX_POWER 'dan küçük olmak zorunda. Yoksa... öngörülemeyen sonuçlar ortaya çıkabilir. 6 sürücü bundan yandı sefaya söylemeyin.

#define SPEED_ADJUSTING_FREQ 1
#define ACCELERATION 1

#define LOCKDOWN_TIME 500

/*---------------------------------------------------------------------*/

// Görevler

void adjustInputs();
Ticker adjustInputsTask(adjustInputs, SPEED_ADJUSTING_FREQ, 0, MILLIS); // Girişleri oku

void emergencyLockdown();
Ticker emergencyLockdownTask(emergencyLockdown, LOCKDOWN_TIME, 0, MILLIS); // Acil durum kapatma
bool dataReceived = false;

void printConsole();
Ticker printConsoleTask(printConsole, 500);

void chargeCheck();
Ticker chargeCheckTask(chargeCheck, 1000, 0, MILLIS); // Şarj kontrolü

/*---------------------------------------------------------------------*/

// pwm değerleri

const int pwmFreq = 15000;
const int pwmResolution = 8;

// pwm kanalları

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

int chargeValue;
int chargeInfo;

float power;
float angle;
float turn;

float motorOffsets[4] = {1, 1, 1, 1};

/*---------------------------------------------------------------------*/

// Objeler.

const byte address[6] = "00031";                                   // Haberleşme adresi
uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xBC, 0x15, 0xF4}; // Kumanda mac adresi
byte data[4];

typedef struct
{
  int received_xValueGas = 0, received_yValueGas = 0, received_xValueStr = 0, received_yValueStr = 0;
} JoystickData;

JoystickData joystickData;

esp_now_peer_info_t peerInfo;

/*---------------------------------------------------------------------*/

// prototipler
void setPins();
void omniDrive();
void omniTurn();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void initESPNow();
void lockdownCheck();

/*---------------------------------------------------------------------*/

// Başlancıç fonksiyonları

void setPins()
{
  pinMode(4, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(4, 0);
  digitalWrite(ENABLE_PIN, 1);

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

void initESPNow()
{
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    while (true)
    {
      Serial.println("Error initializing ESP-NOW");
      digitalWrite(4, 0);
      delay(500);
      digitalWrite(4, 1);
      delay(500);
    }
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

/*---------------------------------------------------------------------*/

// Görev fonksiyonları

void setup()
{
#if DEBUG_MODE
  Serial.begin(9600);
  printConsoleTask.start();
#endif

  setPins();
  adjustInputsTask.start();
  initESPNow();
}

void loop()
{
  printConsoleTask.update();
  lockdownCheck();
  adjustInputsTask.update();
  omniDrive();
}

/*---------------------------------------------------------------------*/

// Döngü fonksiyonları

void printConsole()
{
  Serial.println("-----------------------------");
/*
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

  Serial.print("\n\n");
  Serial.println(angle);

  Serial.print("\n\n");
  Serial.print("cos: ");
  Serial.println(power * cos(angle * PI / 180));
  Serial.print("sin: ");
  Serial.println(-power * sin(angle * PI / 180));
*/
  Serial.print("chargeInfo: ");
  Serial.println(chargeInfo);

  Serial.println("-----------------------------");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  xValueGas = map(joystickData.received_xValueGas, 0, 255, -255, 255);
  yValueGas = map(joystickData.received_yValueGas, 0, 255, -255, 255);
  xValueStr = map(joystickData.received_xValueStr, 0, 255, -255, 255);
  yValueStr = map(joystickData.received_yValueStr, 0, 255, -255, 255);
  digitalWrite(4, 1);
  dataReceived = true;
}

void linearlyRefineInputs(int &determinedValue, int currentValue, int Accel)
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
  float currentOffset = motorOffsets[((channel_L + 1) / 2) - 1];

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
    ledcWrite(channel_R, R_value * currentOffset);
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
    ledcWrite(channel_L, L_value * currentOffset);
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

void lockdownCheck() // Bu versiyonda tamamen kırık. Düzeltilecek.
{
  if (!dataReceived)
  {
    if (emergencyLockdownTask.state() == STOPPED)
    {
      emergencyLockdownTask.start();
    }
    emergencyLockdownTask.update();
  }
  else
  {
    if (emergencyLockdownTask.state() == RUNNING)
    {
      emergencyLockdownTask.stop();
    }
  }
}

void omniDrive() // Şu fonksiyonu yazmaya giden zamanı bi ben bide halil biliyo -y
{
  power = sqrt(det_xValueGas * det_xValueGas + det_yValueGas * det_yValueGas);
  angle = (atan2(det_yValueGas, det_xValueGas) * 180 / PI) - 45;
  turn = det_xValueStr;

  omniX = map(power * cos(angle * PI / 180), -360, 360, -255, 255);
  omniY = map(-power * sin(angle * PI / 180), -360, 360, -255, 255);

  int motor1 = constrain(omniX + turn, -255, 255);
  int motor2 = constrain(omniY - turn, -255, 255); // Lütfen çalışsın
  int motor3 = constrain(omniX - turn, -255, 255);
  int motor4 = constrain(omniY + turn, -255, 255);

  outputPwmValues(motor1, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  outputPwmValues(motor2, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  outputPwmValues(motor3, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  outputPwmValues(motor4, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}

void chargeCheck()
{

  chargeValue = analogRead(CHARGE_PIN);
  chargeInfo = map(chargeValue, 3200, 4095, 0, 100);
  
}
