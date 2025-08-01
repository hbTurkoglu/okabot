
/*---------------------------------------------------------------------*/

// Kütüphaneler.

#include <Ticker.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

/*---------------------------------------------------------------------*/

#define DEBUG_MODE true


// Pin tanımlamaları.

#define MOTOR_1_R_PWM 12
#define MOTOR_1_L_PWM 14

#define MOTOR_2_R_PWM 27
#define MOTOR_2_L_PWM 26

#define MOTOR_3_L_PWM 32
#define MOTOR_3_R_PWM 15

#define MOTOR_4_L_PWM 25
#define MOTOR_4_R_PWM 33

#define ENABLE_PIN 21

#define PP_BUTTON 9
#define AD_BUTTON 9

#define BATTERY_PIN 34

#define DB_PIN_R 2
#define DB_PIN_G 18
#define DB_PIN_B 5
#define DB_PIN_RF 4

#define BUZZER_PIN  23


// İsim tanımlamaları.

#define FRONT_SENSOR 0
#define LEFT_FRONT_SENSOR 1
#define LEFT_MIDDLE_SENSOR 2
#define LEFT_REAR_SENSOR 3
#define REAR_SENSOR 4
#define RIGHT_FRONT_SENSOR 5
#define RIGHT_MIDDLE_SENSOR 6
#define RIGHT_REAR_SENSOR 7


// Parametreler

#define MAX_POWER_PERCENT 50  // Motorların maksimum gücü (yüzde)
#define PP_SPEED 30 //255 üzerinden.
#define SLOWDOWNZONE 40 // MAX_POWER 'dan küçük olmak zorunda. Yoksa... öngörülemeyen sonuçlar ortaya çıkabilir.
#define MOTION_START 10 // Büyüdükçe harekete daha erken başlar ama hassaslık azalır.

#define SPEED_ADJUSTING_FREQ 1  //Küçüldükçe ivmelenme artar
#define ACCELERATION 1  //Büyüdükçe ivmelenme artar

#define LOCKDOWN_TIME 500 //Veri alımı zaman aşımı durumunda acildurum kapanması için beklenecek süre (ms).

#define SENSOR_COUNT 8  // Ultrasonik sensör sayısı

#define AD_MIN_DIST 20 // Assisted driving için minimum mesafe
#define AD_MAX_DIST 35  // Assisted driving için maksimum mesafe

/*---------------------------------------------------------------------*/

// Görevler

void adjustInputs();
Ticker adjustInputsTask(adjustInputs, SPEED_ADJUSTING_FREQ, 0, MILLIS); // Girişleri oku


void pp_findSpot_CB();
Ticker pp_findSpotTask(pp_findSpot_CB, 10, 0, MILLIS);

void pp_parkToSpot_CB();
Ticker pp_ParkToSpotTask(pp_parkToSpot_CB, 10, 0, MILLIS); 

void emergencyLockdown();
Ticker emergencyLockdownTask(emergencyLockdown, LOCKDOWN_TIME, 0, MILLIS); // Acil durum kapatma
bool dataReceived = false;

void printConsole();
Ticker printConsoleTask(printConsole, 2000);

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

int maxPwmPower = 0; // Motorların maksimum gücü (PWM değeri). Kod içinde hesaplanır.

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

bool deactivateInput = false;
bool parallelParkingBool = false;
bool assistedDrivingBool = false;
bool currentlyParallelParking = false;


float motorOffsets[4] = {0.95505, 1.0625, 0.9659, 1.02409};
float motorOffsets_reverse[4] = {1.04938, 1.07594, 0.94444, 1.18055};

byte arduinoDistances[SENSOR_COUNT];
int safeDistance = 25;


unsigned long pp_spotStartTime;
unsigned long pp_spotEndTime;



/*---------------------------------------------------------------------*/

// Objeler.

uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0xBD, 0x32, 0x0C}; // Kumanda mac adresi

typedef struct
{
  int sent_xValueGas = 0, sent_yValueGas = 0, sent_xValueStr = 0, sent_yValueStr = 0;
  bool sent_parallelParking = false;
  bool sent_assistedDriving = false;
} JoystickData;

JoystickData joystickData;

esp_now_peer_info_t peerInfo;

/*---------------------------------------------------------------------*/

// prototipler
void setPins();
void omniDrive();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void initESPNow();
void lockdownCheck();
void getArduinoData();
void assistedDriving();
void fatalError(int pinToBlink);
void initializeParallelParking();

/*---------------------------------------------------------------------*/

// Başlancıç fonksiyonları

void setPins()
{
  pinMode(DB_PIN_RF, OUTPUT);
  //pinMode(DB_PIN_R, OUTPUT);
  //pinMode(DB_PIN_G, OUTPUT);
  pinMode(DB_PIN_B, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(DB_PIN_R, 0);
  digitalWrite(DB_PIN_G, 0);
  digitalWrite(DB_PIN_B, 0);
  digitalWrite(DB_PIN_RF, 0);
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
    Serial.println("Error initializing ESP-NOW");
    fatalError(DB_PIN_RF);
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_wifi_set_max_tx_power(78);
}


void calculateMaxPwmPower()
{
  if (MAX_POWER_PERCENT > 100)
  {
    Serial.println("MAX_POWER_PERCENT is greater than 100.");
    fatalError(DB_PIN_B);
  }
  else if (MAX_POWER_PERCENT < 0)
  {
    Serial.println("MAX_POWER_PERCENT is less than 0.");
    fatalError(DB_PIN_B);
  }

  maxPwmPower = map(MAX_POWER_PERCENT, 0, 100, 0, 255);
}

void fatalError(int pinToBlink) // Hata durumunda pinleri yanıp söndür ve kodu sonsuz döngüye sokarak ilerleyen hasarı önler.
{
  Serial.println("Fatal error for some reason.");
  while (true)
  {
    digitalWrite(pinToBlink, 0);
    delay(500);
    digitalWrite(pinToBlink, 1);
    delay(500);
  }
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
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  adjustInputsTask.start();
  initESPNow();
  calculateMaxPwmPower();
}

void loop()
{
  getArduinoData();
  if (assistedDrivingBool){assistedDriving();}
  if (parallelParkingBool && !currentlyParallelParking) {initializeParallelParking();}
  printConsoleTask.update();
  lockdownCheck();
  adjustInputsTask.update();
  omniDrive();

  if (pp_findSpotTask.state() == RUNNING)
  {pp_findSpotTask.update();}
  if (pp_ParkToSpotTask.state() == RUNNING)
  {pp_ParkToSpotTask.update();}


}

/*---------------------------------------------------------------------*/

// Döngü fonksiyonları


void getArduinoData()
{
  if (Serial2.available() >= SENSOR_COUNT)
  {
    Serial2.readBytes(arduinoDistances, SENSOR_COUNT);



   for (int i = 0; i < SENSOR_COUNT; i++)
    {
      if (arduinoDistances[i] <= 0) {arduinoDistances[i] = 1;}


      if (arduinoDistances[i] == 1 || arduinoDistances[i] > 40)
      {
        digitalWrite(DB_PIN_B, 0);
      }
      else
      {
        digitalWrite(DB_PIN_B, 1);
      }
    }
  }
}



void printConsole()
{
  Serial.println("-----------------------------");


  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(i+1);
    Serial.print(". Sensor: ");
    Serial.println(arduinoDistances[i]);
  }

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

  Serial.print("\n\nangle");
  Serial.println(angle);

  Serial.print("\n\n");
  Serial.print("cos: ");
  Serial.println(power * cos(angle * PI / 180));
  Serial.print("sin: ");
  Serial.println(-power * sin(angle * PI / 180));

  Serial.println("-----------------------------"); */
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&joystickData, incomingData, sizeof(joystickData));


    xValueGas = map(joystickData.sent_xValueGas, 0, 1023, -255, 255);
    yValueGas = map(joystickData.sent_yValueGas, 0, 1023, -255, 255);
    xValueStr = map(joystickData.sent_xValueStr, 0, 1023, -255, 255);
    yValueStr = map(joystickData.sent_yValueStr, 0, 1023, -255, 255);
  

  parallelParkingBool = joystickData.sent_parallelParking;
  assistedDrivingBool = joystickData.sent_assistedDriving;

  //Belki gerizekalıca.


  digitalWrite(DB_PIN_RF, 1);
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
  float currentOffset_reverse = motorOffsets_reverse[((channel_L + 1) / 2) - 1];

  if (input > middlePoint)
  {
    if (L_value != 0)
    {
      L_value = 0;
      ledcWrite(channel_L, L_value); // Sol kanalı kapat
    }

    if (input > middlePoint + SLOWDOWNZONE)
    {
      R_value = map(input, middlePoint + SLOWDOWNZONE, 255, SLOWDOWNZONE, maxPwmPower);
    }
    else
    {
      R_value = map((input - middlePoint) * (input - middlePoint), 0, SLOWDOWNZONE * SLOWDOWNZONE, MOTION_START, SLOWDOWNZONE);
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
      L_value = map(input, -255, middlePoint - SLOWDOWNZONE, maxPwmPower, SLOWDOWNZONE);
    }
    else
    {
      L_value = map((input - middlePoint) * (input - middlePoint), 0, -SLOWDOWNZONE * SLOWDOWNZONE, MOTION_START, SLOWDOWNZONE);
    }
    ledcWrite(channel_L, L_value * currentOffset_reverse);
  }
}

void emergencyLockdown()
{
  xValueGas = joystickIdleValue;
  yValueGas = joystickIdleValue;
  xValueStr = joystickIdleValue;
  yValueStr = joystickIdleValue;
  digitalWrite(DB_PIN_RF, 0);
}

void lockdownCheck()
{
  if (!dataReceived)
  {
    if (emergencyLockdownTask.state() == STOPPED)
    {
      emergencyLockdownTask.start();
      /*
      Serial.println("\n-----------------------------");
      Serial.println("System is at emergeny lockdown.");
      Serial.println("-----------------------------");
      */
    }
    emergencyLockdownTask.update();
  }
  else
  {
    if (emergencyLockdownTask.state() == RUNNING)
    {
      emergencyLockdownTask.stop();
      /*
      Serial.println("\n-----------------------------");
      Serial.println("Eemergeny lockdown lifted.");
      Serial.println("-----------------------------");
      */
    }
  }
  dataReceived = false; //kontrol bool'unu sıfırla
}

void omniDrive() // Şu fonksiyonu yazmaya giden zamanı bi ben bide halil biliyo -y
{
  power = sqrt(det_xValueGas * det_xValueGas + det_yValueGas * det_yValueGas);
  angle = (atan2(det_yValueGas, det_xValueGas) * 180 / PI) - 45;
  turn = constrain(det_xValueStr, -144, 144);

  omniX = map(power * cos(angle * PI / 180), -360, 360, -255, 255);
  omniY = map(-power * sin(angle * PI / 180), -360, 360, -255, 255);

  int motor1 = constrain(omniX + turn, -255, 255);
  int motor2 = constrain(omniY - turn, -255, 255);
  int motor3 = constrain(omniX - turn, -255, 255);
  int motor4 = constrain(omniY + turn, -255, 255);

  outputPwmValues(motor1, pwmChannel_1R, pwmChannel_1L, joystickIdleValue);
  outputPwmValues(motor2, pwmChannel_2R, pwmChannel_2L, joystickIdleValue);
  outputPwmValues(motor3, pwmChannel_3R, pwmChannel_3L, joystickIdleValue);
  outputPwmValues(motor4, pwmChannel_4R, pwmChannel_4L, joystickIdleValue);
}

void chargeCheck()  //dertlerimizin en sonuncusu.
{
  chargeValue = analogRead(BATTERY_PIN);
  chargeInfo = map(chargeValue, 3200, 4095, 0, 100); 
}





void dampenInput(int &input, int index, bool sign)
{
  if (sign)
  {
    if (input < 0) {return;}
    input = 0;
  }
  else
  {
    if (input > 0) {return;}
    input = 0;
  }

}


void assistedDriving()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (arduinoDistances[i] <= AD_MAX_DIST)
    {

      //Serial.println("a");
      switch (i)
      {
        case FRONT_SENSOR:
        dampenInput(xValueGas, i, true);
        break;

        case LEFT_FRONT_SENSOR:
        case LEFT_MIDDLE_SENSOR:
        case LEFT_REAR_SENSOR:
        dampenInput(yValueGas, i, false);
        break;

        case REAR_SENSOR:
        dampenInput(xValueGas, i, false);
        break;

        case RIGHT_FRONT_SENSOR:
        case RIGHT_MIDDLE_SENSOR:
        case RIGHT_REAR_SENSOR:
        dampenInput(yValueGas, i, true);
        break;
      }
    }
  }
}


byte parking_dir = 0; //0 sağ, 1 sol

void initializeParallelParking()
{
  //bool pp_available = false;
  bool rightProximity = false;
  bool leftProximity = false;
  currentlyParallelParking = true;

  for (int i = 0; i < SENSOR_COUNT; i++)  //Park yönü adaylarını belirle.
  {
    if ((arduinoDistances[i] <= AD_MAX_DIST) && (i != FRONT_SENSOR) && (i != REAR_SENSOR))
    {
      if (i == RIGHT_FRONT_SENSOR || i == RIGHT_MIDDLE_SENSOR || i == RIGHT_REAR_SENSOR)
      {
        rightProximity = true;
      }
      else {leftProximity = true;}
    }
  }

  if (rightProximity && leftProximity)  //Park yönü adaylarını değerlendir.
  {
    //Park yönüne karar verilemedi, hem sağ hem solda aday var. Abort.
    //[BUZZER]
    currentlyParallelParking = false;
    return;
  }
  else if (rightProximity) {parking_dir = 0;}
  else if (leftProximity) {parking_dir = 1;}
  else 
  {
    /*Park yönüne karar verilemedi, iki taraftada aday yok. Abort.*/
    //[BUZZER]
    currentlyParallelParking = false;
    return;
  }

  //Park yeri aramaya başla
  deactivateInput = true; //bi ara kumandadan kontrolü zorla geri almanın bi yolunu ekle.
  xValueGas = PP_SPEED;
  yValueGas = 0;
  xValueStr = 0;
  pp_findSpotTask.start();
  
}


void getParallelWithTheWall()
{
  //Duvarla paralel hale gel.
}



bool frontProximity = true;
bool middleProximity = true;
bool rearProximity = true;
void pp_findSpot_CB()
{
  if (parking_dir == 0)
  {
    if (arduinoDistances[RIGHT_FRONT_SENSOR] > 35)
    {frontProximity = false;}
    if (arduinoDistances[RIGHT_MIDDLE_SENSOR] > 35)
    {middleProximity = false;}
    if (arduinoDistances[RIGHT_REAR_SENSOR] > 35)
    {rearProximity = false;}
  }
  else if (parking_dir == 1)
  {
    if (arduinoDistances[LEFT_FRONT_SENSOR] > 35)
    {frontProximity = false;}
    if (arduinoDistances[LEFT_MIDDLE_SENSOR] > 35)
    {middleProximity = false;}
    if (arduinoDistances[LEFT_REAR_SENSOR] > 35)
    {rearProximity = false;}
  }


    if (parking_dir == 0)
  {
    if (arduinoDistances[RIGHT_FRONT_SENSOR] < 35 && !frontProximity)
    {frontProximity = true;
      currentlyParallelParking = false;
    return;}
    if (arduinoDistances[RIGHT_MIDDLE_SENSOR] < 35 && !middleProximity)
    {middleProximity = true;
      currentlyParallelParking = false;
    return;}
    if (arduinoDistances[RIGHT_REAR_SENSOR] < 35 && !rearProximity)
    {rearProximity = true;
      currentlyParallelParking = false;
    return;}
  }
  else if (parking_dir == 1)
  {
    if (arduinoDistances[LEFT_FRONT_SENSOR] < 35 && !frontProximity)
    {frontProximity = true;
      currentlyParallelParking = false;
    return;}
    if (arduinoDistances[LEFT_MIDDLE_SENSOR] < 35 && !middleProximity)
    {middleProximity = true;
      currentlyParallelParking = false;
    return;}
    if (arduinoDistances[LEFT_REAR_SENSOR] < 35 && !rearProximity)
    {rearProximity = true;
      currentlyParallelParking = false;
    return;}
  }
  
  if (!frontProximity && !middleProximity && !rearProximity)
  {
    xValueGas = 0;
    if (parking_dir == 0) {yValueGas == PP_SPEED;}
    else if (parking_dir == 1) {yValueGas == -PP_SPEED;}
    pp_ParkToSpotTask.start(); //  :)
    pp_findSpotTask.stop();
  }


}

void pp_parkToSpot_CB()
{
  if (parking_dir == 0)
  {
    if (arduinoDistances[RIGHT_FRONT_SENSOR] <= 20 || arduinoDistances[RIGHT_MIDDLE_SENSOR] <= 20 || arduinoDistances[RIGHT_REAR_SENSOR] <= 20)
    {yValueGas = 0;
    pp_ParkToSpotTask.stop();
    currentlyParallelParking = false;
    return;}
  }
  else if (parking_dir == 1)
  {
    if (arduinoDistances[LEFT_FRONT_SENSOR] <= 20 || arduinoDistances[LEFT_MIDDLE_SENSOR] <= 20 || arduinoDistances[LEFT_REAR_SENSOR] <= 20)
    {yValueGas = 0;
    pp_ParkToSpotTask.stop();
    currentlyParallelParking = false;
    return;}
  }


} 