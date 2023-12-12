#include "EEPROM.h"

#include <M5Stack.h>
#include <Wire.h>
#include <WiFi.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "config.h"

#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <time.h>
#include <sys/time.h>

#include "esp_system.h"

const int wdtTimeout = 10000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

int addr = 0;
#define EEPROM_SIZE 2

#ifdef DEBUG_DUMP_AT_COMMAND
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger, SIM7020_RESET);

#else
TinyGsm       modem(SerialAT);

#endif

TinyGsmClient tcpClient(modem);
PubSubClient  mqttClient(MQTT_BROKER, MQTT_PORT, tcpClient);

void mqttCallback(char *topic, byte *payload, unsigned int len);
void mqttConnect(void);
void nbConnect(void);

struct tm now;
char s_time[50];
char buffer1[2000] = "123"; // = "{\"max_x\":22,\"max_y\":21,\"frame\":7,\"data\":\"SiUwQy0+Lk4dHyshSmDPxH1lRkpDRzA5UVRMS1VBSCYaSCA0OD5IaicpJB0nTrHdmoBKTkdCQz5HUk1bXzNIPTE2USVVPVmKRw4WEggnXnbK03tsV1dBPlFTWlVdNEVDNUMzLi87PT8bJyQeMzVDW6PEqYlQV0paT0pFTjxFQ0gwHjs3ODE/T2VdR1RBUEs/U2i5x4x6Yl9fXVxIRkJPTjcxRzxFNz1UiYhcZUxFTkNEWIO3tJJsXF1XUDpGS0FGKzQtLDI+TF2IlZqKg2VYV0VMQmWmv4uDYldMUT41QkYjNDgxMT1OaICQpqOIhG9kQVNGS32ippFmamJJND5BNzAlJDA0P0Zee4uluaesoZR1ZkpTTluhsW5sSlBQSUpNCSoiPDk2VGSOoJmkpZyopImFZVlMSXOoimtoVD1OK1QjJhgcLC9GWo6cmJSYn5WjqKmRiWxdXFesmmNbQCxPSgkoGiw6OD5VmKCZoZWVmJiboqedh21JUIGhW1JFO0Fk\",\"min\":\"23.0\",\"max\":\"34.0\"}{\"max_x\":22,\"max_y\":21,\"frame\":7,\"min\":\"23.0\",\"max\":\"34.0\",\"data\":\"GBskIjI5QUePm6GpjZyippSgnKOgomdgR1N5VUtHkasWJik/LD1AW4eYpKqYpKWhmpWTnqalgG08QkNSOFmepzAGGiY0OjlPfaSur6appKKnn5+ik5ihjV1DRDRKcrPVCicaLyoyNkyBnbWroqmnp6Knnq2OoJ+XVEdDPld6zL4gHDE3KS01RX2ep7WlrbG4qZyrmqWUmKR8WEdCa6HCxxwsGiMuMTVRc4W3rrGqu7Ottqmro56nnpF3S0lxn8rfEyw7Oj48N1Flfr6yua+5t7e0vrasrbmluauOpbzB2d0kMys6PjxJRmV3w8G2tL++yMu8xLnHurjOyL3A0MTR4CRKKktMR1NhdHzCs7K5wdLQz+Db3tvm6ezoy9vM2dHsISlATkhZYmRmerm9v83K0t/i5fPd6//r9d7b18nC1OIPREQhXExYaWB4s7rD3Onz7uzy7+fp5OTZx8/Y46++5AAzMVRIT15KWXWsuMHe6PTr7+/23eXU6b3OyNbCxsXJ\"}";



const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air

#define COLS 32
#define ROWS 24
#define COLS_2 (COLS * 2)
#define ROWS_2 (ROWS * 2)

float pixelsArraySize = COLS * ROWS;
float pixels[COLS * ROWS];
float pixels_2[COLS_2 * ROWS_2];
int reversePixels[COLS * ROWS];

byte speed_setting = 2 ; // High is 1 , Low is 2
bool reverseScreen = true;

#define INTERPOLATED_COLS 32
#define INTERPOLATED_ROWS 32

static float mlx90640To[COLS * ROWS];
paramsMLX90640 mlx90640;
float signedMag12ToFloat(uint16_t val);

//low range of the sensor (this will be blue on the screen)
int MINTEMP = 24; // For color mapping
int min_v = 24; //Value of current min temp
int min_cam_v = -40; // Spec in datasheet


//high range of the sensor (this will be red on the screen)
int MAXTEMP = 35; // For color mapping
int max_v = 35; //Value of current max temp
int max_cam_v = 300; // Spec in datasheet
int resetMaxTemp = 45;

//the colors we will be using

const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };



float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);

long loopTime, startTime, endTime, fps;
extern const unsigned char gImage_bk_eng[];
extern const unsigned char gImage_bk0[];
extern const unsigned char gImage_numberBlueWhite[];
extern const unsigned char gImage_numberRedWhite[];
extern const unsigned char gImage_numberBlueWhite0915[];
extern const unsigned char gImage_numberRedWhite0915[];
extern const unsigned char gImage_numberYellowBlack[];
extern const unsigned char gImage_online[];
extern const unsigned char gImage_pauseStart[];
extern const unsigned char gImage_mute[];
int recordMax[134];
int recordMin[134];
int recordMaxX[134], recordMaxY[134];
int recordMinX[134], recordMinY[134];
int oldRecordMaxX[134], oldRecordMaxY[134];
int oldRecordMinX[134], oldRecordMinY[134];
//int recordMin[120];
int recordMaxMax, recordMaxMin;
int recordIndex = -1;

int recordMode = 0; //0:240second;1:10 min; 2:1 hours; 3:1 day
int count = 0;
int sendCount = 0;
int frames = 0, idle = 0;
int connectStatus = 0;
int errCount = 0;
int enable = 0;
int alert = 120;
int speakerEnable = 1;
void showPauseStart()
{
  if (enable == 0)
  {
    M5.Lcd.drawBitmap(120, 215, 80, 24  , (uint16_t *)gImage_pauseStart + (80 * 24 * 0));
  }
  else
    M5.Lcd.drawBitmap(120, 215, 80, 24  , (uint16_t *)gImage_pauseStart + (80 * 24 * 1));

}

void showMute()
{
  if (speakerEnable == 0)
  {
    M5.Lcd.drawBitmap(50, 225, 32, 14  , (uint16_t *)gImage_mute + (32 * 14 * 0));
  }
  else
    M5.Lcd.drawBitmap(50, 225, 32, 14  , (uint16_t *)gImage_mute + (32 * 14 * 1));

}
void showOnline(int m)
{
  M5.Lcd.drawBitmap(272, 37, 20, 20, (uint16_t *)gImage_online + (20 * 20 * m));
}
void showSendTemp(int m)
{
  M5.Lcd.drawBitmap(272, 69, 20, 20, (uint16_t *)gImage_online + (20 * 20 * m));
}
void showSendImage(int m)
{
  M5.Lcd.drawBitmap(272, 101, 20, 20, (uint16_t *)gImage_online + (20 * 20 * m));
}
String mac;

void showAlert(int minC)
{
  int mm = minC;
  int temp = mm / 100;


  if (temp == 0)
  {
    M5.Lcd.drawBitmap(180  + 12 * 0, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * 10));
    mm = mm % 100;
    temp = mm / 10;
    M5.Lcd.drawBitmap(180 + 12 * 1, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * temp));
    mm = mm % 10;
    M5.Lcd.drawBitmap(180  + 12 * 2, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * mm));
    M5.Lcd.drawBitmap(180  + 12 * 3, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * 10));
  }
  else
  {
    M5.Lcd.drawBitmap(186 + 12 * 0, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * temp));
    mm = mm % 100;
    temp = mm / 10;
    M5.Lcd.drawBitmap(186 + 12 * 1, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * temp));
    mm = mm % 10;
    M5.Lcd.drawBitmap(186  + 12 * 2, 216, 12, 22, (uint16_t *)gImage_numberYellowBlack + (12 * 22 * mm));
  }
}

void setup()
{
  M5.begin();
  M5.Power.begin();
  Wire.begin();
  Wire.setClock(450000); //Increase I2C clock speed to 400kHz
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // EEPROM.write(0, 123);
  M5.Lcd.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.setBrightness(20);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  pinMode(25, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  while (!Serial); //Wait for user to open terminal
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int status;
  uint16_t eeMLX90640[832];//32 * 24 = 768
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)    Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)    Serial.println("Parameter extraction failed");
  int SetRefreshRate;
  SetRefreshRate = MLX90640_SetRefreshRate (0x33, 0x05);

  SerialMon.begin(MONITOR_BAUDRATE);
  SerialAT.begin(SIM7020_BAUDRATE, SERIAL_8N1, 35, 0);


  // M5.Lcd.drawBitmap(0, 0, 320, 240, (uint16_t *)gImage_bk0);
  // for (int i = 20; i < 200; i++)
  // {
  //   M5.Lcd.setBrightness(i);
  //   delay(10);
  // }
  errCount = 0;
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(15, 220); //move to bottom right 
  mac = WiFi.macAddress();
  M5.Lcd.println(mac);
  mac.replace(":", "");
  String str = "https://things.m5stack.com/?DeviceID=M5";
  str += mac;
  str+= "M5&Product=TO&NetworkType=NB-IoT";
  // M5.Lcd.qrcode(str, 160, 90, 140, 6);
  // M5.Lcd.setTextSize(1);
  // while (connectStatus == 0)
  // {
  //   nbConnect();
  //   if (errCount < 5)
  //   {
  //     M5.Lcd.setCursor(295, 35);
  //     M5.Lcd.printf("%d", errCount);
  //   }
  //   else
  //     ESP.restart();
  // }
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(300);
  showOnline(4);
  showSendTemp(4);
  showSendImage(4);
  for (int i = 0; i < 50; i++)
  {
    digitalWrite(25, HIGH);
    delayMicroseconds(120);
    digitalWrite(25, LOW);
    delayMicroseconds(120);
  }
  M5.Lcd.drawBitmap(0, 0, 320, 240, (uint16_t *)gImage_bk_eng);
  Serial.printf("%d\n", EEPROM.read(0));
  if (EEPROM.read(0) != 0xff)
    alert = EEPROM.read(0);
  else
  {
    alert = 120;
    EEPROM.write(0, 120);
    EEPROM.commit();
  }
  showAlert(alert);
  showMute();
  dacWrite(25, 0);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
}
void showNumMax(int maxC)
{
  int mm = maxC;
  int temp = mm / 100;
  if (temp != 0)
    M5.Lcd.drawBitmap(140 + 12 * 0, 55, 12, 22, (uint16_t *)gImage_numberRedWhite + (12 * 22 * temp));
  else
    M5.Lcd.drawBitmap(140 + 12 * 0, 55, 12, 22, (uint16_t *)gImage_numberRedWhite + (12 * 22 * 10));
  mm = mm % 100;
  temp = mm / 10;
  M5.Lcd.drawBitmap(140 + 12 * 1, 55, 12, 22, (uint16_t *)gImage_numberRedWhite + (12 * 22 * temp));
  mm = mm % 10;
  M5.Lcd.drawBitmap(140 + 12 * 2, 55, 12, 22, (uint16_t *)gImage_numberRedWhite + (12 * 22 * mm));
}

void showNumMin(int minC)
{
  int mm = minC;
  int temp = mm / 100;
  if (temp != 0)
    M5.Lcd.drawBitmap(140 + 12 * 0, 83, 12, 22, (uint16_t *)gImage_numberBlueWhite + (12 * 22 * temp));
  else
    M5.Lcd.drawBitmap(140 + 12 * 0, 83, 12, 22, (uint16_t *)gImage_numberBlueWhite + (12 * 22 * 10));
  mm = mm % 100;
  temp = mm / 10;
  M5.Lcd.drawBitmap(140 + 12 * 1, 83, 12, 22, (uint16_t *)gImage_numberBlueWhite + (12 * 22 * temp));
  mm = mm % 10;
  M5.Lcd.drawBitmap(140 + 12 * 2, 83, 12, 22, (uint16_t *)gImage_numberBlueWhite + (12 * 22 * mm));
}


void showRecordNumMax(int mm)
{
  int temp = mm / 100;
  if (temp != 0)
    M5.Lcd.drawBitmap(3 + 9 * 0, 149, 9, 15, (uint16_t *)gImage_numberRedWhite0915 + (9 * 15 * temp));
  else
    M5.Lcd.drawBitmap(3 + 9 * 0, 149, 9, 15, (uint16_t *)gImage_numberRedWhite0915 + (9 * 15 * 10));
  mm = mm % 100;
  temp = mm / 10;
  M5.Lcd.drawBitmap(3 + 9 * 1, 149, 9, 15, (uint16_t *)gImage_numberRedWhite0915 + (9 * 15 * temp));
  mm = mm % 10;
  M5.Lcd.drawBitmap(3 + 9 * 2, 149, 9, 15, (uint16_t *)gImage_numberRedWhite0915 + (9 * 15 * mm));
}

void showRecordNumMin(int mm)
{
  int temp = mm / 100;
  if (temp != 0)
    M5.Lcd.drawBitmap(3 + 9 * 0, 182, 9, 15, (uint16_t *)gImage_numberBlueWhite0915 + (9 * 15 * temp));
  else
    M5.Lcd.drawBitmap(3 + 9 * 0, 182, 9, 15, (uint16_t *)gImage_numberBlueWhite0915 + (9 * 15 * 10));
  mm = mm % 100;
  temp = mm / 10;
  M5.Lcd.drawBitmap(3 + 9 * 1, 182, 9, 15, (uint16_t *)gImage_numberBlueWhite0915 + (9 * 15 * temp));
  mm = mm % 10;
  M5.Lcd.drawBitmap(3 + 9 * 2, 182, 9, 15, (uint16_t *)gImage_numberBlueWhite0915 + (9 * 15 * mm));
}


float oldMaxC = 1, oldMinC = 1;
float avgMaxC = 0;
int firstMax = 0;
int firstRecord = 0;
int lcdIdle = 0;
void loop()
{
  timerWrite(timer, 0);
  if (lcdIdle > 200)
  {
    M5.Lcd.setBrightness(20);
  }
  else
  {
    lcdIdle++;
    M5.Lcd.setBrightness(200);
  }
  if (digitalRead(39) == LOW)
  {
    lcdIdle = 0;
    while (digitalRead(39) == LOW);
    if (speakerEnable == 0)
      speakerEnable = 1;
    else
      speakerEnable = 0;

    for (int i = 0; i < 10; i++)
    {
      digitalWrite(25, HIGH);
      delayMicroseconds(120);
      digitalWrite(25, LOW);
      delayMicroseconds(120);
    }
    showMute();
    // showPauseStart();
  }


  if (digitalRead(38) == LOW)
  {
    lcdIdle = 0;
    while (digitalRead(38) == LOW);
    if (alert < 200)alert += 5;
    EEPROM.write(0, alert);
    EEPROM.commit();
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(25, HIGH);
      delayMicroseconds(120);
      digitalWrite(25, LOW);
      delayMicroseconds(120);
    }
    showAlert(alert);
    // showPauseStart();
  }
  if (digitalRead(37) == LOW)
  {
    lcdIdle = 0;
    while (digitalRead(37) == LOW);
    if (alert > 10)alert -= 5;
    EEPROM.write(0, alert);
    EEPROM.commit();
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(25, HIGH);
      delayMicroseconds(110);
      digitalWrite(25, LOW);
      delayMicroseconds(110);
    }
    showAlert(alert);
    // showPauseStart();
  }
  // loopTime = millis();
  // startTime = loopTime;
  static unsigned long timer = 0;

  // Serial.println(errCount);
  if (errCount > 20)
    // ESP.restart();
  if (!mqttClient.connected())
  {

    errCount++;
    if (!modem.isNetworkConnected())
    {
      Serial.print("disconnected!\n");
      showOnline(1);
      nbConnect();
    }
    if (connectStatus == 1)
    {
      SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
      mqttConnect();
    }
  }
  for (byte x = 0 ; x < speed_setting ; x++) // x < 2 Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }
    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels); //save pixels temp to array (pixels)
  }

  if (reverseScreen == 1)
  {
    for (int x = 0 ; x < 32 * 24 ; x++)
    {
      if (x % COLS == 0) //32 values wide
      {
        for (int j = 0 + x, k = (COLS - 1) + x; j < COLS + x ; j++, k--)
        {
          reversePixels[j] = (int)(pixels[k]);
        }
      }
    }
  }





  int ppp[32 * 24];
  for (int i = 0; i < 32 * 24; i++)
  {
    ppp[i] = (int)(reversePixels[i]);
    if (ppp[i] > 250)ppp[i] = 250;
    if (ppp[i] < 1)ppp[i] = 1;
  }
  //进行排序
  int temppp = 0;
  for (int j = 0; j < 32 * 24 - 2; j++)
    for (int i = 0; i < 32 * 24 - 1; i++)
    {
      if (ppp[i] < ppp[i + 1])
      {
        temppp = ppp[i + 1];
        ppp[i + 1] = ppp[i];
        ppp[i] = temppp;
      }
    }

  for (int i = 0; i < 32 * 24; i++)
  {
    if ((reversePixels[i]) == ppp[0])
    {
      reversePixels[i] = ppp[2];
      // Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", i, reversePixels[i], ppp[0], ppp[1], ppp[2], ppp[3], ppp[4], ppp[32 * 24 - 1], ppp[32 * 24 - 2]);
    }
    if ((int)(reversePixels[i]) == ppp[1])
    {
      reversePixels[i] = ppp[3];
      // Serial.printf("%d,%f,%d,%d,%d,%d,%d,%d,%d\n", i, reversePixels[i], ppp[0], ppp[1], ppp[2], ppp[3], ppp[4], ppp[32 * 24 - 1], ppp[32 * 24 - 2]);
    }
    if ((int)(reversePixels[i]) == ppp[32 * 24 - 1])
    {
      reversePixels[i] = ppp[32 * 24 - 3];
      //Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", i, reversePixels[i], ppp[0], ppp[1], ppp[2], ppp[3], ppp[4], ppp[32 * 24 - 1], ppp[32 * 24 - 2]);
    }
    if ((int)(reversePixels[i]) == ppp[32 * 24 - 2])
    {
      reversePixels[i] = (float)(ppp[32 * 24 - 4]);
      //Serial.printf("%d,%f,%d,%d,%d,%d,%d,%d,%d\n", i, reversePixels[i], ppp[0], ppp[1], ppp[2], ppp[3], ppp[4], ppp[32 * 24 - 1], ppp[32 * 24 - 2]);
    }

  }
  //Serial.printf("\n");

  int maxC = -10000;
  int minC = 23300;

  for (int i = 0; i < 32 * 24; i++)
  {
    if (reversePixels[i] > 300)return;
    if (reversePixels[i] < 1)return;
    if (reversePixels[i] < 2)reversePixels[i] = 1;

    if (reversePixels[i] > 250)
    {

      reversePixels[i] = 250;
    }

    if (reversePixels[i] > maxC)
    {
      maxC = reversePixels[i];
    }
    if (reversePixels[i] < minC)
    {
      minC = reversePixels[i];
    }
  }

  count = 0;
  for (int i = 0; i < 24; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      int colorTemp = reversePixels[32 * i + j];
      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);// 0 ~ 255
      M5.Lcd.fillRect(7 + j * 4,  31 + i * 4, 4, 4, camColors[colorIndex]);
      buffer1[count++] = colorTemp; //(char)( (camColors[colorIndex] & 0xff00) >> 8);
    }
  }
  showNumMax(maxC);
  showNumMin(minC);
  if (maxC > alert)
  {
    lcdIdle = 0;
    if (speakerEnable == 1)
      for (int i = 0; i < 100; i++)
      {
        dacWrite(25, 20);
        delayMicroseconds(300);
        dacWrite(25, 0);
        delayMicroseconds(500);
      }
  }

  //float recordMaxScope,recordMScope
  for (int i = 0; i < 133; i++)
  {
    recordMax[133 - i] = recordMax[132 - i];
    recordMin[133 - i] = recordMin[132 - i];
  }
  recordMax[0] = (int)(maxC);
  recordMin[0] = (int)(minC);

  recordMaxMax = -1000; recordMaxMin = 1000;
  for (int i = 0; i < 134; i++)
  {
    if (recordMax[i] > recordMaxMax)
      recordMaxMax = recordMax[i];
    if (recordMin[i] != 0)
      if (recordMin[i] < recordMaxMin  )
        recordMaxMin = recordMin[i];
  }
  showRecordNumMax(recordMaxMax);
  showRecordNumMin(recordMaxMin);
  int h = recordMaxMax - recordMaxMin;
  for (int i = 0; i < 134; i++)
  {
    recordMaxX[i] = 44 + i * 2;
    recordMaxY[i] = 205 - (int)(recordMax[i] - recordMaxMin)  * 65 / h;
    if (recordMaxY[i] > 205)
      recordMaxY[i] = 205;
    recordMinX[i] = 44 + i * 2;
    recordMinY[i] = 205 - (int)(recordMin[i] - recordMaxMin)  * 65 / h;
    if (recordMinY[i] > 205)
      recordMinY[i] = 205;
  }
  if (firstRecord != 0)
    for (int i = 0; i < 133; i++)
    {
      M5.Lcd.drawLine(oldRecordMaxX[i], oldRecordMaxY[i], oldRecordMaxX[i + 1], oldRecordMaxY[i + 1], 0xE73C);
      M5.Lcd.drawLine(oldRecordMinX[i], oldRecordMinY[i], oldRecordMinX[i + 1], oldRecordMinY[i + 1], 0xE73C);
    }
  for (int i = 0; i < 133; i++)
  {
    if (recordMax[i + 1] != 0)
    {
      M5.Lcd.drawLine(recordMaxX[i], recordMaxY[i], recordMaxX[i + 1], recordMaxY[i + 1], RED);
      M5.Lcd.drawLine(recordMinX[i], recordMinY[i], recordMinX[i + 1], recordMinY[i + 1], BLUE);
    }
  }
  firstRecord = 1;
  for (int i = 0; i < 134; i++)
  {
    oldRecordMaxX[i] = recordMaxX[i] ;
    oldRecordMaxY[i] = recordMaxY[i] ;
    oldRecordMinX[i] = recordMinX[i] ;
    oldRecordMinY[i] = recordMinY[i] ;
  }





  if (millis() >= timer)
  {
    timer = millis() + 500;//UPLOAD_INTERVAL;
    idle++;
    switch (idle % 120)
    {
      case 0: case 10: case 20: case 30: case 40: case 50:
      case 60: case 70: case 80: case 90: case 100: case 110:
        buffer1[0] = maxC;
        buffer1[1] = 0;
        if (connectStatus == 1)
        {
          //char buf[50];
          //mac.toCharArray(buf, 20);
          // Serial.println(mac);
          char ch[20];
          String str = mac + "/T";
          str.toCharArray(ch, 15);
          // Serial.println(ch);
          mqttClient.publish( ch, buffer1);  // 发送数据
          getLocalTime(&now, 0);
          strftime(s_time, sizeof(s_time), "%Y-%m-%d %H:%M:%S", &now);
          Serial.println(s_time);
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.setTextSize(1);
          M5.Lcd.setCursor(12, 216);
          M5.Lcd.println(s_time);
          sendCount++;
          showSendTemp(3);
        }
        break;
      case 5:
        if (connectStatus == 1)
        {
          char ch[20];
          String str = mac + "/I";
          str.toCharArray(ch, 15);
          // Serial.println(ch);
          mqttClient.publish(ch , buffer1);  // 发送数据
          getLocalTime(&now, 0);
          strftime(s_time, sizeof(s_time), "%Y-%m-%d %H:%M:%S", &now);
          Serial.println(s_time);
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.setTextSize(1);
          M5.Lcd.setCursor(12, 216);
          M5.Lcd.println(s_time);
          sendCount++;
          showSendImage(3);
        }
        break;
      default:  showSendTemp(4); showSendImage(4); break;
    }

    if (idle % 2 == 1)
    {
      if (connectStatus == 0)
        showOnline(2);
      else
        showOnline(0);
    }
    else
    {
      showOnline(4);
    }




    //M5.Lcd.setTextColor(BLACK, WHITE);
    // M5.Lcd.setTextSize(2);
    //M5.Lcd.setCursor(60, 140); //move to bottom right
    //M5.Lcd.printf("Send:%d,idle:%d", sendCount, idle);

    // getLocalTime(&now, 0);
    // strftime(s_time, sizeof(s_time), " % Y - % m - % d % H: % M: % S", &now);
    // SerialMon.println(s_time);
  }

  if (connectStatus == 1)
    mqttClient.loop();


  // loopTime = millis();
  //endTime = loopTime;
  //fps = 1000 / (endTime - startTime);
  /*M5.Lcd.fillRect(300, 209, 20, 12, TFT_BLACK); //Clear fps text area
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(284, 210);
    M5.Lcd.print("fps: " + String( fps ));
    M5.Lcd.setTextSize(1);
  */

}


void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  SerialMon.print(F("Message arrived ["));
  SerialMon.print(topic);
  SerialMon.print(F("]: "));
  SerialMon.write(payload, len);
  SerialMon.println();

  if (strcmp(MQTT_SYNC_TIME_D_TOPIC, topic) == 0) {
    time_t timestamps = atoi((char *)payload) + 8 * 60 * 60;
    timeval epoch = {timestamps, 0};
    settimeofday((const timeval *)&epoch, 0);  // 同步时间
    SerialMon.println("Sync time success");
    mqttClient.publish(MQTT_SYNC_TIME_U_TOPIC, "time sync success");
  } else {
    SerialMon.println("Sync time fail");
    mqttClient.publish(MQTT_SYNC_TIME_U_TOPIC, "time sync fail");
  }
}

void mqttConnect(void)
{
  SerialMon.print(F("Connecting to "));
  SerialMon.print(MQTT_BROKER);
  SerialMon.print(F("..."));

  // Connect to MQTT Broker
  String mqttid = "MQTTID_" + mac;

  if (!mqttClient.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD))
  {
    connectStatus = 0;
    errCount++;
    SerialMon.println(F(" fail"));
    return;
  }
  SerialMon.println(F(" success"));
  mqttClient.subscribe(MQTT_SYNC_TIME_D_TOPIC);
  if (!mqttClient.publish(MQTT_SYNC_TIME_U_TOPIC, "time sync"))
  {
    connectStatus = 0;
    errCount++;
    delay(1000);
    return;
  }
  connectStatus = 1;
}
void mqttConnect1(void)
{
  SerialMon.print(F("Connecting to "));
  SerialMon.print(MQTT_BROKER);
  SerialMon.print(F("..."));

  // Connect to MQTT Broker
  String mqttid = ("MQTTID_" + String(random(65536)));
  while (!mqttClient.connect(mqttid.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    SerialMon.println(F(" fail"));
  }
  SerialMon.println(F(" success"));
  mqttClient.subscribe(MQTT_SYNC_TIME_D_TOPIC);
  while (!mqttClient.publish(MQTT_SYNC_TIME_U_TOPIC, "time sync")) {
    delay(1000);
  }
}


void nbConnect(void)
{

  digitalWrite(12, HIGH);
  delay(1000);
  digitalWrite(12, LOW);
  delay(100);
  Serial.println("Initializing modem...");
  delay(1000);

  if (!modem.init())
  {
    connectStatus = 0;
    errCount++;
    Serial.print("Modem fail.\n");
    return;
  }

  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    connectStatus = 0;
    errCount++;
    Serial.print("Network fail.\n");
    return;
  }
  Serial.print("Success.\n");
  connectStatus = 1;
  errCount = 0;
}


void nbConnect1(void)
{
  Serial.println("Initializing modem...");

  int errCount = 0;
  while (!modem.init())
  {

    if (errCount < 3)
      errCount++;
    else
      ESP.restart();
    Serial.print("Modem fail.\n");
  };
  errCount = 0;
  Serial.println("Waiting for network...");
  while (!modem.waitForNetwork())
  {
    if (errCount < 3)
      errCount++;
    else
      ESP.restart();
    Serial.print("Network fail.\n");
  }
  Serial.print("Success.\n");
}
/***infodisplay()*****/
void infodisplay(void)
{
  M5.Lcd.fillRect(0, 198, 320, 4, TFT_WHITE);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.fillRect(284, 223, 320, 240, TFT_BLACK); //Clear MaxTemp area
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(284, 222); //move to bottom right
  M5.Lcd.print(MAXTEMP , 1);  // update MAXTEMP
  M5.Lcd.printf("C" , 1);
  M5.Lcd.setCursor(0, 222);  // update MINTEMP text
  M5.Lcd.fillRect(0, 222, 36, 16, TFT_BLACK);
  M5.Lcd.print(MINTEMP , 1);
  M5.Lcd.printf("C" , 1);
  M5.Lcd.setCursor(106, 224);
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal)
{
  int colorTemp;
  //Serial.printf("w % d, h % d", rows, cols);
  for (int y = 0; y < rows; y++)
  {
    for (int x = 0; x < cols; x++)
    {
      float val = get_point(p, rows, cols, x, y);

      if (val >= MAXTEMP)
        colorTemp = MAXTEMP;
      else if (val <= MINTEMP)
        colorTemp = MINTEMP;
      else colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);// 0 ~ 255
      //draw the pixels!
      uint16_t color;
      color = val * 2;
      //M5.Lcd.fillRect(boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
      M5.Lcd.fillRect(x * 4,  y * 4, 4, 4, camColors[colorIndex]);
    }
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
