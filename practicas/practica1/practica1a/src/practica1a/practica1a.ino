// imports
#include <time.h>
#include <RTCZero.h>
#include <Arduino_MKRMEM.h>
#include <ArduinoLowPower.h>

#define MESS_TAM 31

// rtc object
RTCZero rtc;



volatile uint32_t _period_sec = 0;
char filename[] = "datos.txt";
Arduino_W25Q16DV flash(SPI1, FLASH_CS);

volatile uint32_t currentTime = 0;
volatile uint32_t lastTime = 0;

volatile int ctrl = -1;

void setup() {

  // LED config
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // MKRMEM
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);

  // PIN 5
  pinMode(5, INPUT_PULLUP);

  // PIN 1
  pinMode(1, INPUT_PULLUP);

  // INIT USB port
  SerialUSB.begin(9600);
  while (!SerialUSB) { ; }

  flash.begin();

  SerialUSB.println("Mounting...");
  int res = filesystem.mount();

  if (res != SPIFFS_OK && res != SPIFFS_ERR_NOT_A_FS) {
    SerialUSB.println("mount() faild with error code ");
    SerialUSB.println(res);
    exit(EXIT_FAILURE);
  }
  File file = filesystem.open(filename, WRITE_ONLY | TRUNCATE);
  SerialUSB.println("Mounted");

  rtc.begin();

  // RTC config
  SerialUSB.print("Starting at: ");
  SerialUSB.print(__DATE__);
  SerialUSB.print("  ");
  SerialUSB.println(__TIME__);

  if (!setDateTime(__DATE__, __TIME__)) {
    Serial.println("ERROR");
  }

  LowPower.attachInterruptWakeup(1, closing, FALLING);
  LowPower.attachInterruptWakeup(5, externalCallback, FALLING);
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmCallback, CHANGE);
  setPeriodicAlarm(9, 4);
}

void loop() {
  if (ctrl == 999) {
    filesystem.unmount();
    exit(0);
  }

  printDateTime(ctrl == 1);
  rtc.setAlarmEpoch(rtc.getEpoch() + _period_sec);
  digitalWrite(LED_BUILTIN, HIGH); delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  ctrl = -1;
  LowPower.sleep();
}

bool setDateTime(const char* date_str, const char* time_str) {

  char month_str[4];
  char months[12][4] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug",
                        "Sep", "Oct", "Nov", "Dec" };
  uint16_t i, mday, month, hour, min, sec, year;

  if (sscanf(date_str, "%3s %hu %hu", &month_str, &mday, &year) != 3) return false;
  if (sscanf(time_str, "%hu:%hu:%hu", &hour, &min, &sec) != 3) return false;

  for (i = 0; i < 12; i++) {
    if (!strncmp(month_str, months[i], 3)) {
      month = i + 1;
      break;
    }
  }

  if (i == 12) return false;

  rtc.setTime((uint8_t)hour, (uint8_t)min, (uint8_t)sec);
  rtc.setDate((uint8_t)mday, (uint8_t)month, (uint8_t)(year - 2000));
  return true;
}

void setPeriodicAlarm(uint32_t period_sec, uint32_t _offset_sec) {
  _period_sec = period_sec;
  rtc.setAlarmEpoch(rtc.getEpoch() + _offset_sec);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

void alarmCallback() {
  ctrl = 0;
}

void externalCallback() {
  currentTime = millis();
  if (currentTime <= lastTime + 200) {
    lastTime = currentTime;
    return;
  }
  ctrl = 1;
}

void closing() {
  currentTime = millis();
  if (currentTime <= lastTime + 200) {
    lastTime = currentTime;
    return;
  }
  ctrl = 999;
}

void printDateTime(bool isExternal) {
  const char* weekDay[7] = { "Sun", "Mon", "Tue", "Wed", "Thr", "Fri", "Sat" };

  time_t epoch = rtc.getEpoch();
  struct tm stm;
  gmtime_r(&epoch, &stm);

  char dateTime[38];
  if (isExternal)
    snprintf(dateTime, sizeof(dateTime), "%s %4u/%02u/%02u %02u:%02u:%02u - EXT\n",
      weekDay[stm.tm_wday],
      stm.tm_year + 1900, stm.tm_mon + 1, stm.tm_mday,
      stm.tm_hour, stm.tm_min, stm.tm_sec);
  else
    snprintf(dateTime, sizeof(dateTime), "%s %4u/%02u/%02u %02u:%02u:%02u\n",
      weekDay[stm.tm_wday],
      stm.tm_year + 1900, stm.tm_mon + 1, stm.tm_mday,
      stm.tm_hour, stm.tm_min, stm.tm_sec);

  writeFile(filename, dateTime, strlen(dateTime));
}

char writeFile(char* filename, char* data, size_t sz) {

  File file = filesystem.open(filename, WRITE_ONLY | APPEND);
  if (!file) {
    SerialUSB.println("ERROR opening the file");
    exitError();
  }

  int const writtenData = file.write((void*)data, sz);
  if (writtenData != sz) {
    SerialUSB.println("ERROR writting the file");
    exitError();
  }
  file.close();
}

void exitError() {
  filesystem.unmount();
  exit(EXIT_FAILURE);
}