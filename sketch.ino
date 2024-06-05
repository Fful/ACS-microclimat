#define DISPLAY_ADDR 0x27 // i2c адрес дисплея
// #define BME внутри библеотеки

#define PWM_PERIOD 1000 // период шим сигнала для нагревателя и охладителя
#define SENS_TIME 500 // время обновления показаний сенсоров на экране, миллисекунд
#define BRIGHT_THRESHOLD 350  // величина сигнала, ниже которой яркость переключится на минимум (0-1023)
#define TEMP_MAX_THRESHOLD 24 // максимальная температура включить охлаждение
#define TEMP_MIN_THRESHOLD 20 // минимальная температура включить нагреватель
#define CO2_THRESHOLD 1000 // максимальная концентрация CO2
#define HUM_MIN_THRESHOLD 40 // минимальная влажность
#define HUM_MAX_THRESHOLD 60 // максимальаня влажность
#define SMOKE_THRESHOLD 400 // максимальная концентрация дыма

//средние показатели, по их достижению САУ выключается
unsigned int tempAvarage = (TEMP_MAX_THRESHOLD + TEMP_MIN_THRESHOLD) / 2; // выключение устройств нагревания и охлаждения
unsigned int humAvarage = (HUM_MAX_THRESHOLD + HUM_MIN_THRESHOLD) / 2; // выключение осушителя и увлажнителя
unsigned int TEMP_MAX_THRESHOLD_2 = 1.1 * TEMP_MAX_THRESHOLD;  // x2 boost cooler
unsigned int TEMP_MIN_THRESHOLD_2 = 0.9 * TEMP_MIN_THRESHOLD; // x2 boost heater
unsigned int CO2Avarage = CO2_THRESHOLD * 0.6; // выключение вентиляции
// init PWM off
uint8_t heatPWM = 0;
uint8_t coolPWM = 0;

// библеотеки
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(DISPLAY_ADDR, 16, 2);
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
#include <MHZ19_uart.h>
MHZ19_uart mhz19;
#include "PWMrelay.h"


// распиновка
#define PHOTO A2 // фоторезистор
#define SMOKE A3 // датчик дыма

#define MHZ_RX 2 // UART RX CO2
#define MHZ_TX 3 // UART TX CO2
#define LED_LED 4 // светодиодная лента
#define LED_HEAT 5 // нагреватель
#define LED_COOL 6 // охладитель
#define LED_HUM 7 // увлажнитель
#define LED_DEHUM 8 // осушитетель
#define LED_VENT 9 // вентиляция
#define BUZZER 10 // пожар!
// Управление шимиом
PWMrelay HEATER(LED_HEAT, HIGH, PWM_PERIOD);
PWMrelay COOLER(LED_COOL, HIGH, PWM_PERIOD);

// контроль времени
unsigned long sensorsTimer = SENS_TIME;
unsigned long sensorsTimerD = 0;
unsigned long controlONTimer = SENS_TIME*3; // контролировать будем каждые три счета считываний
unsigned long controlONTimerD = 0;
unsigned long controlOFFTimer = SENS_TIME*3; // контролировать будем каждые три счета считываний
unsigned long controlOFFTimerD = 0;

bool testTimer(unsigned long & dataTimer, unsigned long setTimer) {   // Проверка таймеров
  if (millis() - dataTimer >= setTimer) {
    dataTimer = millis();
    return true;
  } else {
    return false;
  }
}


void readSensors() {
  bme.takeForcedMeasurement();
  dispTemp = bme.readTemperature();// считывание показаний температуры
  dispHum = bme.readHumidity();// считывание показаний влажности
  dispPres = (float)bme.readPressure() * 0.00750062;// считывание показаний давления и перервод в миллиметры
  dispCO2 = mhz19.getPPM(); // показания дачика углекислого газа 
  dispSmoke = analogRead(SMOKE); // показания датчика дыма
  dispPhoto= analogRead(PHOTO);
}

void drawSensors(){
  lcd.setCursor(0, 0);
  lcd.print(String(dispTemp, 1));
  lcd.write(223);
  lcd.setCursor(6, 0);
  lcd.print(String(dispHum) + "% ");
  lcd.setCursor(11, 0);
  lcd.print(Strint(int(dispPres))+"mm ")
  lcd.setCursor(0, 1);
  lcd.print(Strint(dispCO2)+"ppm ")
  lcd.setCursor(8, 1);
  lcd.print(String(int(dispSmoke))+"GAS") // не знаю что лучше в оставшееся место дым
  // lcd.print(String(int(dispPhoto))+"LED")   // или свет
}

void consoleWrite(){ //вывод в в консоль
  Serial.print(dispTemp);
  Serial.print("\t");
  Serial.print(dispHum);
  Serial.print("\t");
  Serial.print(dispPres);
  Serial.print("\t");
  Serial.print(dispCO2);
  Serial.print("\t");
  Serial.print(dispSmoke);
  Serial.print("\t");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(SMOKE, INPUT)
  pinMode(PHOTO, INPUT)
  pinMode(LED_LED , OUTPUT);
  pinMode(LED_HEAT, OUTPUT);
  pinMode(LED_COOL, OUTPUT);
  pinMode(LED_HUM, OUTPUT);
  pinMode(LED_DEHUM, OUTPUT);
  pinMode(LED_VENT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  

  lcd.init();
  lcd.backlight();
  lcd.clear();
  bool status = true;
 // mhz19 test
  lcd.setCursor(0, 0);
  lcd.print(F("MHZ-19... "));
  Serial.print(F("MHZ-19... "));
  mhz19.begin(MHZ_TX, MHZ_RX);
  mhz19.setAutoCalibration(false);
  mhz19.getStatus();    // первый запрос, в любом случае возвращает -1
  delay(500);
  if (mhz19.getStatus() == 0) {
    lcd.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    lcd.print(F("ERR"));
    Serial.println(F("ERR"));
    status = false;
  }
  //BME test
  lcd.setCursor(0, 1);
  lcd.print(F("BME280... "));
  Serial.print(F("BME280... "));
  delay(50);
  if (bme.begin(&Wire)) {
    lcd.print(F("OK"));
    Serial.println(F("OK"));
  } else {
    lcd.print(F("ERR"));
    Serial.println(F("ERR"));
    status = false;
  }
  // Photo test
  for (byte i = 1; i < 20; i++) { 
    lcd.setCursor(10, 1);
    lcd.print("P:    ");
    lcd.setCursor(12, 1);
    lcd.print(analogRead(PHOTO), 1);
    Serial.println(analogRead(PHOTO));
    delay(250);
  }
  // status test
  lcd.clear();
  lcd.setCursor(0, 0);
  if (status) {
    lcd.print(F("All good"));
    Serial.println(F("All good"));
  } else {
    lcd.print(F("Check wires!"));
    Serial.println(F("Check wires!"));
  }
  

  mhz19.begin(MHZ_TX, MHZ_RX);
  mhz19.setAutoCalibration(false);
  bme.begin(&Wire);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
  bme.takeForcedMeasurement();

  COOLER.tick(0) // шим офф init
  HEATER.tick(0) // шим офф init
  
}

void loop() {
  // считываем показатели
    if (testTimer(sensorsTimerD, sensorsTimer)) { // читаем и выводим показания датчиков с периодом SENS_TIME
      readSensors();
      consoleWrite();
      drawSensors();    
    }
  // САУ микроклиматом (лента, увлажнитль, осушитель, вентилятор, пожароизвещатель) T = SENS_TIME*3 
    if (testTimer(controlONTimerD, controlONTimer))
    {
      if (dispHum > HUM_MAX_THRESHOLD) digitalWrite(LED_DEHUM, HIGH); // высокая влажность
      if (dispHum < HUM_MIN_THRESHOLD) digitalWrite(LED_HUM, HIGH); // низкая влажность
      if (dispCO2 > CO2_THRESHOLD) digitalWrite(LED_VENT, HIGH); // высокий уровень углекислого газа
      if (dispSmoke > SMOKE_THRESHOLD) digitalWrite(BUZZER, HIGH); // сигнализация
      if (dispPhoto > BRIGHT_THRESHOLD) digitalWrite(LED_LED, HIGH); // темно -  включили
      else digitalWrite(LED_LED, LOW);  // светло - выключили
    }
  // САУ микроклиматом ШИМ установка заполнения шим
  if (dispTemp < TEMP_MIN_THRESHOLD) 
  {
    heatPWM = 60;
    if (dispTemp < TEMP_MIN_THRESHOLD_2) heatPWM = 120; // 0-255 (соответствует 0-100%)
    digitalWrite(LED_VENT, HIGH); // включить вентиляцию
  } 
  if (dispTemp > TEMP_MAX_THRESHOLD)
  {
    coolPWM = 60;
    if (dispTemp > TEMP_MAX_THRESHOLD_2) coolPWM = 120; // 0-255 (соответствует 0-100%)
    digitalWrite(LED_VENT, HIGH); // включить вентиляцию
  }
  // проверки на выколючение приборов
  if (testTimer(controlOFFTimerD, controlOFFTimer))
  {
    if (getPWM(coolPWM) != 0)&& (dispTemp<=tempAvarage) coolPWM = 0; // выключаем охлаждение 
    if (getPWM(heatPWM) != 0)&& (dispTemp>=tempAvarage) heatPWM = 0; // выключаем нагрев
    if (dispHum >= humAvarage) digitalWrite(LED_HUM, LOW);
    if (dispHum <= humAvarage) digitalWrite(LED_DEHUM, LOW);
    if (dispSmoke < SMOKE_THRESHOLD) digitalWrite(BUZZER, LOW);
    if (heatPWM == 0) && (coolPWM == 0) && (dispCO2 < CO2Avarage) digitalWrite(LED_VENT, LOW);
  }

  // внутри библеотеки встироенный таймер на miles, изменять таймер можно с помощью #define PWM_PEROID
  HEATER.tick(heatPWM);// ШИМ нагревателя
  COOLER.tick(coolPWM); //ШИМ охладителя
  delay(10); // небольшая задержка для предотвращения сбоев
}

