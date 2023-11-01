#include "DHT.h"
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <time.h>

#define DEFAULT_INIT_DUST 0.5
#define DHT_SENSOR_DIGITAL_PIN 10
#define DHT_SENSOR_MODEL DHT22
#define BLUETOOTH_SERIAL_RXD_PIN 12
#define BLUETOOTH_SERIAL_TXD_PIN 13

const uint8_t motor_input_pin1 = 2;
const uint8_t motor_input_pin2 = 3;
const uint8_t motor_enable_pin = 11;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
DHT dhtSensor(DHT_SENSOR_DIGITAL_PIN,
              DHT_SENSOR_MODEL);
SoftwareSerial bluetoothSerial(BLUETOOTH_SERIAL_RXD_PIN,
                               BLUETOOTH_SERIAL_TXD_PIN);

int g_dustSensorPinNum = A4;
int g_dustSensorLedPinNum = A5;
int g_is_windowOpen = 0;
int g_is_windowClose = 0;
int g_defaultMotorPower = 100;
int g_set_automatic = 0;
int Debug = 1;

float g_dustSensorValue = 0;
float g_dustSensorVoltage = 0;
float g_dustDensity = 0;

unsigned long g_asyncStartTime;
unsigned long g_asyncEndTime;
unsigned long g_asyncStartTime2;
unsigned long g_asyncEndTime2;
unsigned long g_halfOpenMsec;

void openTheWindow();
void closeTheWindow();
void halfOpenTheWindow();
void halfOpenSetting();
void bluetoothCommand(String cmd);
void (*resetFunc)(void) = 0;
float get_dustSensorVoltage(float dustSensorValue);
float get_dustDensity(float dustSensorVoltage);
void openMotion();
void closeMotion();
void stopMotion();
bool check_bluetoothSignal();
void show_allStatus();
void serialCommand(String cmd);

void setup()
{
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  Serial.setTimeout(10);
  bluetoothSerial.setTimeout(10);
  dhtSensor.begin();
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(g_dustSensorLedPinNum, OUTPUT);
  pinMode(motor_input_pin1, OUTPUT);
  pinMode(motor_input_pin2, OUTPUT);
  pinMode(motor_enable_pin, OUTPUT);
  g_is_windowClose = !analogRead(A1);
  g_is_windowOpen = !analogRead(A2);
  halfOpenSetting();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.clear();
}

void loop()
{
  g_is_windowClose = !analogRead(A1);
  g_is_windowOpen = !analogRead(A2);
  delay(500);

  if (bluetoothSerial.available())
  {
    String data = bluetoothSerial.readString();
    Serial.println(data);
    bluetoothCommand(data);
  }
  digitalWrite(g_dustSensorLedPinNum, LOW);
  delayMicroseconds(280);
  g_dustSensorValue = analogRead(g_dustSensorPinNum);
  delayMicroseconds(40);
  digitalWrite(g_dustSensorLedPinNum, HIGH);
  delayMicroseconds(9600);
  lcd.clear();

  g_dustSensorVoltage = get_dustSensorVoltage(g_dustSensorValue);
  g_dustDensity = get_dustDensity(g_dustSensorVoltage);
  int integer_dustDensity = g_dustDensity;

  lcd.setCursor(0, 1);
  lcd.print("Dust:");
  lcd.setCursor(5, 1);
  lcd.print(integer_dustDensity);
  lcd.setCursor(8, 1);
  lcd.print("[ug/m^3]");

  float humidity = dhtSensor.readHumidity();
  float temperature = dhtSensor.readTemperature();
  float fahrenheit = dhtSensor.readTemperature(true);
  float show_lcdTemperature = dhtSensor.computeHeatIndex(temperature, humidity, false);

  lcd.setCursor(0, 0);
  lcd.print("temp:");
  lcd.setCursor(5, 0);
  lcd.print(show_lcdTemperature);
  lcd.setCursor(7, 0);
  lcd.print("C ");

  lcd.setCursor(9, 0);
  lcd.print("Hum:");
  lcd.setCursor(13, 0);
  lcd.print(humidity);
  lcd.setCursor(15, 0);
  lcd.print("%");

  if (Debug == 1 && Serial.available())
  {
    String data = Serial.readString();
    serialCommand(data);
  }

  bluetoothSerial.print((String)temperature +
                        "and" +
                        (String)humidity +
                        "and" +
                        (String)g_dustDensity);
  if (g_set_automatic)
  {
    if (humidity >= 75 ||
        g_dustDensity > 600)
    {
      closeTheWindow();
    }
    else if (temperature >= 30)
    {
      openTheWindow();
    }
    else if (g_dustDensity >= 100 &&
             g_dustDensity <= 300)
    {
      halfOpenTheWindow();
    }
  }
}

float get_dustSensorVoltage(float dustSensorValue)
{
  float returnValue = dustSensorValue * 5.0 / 1024;
  return returnValue;
}

float get_dustDensity(float dustSensorVoltage)
{
  float returnValue = (dustSensorVoltage -
                       DEFAULT_INIT_DUST) /
                      0.005;
  return returnValue;
}

void openTheWindow()
{
  g_is_windowOpen = !analogRead(A2);
  if (g_is_windowOpen)
  {
    stopMotion();
  }
  else
  {
    g_asyncStartTime = millis();
    openMotion();

    while (1)
    {
      g_is_windowOpen = !analogRead(A2);
      g_asyncEndTime = millis();
      if (g_asyncEndTime - g_asyncStartTime > 2000)
      {
        stopMotion();
        break;
      }
      else
      {
        if (g_is_windowOpen)
        {
          g_asyncStartTime2 = millis();
          while (1)
          {
            g_is_windowOpen = !analogRead(A2);
            g_asyncEndTime2 = millis();
            if (g_asyncEndTime2 - g_asyncStartTime2 > 500 &&
                g_is_windowOpen)
            {
              stopMotion();
              break;
            }
          }
          break;
        }
        if (bluetoothSerial.available()){
          stopMotion();
          bluetoothCommand(bluetoothSerial.readString());
          break;
        }
      }
    }
  }
}

void closeTheWindow()
{
  g_is_windowClose = !analogRead(A1);
  if (g_is_windowClose)
  {
    stopMotion();
  }
  else
  {
    g_asyncStartTime = millis();
    closeMotion();

    while (1)
    {
      g_is_windowClose = !analogRead(A1);
      g_asyncEndTime = millis();
      if (g_asyncEndTime - g_asyncStartTime > 2000)
      {
        stopMotion();
        break;
      }
      else
      {
        if (g_is_windowClose)
        {
          g_asyncStartTime2 = millis();
          while (1)
          {
            g_is_windowClose = !analogRead(A1);
            g_asyncEndTime2 = millis();
            if (g_asyncEndTime2 - g_asyncStartTime2 > 500 &&
                g_is_windowClose)
            {
              stopMotion();
              break;
            }
          }
          break;
        }
        if (bluetoothSerial.available()){
          stopMotion();
          bluetoothCommand(bluetoothSerial.readString());
          break;
        }
      }
    }
  }
}

void halfOpenTheWindow()
{
  if (!g_is_windowClose && !g_is_windowOpen)
  {
    stopMotion();
  }
  else
  {
    if (g_is_windowClose)
    {
      g_asyncStartTime = millis();
      openMotion();
      while (1)
      {
        g_asyncEndTime = millis();
        if (g_asyncEndTime - g_asyncStartTime > g_halfOpenMsec)
        {
          stopMotion();
          break;
        }
        else if (bluetoothSerial.available()){
          stopMotion();
          bluetoothCommand(bluetoothSerial.readString());
          break;
        }
      }
    }
    if (g_is_windowOpen)
    {
      g_asyncStartTime = millis();
      closeMotion();
      while (1)
      {
        g_asyncEndTime = millis();
        if (g_asyncEndTime - g_asyncStartTime > g_halfOpenMsec)
        {
          stopMotion();
          break;
        }
        else if (bluetoothSerial.available()){
          stopMotion();
          bluetoothCommand(bluetoothSerial.readString());
          break;
        }
      }
    }
  }
}

void bluetoothCommand(String cmd)
{
  if (cmd == "open")
  {
    g_set_automatic = 0;
    openTheWindow();
  }
  else if (cmd == "close")
  {
    g_set_automatic = 0;
    closeTheWindow();
  }
  else if (cmd == "half")
  {
    g_set_automatic = 0;
    halfOpenTheWindow();
  }
  else if (cmd == "automatic")
  {
    g_set_automatic = 1;
  }

  else if (cmd == "passive")
  {
    g_set_automatic = 0;
  }

  else if (cmd == "반만 열어 줘 참깨" ||
           cmd == "반만 열어 줘" ||
           cmd == "창문 반만 열어 줘" ||
           cmd == "창문 반만 닫아 줘" ||
           cmd == "반만 닫아 줘")
  {
    halfOpenTheWindow();
  }

  else if (cmd == "열려라 참깨" ||
           cmd == "창문 열어 줘" ||
           cmd == "열어 줘")
  {
    openTheWindow();
  }
  else if (cmd == "닫혀라 참깨" ||
           cmd == "다쳐라 참깨" ||
           cmd == "창문 닫아 줘" ||
           cmd == "닫아 줘")
  {
    closeTheWindow();
  }
}

void halfOpenSetting()
{
  unsigned long startTime, endTime;
  int num1_setting = 0;
  int num2_setting = 0;
  g_is_windowClose = !analogRead(A1);
  g_is_windowOpen = !analogRead(A2);
  if (g_is_windowOpen ||
      !g_is_windowOpen && !g_is_windowClose){
    closeMotion();
    startTime = millis();
    while(1){
      endTime = millis();
      if (endTime - startTime > 5000 &&
          !g_is_windowClose){
        stopMotion();
        break;
      }
      else{
        g_is_windowClose = !analogRead(A1);
        g_is_windowOpen = !analogRead(A2);
        if (g_is_windowClose){
          num1_setting = 1;
          break;
        }
      }
    }
  }
  if(g_is_windowClose && num1_setting){
    g_asyncStartTime = millis();
    startTime = millis();
    openMotion();
    while(1){
      endTime = millis();
      if (endTime - startTime > 5000){
        stopMotion();
        break;
      }
      else{
        g_is_windowClose = !analogRead(A1);
        g_is_windowOpen = !analogRead(A2);
        if (g_is_windowOpen){
          g_asyncEndTime = millis();
          g_halfOpenMsec = (g_asyncEndTime - g_asyncStartTime) / 2;
          delay(200);
          stopMotion();
          break;
        }
      }
    }
  }
  if(g_is_windowClose &&
    !num1_setting &&
    !num2_setting){
      g_asyncStartTime = millis();
      startTime = millis();
      openMotion();
      while(1){
        endTime = millis();
        if(endTime - startTime > 5000){
          stopMotion();
          break;
        }
        else{
          g_is_windowClose = !analogRead(A1);
          g_is_windowOpen = !analogRead(A2);
          if (g_is_windowOpen){
            g_asyncEndTime = millis();
            g_halfOpenMsec = (g_asyncEndTime - g_asyncStartTime) / 2;
            delay(200);
            stopMotion();
            break;

          }
        }
      }
    }
}

void openMotion()
{
  digitalWrite(motor_input_pin1, HIGH);
  digitalWrite(motor_input_pin2, LOW);
  analogWrite(motor_enable_pin,
              g_defaultMotorPower);
}

void closeMotion()
{
  digitalWrite(motor_input_pin1, LOW);
  digitalWrite(motor_input_pin2, HIGH);
  analogWrite(motor_enable_pin,
              g_defaultMotorPower);
}

void stopMotion()
{
  digitalWrite(motor_input_pin1, LOW);
  digitalWrite(motor_input_pin2, LOW);
  analogWrite(motor_enable_pin, 0);
}

bool check_bluetoothSignal()
{
  if (bluetoothSerial.available())
  {
    String data = bluetoothSerial.readString();
    if (data)
    {
      bluetoothCommand(data);
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

void serialCommand(String cmd)
{
}

void show_allStatus()
{
  Serial.println("Debug Start!");
  Serial.println("Input Variable Name");
}
