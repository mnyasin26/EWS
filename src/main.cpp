#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define RLY_BUZZ 5        // alarm / buzzer
#define RLY_LMP_MERAH 18  // lampu merah
#define RLY_LMP_KUNING 19 // lampu kuning
#define RLY_LMP_HIJAU 23  // lampu hijau
#define RLY_LMP_MODEM 33  // lampu modem
#define RLY6 32

#define RLY_TMA 35  // TMA
#define RLY_WIND 34 // Wind Sensor
#define RLY_RAIN 13 // Rain Sensor

#define RLY_RASP_P 13 // Raspberry Pi, Cam, Modem ()

// #define VoltagePin A0
// #define CurrentPin A1
#define VoltagePin 0
#define CurrentPin 1

SoftwareSerial mySerial(2, 3);
BH1750 lightMeter;
ModbusMaster node;

float lux;
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.6;
float Current = 0.0;
float SensorRead = 0.0;
int adc_value = 0;
float multiplier = 0.100;
int WindSpeed, WindDirection, Pressure, Temperature, Humidity, RainBucket, Distance;
int count, Status_siaga, Status_Alarm;
volatile unsigned long Drop;
String PayLoad;
String DATA[5];
uint8_t result;
uint16_t data[5];

void setup()
{
  Serial.begin(19200);
  mySerial.begin(19200);
  Wire.begin();
  lightMeter.begin();
  pinMode(RLY_BUZZ, OUTPUT);
  pinMode(RLY_LMP_MERAH, OUTPUT);
  pinMode(RLY_LMP_KUNING, OUTPUT);
  pinMode(RLY_LMP_HIJAU, OUTPUT);
  pinMode(RLY_LMP_MODEM, OUTPUT);
  pinMode(RLY6, OUTPUT);

  pinMode(RLY_TMA, OUTPUT);
  pinMode(RLY_WIND, OUTPUT);
  pinMode(RLY_RAIN, OUTPUT);
  pinMode(RLY_RASP_P, OUTPUT);

  digitalWrite(RLY_BUZZ, LOW);
  digitalWrite(RLY_LMP_MERAH, LOW);
  digitalWrite(RLY_LMP_KUNING, LOW);
  digitalWrite(RLY_LMP_HIJAU, LOW);

  digitalWrite(RLY_TMA, LOW);
  digitalWrite(RLY_WIND, LOW);
  digitalWrite(RLY_RAIN, LOW);

  digitalWrite(RLY_RASP_P, LOW);

  Serial.println("START");
  node.begin(2, mySerial);
  node.setTransmitBuffer(0, 0);
  result = node.writeMultipleRegisters(0, 1);
}

void (*resetFunc)(void) = 0;

void loop()
{
  if ((millis() - Drop) > 700)
  {
    lux = lightMeter.readLightLevel();

    adc_value = analogRead(VoltagePin);
    adc_voltage = (adc_value * ref_voltage) / 1024.0;
    in_voltage = adc_voltage / (R2 / (R1 + R2));

    SensorRead = analogRead(CurrentPin) * (5.0 / 1023.0); // We read the sensor output
    Current = (SensorRead - 2.5) / multiplier;            // Calculate the current value
    Current = Current * -1;
    Current = Current * 1000;

    Drop = millis();
  }
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '*')
    {
      if (DATA[0] == "0")
      {
        Status_siaga = 0;
        digitalWrite(RLY_LMP_MERAH, LOW);
        digitalWrite(RLY_LMP_KUNING, LOW);
        digitalWrite(RLY_LMP_HIJAU, LOW);
      }
      else if (DATA[0] == "1")
      {
        Status_siaga = 1;
        digitalWrite(RLY_LMP_MERAH, HIGH);
        digitalWrite(RLY_LMP_KUNING, LOW);
        digitalWrite(RLY_LMP_HIJAU, LOW);
      }
      else if (DATA[0] == "2")
      {
        Status_siaga = 2;
        digitalWrite(RLY_LMP_MERAH, LOW);
        digitalWrite(RLY_LMP_KUNING, HIGH);
        digitalWrite(RLY_LMP_HIJAU, LOW);
      }
      else if (DATA[0] == "3")
      {
        Status_siaga = 3;
        digitalWrite(RLY_LMP_MERAH, LOW);
        digitalWrite(RLY_LMP_KUNING, LOW);
        digitalWrite(RLY_LMP_HIJAU, HIGH);
      }
      else if (DATA[0] == "reset")
      {
        Serial.println("RESETTING...");
        delay(500);
        resetFunc();
      }
      else if (DATA[0] == "REQ")
      {
        node.begin(1, mySerial); // slave1
        result = node.readHoldingRegisters(0, 5);
        if (result == node.ku8MBSuccess)
        {
          WindSpeed = node.getResponseBuffer(0);
          WindDirection = node.getResponseBuffer(1);
          Temperature = node.getResponseBuffer(2);
          Humidity = node.getResponseBuffer(3);
          Pressure = node.getResponseBuffer(4);
          delay(10);
        }
        node.begin(2, mySerial);
        result = node.readHoldingRegisters(0, 1);

        if (result == node.ku8MBSuccess)
        {
          RainBucket = node.getResponseBuffer(0);
          delay(10);
        }

        node.begin(3, mySerial);
        result = node.readHoldingRegisters(0, 3);

        if (result == node.ku8MBSuccess)
        {
          Distance = node.getResponseBuffer(0);
          delay(10);
        }
        Serial.println(String() + Temperature + "," + Humidity + "," + Pressure + "," + WindDirection + "," + WindSpeed + "," + Distance + "," + RainBucket + "," + lux + "," + String(Current, 2) + "," + String(in_voltage, 2) + "," + Status_siaga + "," + Status_Alarm + ",*");
      }
      else if (DATA[0] == "BCKT")
      {
        node.begin(2, mySerial);
        node.setTransmitBuffer(0, 0);
        result = node.writeMultipleRegisters(0, 1);
      }
      if (DATA[1] == "1")
      {
        Status_Alarm = 1;
        digitalWrite(RLY_BUZZ, HIGH);
      }
      else
      {
        Status_Alarm = 0;
        digitalWrite(RLY_BUZZ, LOW);
      }
      if (DATA[2] == "1")
      {
        digitalWrite(RLY_LMP_MODEM, HIGH);
      }
      else
      {
        digitalWrite(RLY_LMP_MODEM, LOW);
      }

      PayLoad = "";

      count = 0;
    }
    else if (c == ',')
    {
      PayLoad.trim();

      DATA[count] = PayLoad;
      count++;
      PayLoad = "";
    }
    else
    {
      PayLoad += c;
    }
    delay(1);
  }
}
