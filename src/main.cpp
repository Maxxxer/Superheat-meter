#define CLK 2
#define DIO 3
#define TEMP_SENSOR_PIN 0
#define PRESSURE_SENSOR_PIN 1
#include "TM1637.h"
#include "Arduino.h"
TM1637 disp(CLK, DIO, enTM1637Type::Time);
float Rres = 4690;
float Rtherm = 5000;
float base_div = Rres / Rtherm;
uint16_t beta = 3970;
uint8_t low_pressure_limit = 0;
float high_pressure_limit = 34.5;

void setup() {
  Serial.begin(9600);
  disp.SetBrightness(3);
  disp.Clear();
}

void loop() {

  uint16_t temp_signal = analogRead(TEMP_SENSOR_PIN);
  uint16_t pressure;

  pressure = analogRead(PRESSURE_SENSOR_PIN);
  pressure = Pressure_compute(pressure);
  float temp = NTC_compute(temp_signal, base_div, beta, 25, 10);
  // Serial.println(pressure);


  int32_t delay_time = 500;
  Show_temp(temp);
  delay(delay_time);
}


// сигнал АЦП, (R резистора / R термистора), B термистора, t термистора, разрешение АЦП
float NTC_compute(float analog, float baseDiv, uint16_t B, uint8_t t, uint8_t res) {
  float b = ((float)((1 << res) - 1) / analog - 1.0f);
  analog = baseDiv / b;
  analog = (log(analog) / B) + 1.0f / (t + 273.15f);
  return (1.0f / analog - 273.15f);
}

float Pressure_compute(float analogue) {
  uint8_t counter = 0;
  float low_limit_voltage = 0.5;  // Нижний лимит по ошибке датчика, ниже 0,5В
  float high_limit_voltage = 4.5;  // Верхний лимит по ошибке датчика, выше 4,5В
  float low_voltage_error = 0.2;  // Нижний лимит по ошибке датчика, ниже 0,5В
  float high_voltage_error = 4.8;  // Верхний лимит по ошибке датчика, выше 4,5В
  uint16_t ADC_resolution = 1024;
  uint8_t ADC_voltage = 5;
  float voltage = analogue * ADC_voltage / ADC_resolution;
  uint8_t sensing_range = 4;  // Вольтовый диапазон работы датчика 4,5 - 0,5В
  if (voltage > low_voltage_error && voltage < high_voltage_error) {
    Serial.println(analogue);
    return (voltage - low_limit_voltage) * high_pressure_limit / sensing_range;
  } else
    Serial.println("Out of range");
}



void Show_temp(uint8_t temp) {
  disp.Clear();
  disp.PrintDeg(temp);
}

void Show_pressure(uint16_t pressure) {
  disp.Clear();
  disp.Print(pressure);
  disp.ShowPoint(true);
}