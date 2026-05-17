#include <LiquidCrystal.h>
#include <Wire.h>
#include "MAX30105.h"
#include <RTClib.h>

unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 60000;

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

RTC_DS3231 rtc; 
MAX30105 particleSensor;
LiquidCrystal lcdTop(52, 50, 48, 46, 44, 42);
LiquidCrystal lcdBottom(53, 51, 49, 47, 45, 43);

volatile bool deviceActive = false; 
volatile unsigned long lastInterruptTime = 0;

float spo2Array[5] = {0,0,0,0,0};
int spo2Index = 0, spo2Count = 0;
float avgSpO2 = 0;
bool BODetected = false;

float axillaryArray[5] = {0,0,0,0,0};
int axIndex = 0, axCount = 0;
float avgAxillary = 0;
const int thermistorPin = A0;

const int B_COEFFICIENT = 3435; 
const int SERIES_RESISTOR = 10000;
const int THERMISTOR_NOMINAL = 10000;
const int TEMPERATURE_NOMINAL = 25;

void UART_sendString(const char* str);
void readSpO2();
void readAxillary();
void handleVitalsDisplay();
void logicOFF();
void logicMonitoring();
void powerToggleISR();


void setup() {

  UBRR0H = (unsigned char)(MYUBRR >> 8);
  UBRR0L = (unsigned char)MYUBRR;
  UCSR0B = (1 << TXEN0); 
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 

  if (!rtc.begin()) UART_sendString("RTC Error\r\n");
  
  ADMUX = (1 << REFS0); 
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  DDRG |= (1 << DDG5);
  DDRC |= (1 << DDC1) | (1 << DDC5) | (1 << DDC7);
  DDRA |= (1 << DDA4); 
  DDRH |= (1 << DDH6); 

  lcdTop.begin(16, 2);
  lcdBottom.begin(16, 2);
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    lcdBottom.print("SENSOR ERROR");
    while (1); 
  }
  
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x1F); 
  particleSensor.setPulseAmplitudeIR(0x1F); 

  DDRE &= ~((1 << DDE4) | (1 << DDE5)); 
  
  PORTE |= (1 << PORTE4) | (1 << PORTE5);

  attachInterrupt(digitalPinToInterrupt(2), powerToggleISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), resetISR, FALLING);
  
  sei();
}

void loop() {

  readSpO2();
  readAxillary();

  if (deviceActive) {
    logicMonitoring(); 
  } else {
    logicOFF();
  }

  static unsigned long lastSecond = 0;
  if (millis() - lastSecond >= 1000) {
    handleVitalsDisplay(); 
    lastSecond = millis();
  }

  static unsigned long lastMinute = 0;
  if (millis() - lastMinute >= 60000) {
    logToSerial();
    lastMinute = millis();
  }
}

void logToSerial() {
  DateTime now = rtc.now();

  UART_sendString("[");
  UART_sendInt(now.hour());
  UART_sendString(":");
  if(now.minute() < 10) UART_sendString("0");
  UART_sendInt(now.minute());
  UART_sendString(":");
  if(now.second() < 10) UART_sendString("0");
  UART_sendInt(now.second());
  UART_sendString("] ");

  UART_sendString("STATE: ");
  if (!deviceActive) UART_sendString("OFF");
  else if (avgAxillary < 27.0 || avgSpO2 < 80.0) UART_sendString("ALARM");
  else if (avgAxillary <= 30.0 || avgSpO2 <= 85.0) UART_sendString("HEATING");
  else UART_sendString("NORMAL");

  UART_sendString(" | Temp: ");
  UART_sendFloat(avgAxillary);
  UART_sendString("C | SpO2: ");
  UART_sendFloat(avgSpO2);
  UART_sendString("%\r\n");
}

void powerToggleISR() {
  unsigned long now = millis();
  if (now - lastInterruptTime > 250) { 
    deviceActive = !deviceActive;
    lastInterruptTime = now;
  }
}

void resetISR() {

  TCCR2A &= ~(1 << COM2B1); 
  OCR2B = 0;

  PORTC |= (1 << PC1) | (1 << PC5) | (1 << PC7); 
  PORTA |= (1 << PA4);

  for (int i = 0; i < 5; i++) {
    axillaryArray[i] = 0;
    spo2Array[i] = 0;
  }
  axIndex = 0;
  spo2Index = 0;
  
  deviceActive = false;
}

void logicOFF() {
  setHeaterPower(0); 
  PORTG &= ~(1 << PG5); 
  PORTC |= (1 << PC5);
  PORTC &= ~((1 << PC1) | (1 << PC7));
  PORTA &= ~(1 << PA4);

  lcdBottom.setCursor(0, 1);
  lcdBottom.print("STANDBY MODE    ");
}
void logicMonitoring() {
  PORTG |= (1 << PG5); 
  if (avgAxillary < 27.0 || avgSpO2 < 80.0) {
    setHeaterPower(0); 
    PORTC |= (1 << PC7);
    PORTC &= ~((1 << PC1) | (1 << PC5));
    PORTA &= ~(1 << PA4);

    lcdBottom.setCursor(0, 0);
    lcdBottom.print("Device: On");
    lcdBottom.setCursor(0, 1);
    lcdBottom.print("No Patient     ");
  } 
  else if (avgAxillary <= 30.0 || avgSpO2 <= 85.0) {
    setHeaterPower(128);
    PORTA |= (1 << PA4); 
    PORTC &= ~((1 << PC1) | (1 << PC5) | (1 << PC7));

    lcdBottom.setCursor(0, 0);
    lcdBottom.print("Device: On");
    lcdBottom.setCursor(0, 1);
    lcdBottom.print("STATUS: CRITICAL");
  } 
  else {
    setHeaterPower(0);
    PORTC |= (1 << PC1); 
    PORTC &= ~((1 << PC5) | (1 << PC7));
    PORTA &= ~(1 << PA4);

    lcdBottom.setCursor(0, 0);
    lcdBottom.print("Device: On");
    lcdBottom.setCursor(0, 1);
    lcdBottom.print("STATUS: NORMAL");
  }
}

void readSpO2() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  if (irValue < 20000) { 
    BODetected = false;
  } else {
    BODetected = true;
    float ratio = (float)redValue / (float)irValue;
    float currentReading = 110.0 - (25.0 * ratio); 
    if (currentReading > 100) currentReading = 100;
    
    spo2Array[spo2Index] = currentReading;
    spo2Index = (spo2Index + 1) % 5; 
    if (spo2Count < 5) spo2Count++;

    float sum = 0;
    for (int i = 0; i < 5; i++) sum += spo2Array[i];
    avgSpO2 = sum / 5.0;
  }
}
void readAxillary() {
  ADCSRA |= (1 << ADSC); 
  while (ADCSRA & (1 << ADSC)); 
  
  int rawValue = 1023 - ADC; 

  float currentTemp = 0.0;
  
  if (rawValue <= 5) { 
    currentTemp = 0.0; 
  } 
  else if (rawValue >= 1020) {
    currentTemp = 99.9; 
  }
  else {
    float resistance = 10000.0 / ((1023.0 / (float)rawValue) - 1.0);
    float steinhart = log(resistance / 10000.0) / 3435.0;
    steinhart += 1.0 / (25.0 + 273.15);
    currentTemp = (1.0 / steinhart) - 273.15;
  }

  axillaryArray[axIndex] = currentTemp;
  axIndex = (axIndex + 1) % 5;
}

void handleVitalsDisplay() {
  float tempSum = 0;
  for (int i = 0; i < 5; i++) tempSum += axillaryArray[i];
  avgAxillary = tempSum / 5.0;

  float o2Sum = 0;
  for (int i = 0; i < 5; i++) o2Sum += spo2Array[i];
  avgSpO2 = o2Sum / 5.0;

  lcdTop.setCursor(0, 0);
  lcdTop.print("Temp: ");
  lcdTop.print(avgAxillary, 1); 
  lcdTop.print(" C      "); 

  lcdTop.setCursor(0, 1);
  lcdTop.print("SpO2: ");
  lcdTop.print(avgSpO2, 0);
  lcdTop.print(" %      ");
}

void UART_transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void UART_sendString(const char* str) {
  while (*str) UART_transmit(*str++);
}

void UART_sendInt(int num) {
  char buf[10];
  itoa(num, buf, 10);
  UART_sendString(buf);
}

void UART_sendFloat(float num) {
  char buf[10];
  dtostrf(num, 4, 1, buf);
  UART_sendString(buf);
}

void setHeaterPower(uint8_t duty) {
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS22);
  OCR2B = duty;
}

