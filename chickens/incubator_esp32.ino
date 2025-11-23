#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "SHT31.h"
#include <PID_v1.h>
#include <ESP32Servo.h>

// -------------------- LCD --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------------------- SENSOR --------------------
SHT31 sht;

// -------------------- PINS --------------------
#define HEATER_PIN 14
#define HUMID_PIN  27
#define SERVO_PIN  18

#define BTN_UP    32
#define BTN_DOWN  33
#define BTN_OK    25

// -------------------- VARIABLES --------------------
double tempInput, tempOutput;
double setTemp;

double humInput;
double setHumidity;

// PID settings
double kp = 2.0, ki = 5.0, kd = 1.0;
PID tempPID(&tempInput, &tempOutput, &setTemp, kp, ki, kd, DIRECT);

// mode: 0 = chicken, 1 = python
int speciesMode = 0;  

// egg turning
Servo turner;
unsigned long lastTurn = 0;
unsigned long turnInterval = 4UL * 3600000UL;   // every 4 hours

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HUMID_PIN, OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);

  sht.begin(0x44);

  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 255);

  turner.attach(SERVO_PIN);

  showStartup();
  chooseSpecies();
  loadProfile();
}

// -------------------- STARTUP --------------------
void showStartup() {
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("INCUBATOR");
  lcd.setCursor(1,1);
  lcd.print("ESP32 SYSTEM");
  delay(1200);
}

// -------------------- SPECIES CHOICE --------------------
void chooseSpecies() {
  lcd.clear();
  lcd.print("Mode?");
  delay(500);

  while (true) {
    lcd.setCursor(0,1);
    lcd.print(speciesMode == 0 ? "Chicken  " : "Python   ");

    if (!digitalRead(BTN_UP)) speciesMode = 0;
    if (!digitalRead(BTN_DOWN)) speciesMode = 1;
    if (!digitalRead(BTN_OK)) break;
    
    delay(200);
  }
}

// -------------------- LOAD PROFILES --------------------
void loadProfile() {
  if (speciesMode == 0) {
    setTemp = 37.5;
    setHumidity = 50;
  } else {
    setTemp = 32.0;
    setHumidity = 98;
  }
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // read sensor
  float t = sht.readTemperature();
  float h = sht.readHumidity();

  if (isnan(t) || isnan(h)) {
    lcd.clear();
    lcd.print("Sensor Error");
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(HUMID_PIN, LOW);
    delay(1000);
    return;
  }

  tempInput = t;
  humInput = h;

  // ---------- SAFETY CUTOFF ----------
  if ((speciesMode == 0 && t > 38.5) ||
      (speciesMode == 1 && t > 33.2)) {
    digitalWrite(HEATER_PIN, LOW);
    lcd.clear();
    lcd.print("OVERHEAT!");
    delay(1000);
    return;
  }

  // ---------- PID HEAT CONTROL ----------
  tempPID.Compute();
  analogWrite(HEATER_PIN, tempOutput);

  // ---------- HUMIDITY CONTROL ----------
  if (h < setHumidity - 2) digitalWrite(HUMID_PIN, HIGH);
  else if (h > setHumidity + 2) digitalWrite(HUMID_PIN, LOW);

  // ---------- EGG TURNING (Chicken Only) ----------
  if (speciesMode == 0) {
    unsigned long now = millis();
    if (now - lastTurn > turnInterval) {
      gentleTurn();
      lastTurn = now;
    }
  }

  // ---------- DISPLAY ----------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(t); lcd.print("C  ");
  lcd.print(h); lcd.print("%");

  lcd.setCursor(0,1);
  lcd.print("Tgt:");
  lcd.print(setTemp);
  lcd.print("  H:");
  lcd.print(setHumidity);

  delay(1000);
}

// -------------------- GENTLE TURN --------------------
void gentleTurn() {
  for (int angle = 0; angle <= 45; angle++) {
    turner.write(angle);
    delay(20);
  }
  delay(3000);
  for (int angle = 45; angle >= 0; angle--) {
    turner.write(angle);
    delay(20);
  }
}
