#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS

// Pines
#define ENCODER_CLK 2
#define ENCODER_DT 3
#define ENCODER_SW 4
#define SENSOR_PIN A0
#define HEATER_PIN 9

#define dirPin 5
#define stepPin 6
#define dir2Pin 10
#define step2Pin 11

AccelStepper stepper1(AccelStepper::DRIVER, stepPin, dirPin);
AccelStepper stepper2(AccelStepper::DRIVER, step2Pin, dir2Pin);

LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 1000; // 1 segundo

// Encoder
RotaryEncoder encoder(ENCODER_DT, ENCODER_CLK, RotaryEncoder::LatchMode::FOUR3);
int lastButtonState = HIGH;
int lastPos = 0;
bool settingTemp = true;

// Variables principales
double temp = 0;
int targetTemp = 160;
int rpm = 100;

// PID
double inputTemp, outputPWM, setpointTemp = 160;
double Kp = 35.0, Ki = 0.5, Kd = 0.0;
PID myPID(&inputTemp, &outputPWM, &setpointTemp, Kp, Ki, Kd, DIRECT);

// Debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Buffers para detectar cambios
double lastTemp = -1.0;
int lastTargetTemp = -1;
int lastRpm = -1;
bool lastSettingTemp = true;

// Mapeo temperatura-voltaje
float temps[] = {
  20, 110, 115, 120, 125, 130, 135, 140, 145,
  150, 155, 160, 165, 170, 175, 180, 185, 
  190, 195, 200, 205, 210, 215, 220, 225, 230
};

float voltages[] = {
  3.30, 2.94, 2.92, 2.89, 2.87, 2.85, 2.83, 2.79, 2.76,
  2.70, 2.63, 2.39, 2.19, 1.96, 1.79, 1.61, 1.45,
  1.31, 1.20, 1.07, 0.97, 0.88, 0.79, 0.72, 0.65, 0.59
};

float voltageToTemperature(float voltage) {
  int numPoints = sizeof(temps) / sizeof(temps[0]);
  for (int i = 0; i < numPoints - 1; i++) {
    if (voltage <= voltages[i] && voltage >= voltages[i + 1]) {
      float slope = (temps[i + 1] - temps[i]) / (voltages[i + 1] - voltages[i]);
      return temps[i] + slope * (voltage - voltages[i]);
    }
  }
  return temps[numPoints - 1];
}

void setup() {
  Serial.begin(115200);          // Serial rápido
  Wire.begin();
  Wire.setClock(400000);         // I2C a 400kHz para acelerar LCD

  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), checkEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), checkEncoder, CHANGE);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  stepper1.setMaxSpeed(2000);
  stepper2.setMaxSpeed(2000);

  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(HEATER_PIN, OUTPUT);

  encoder.setPosition(0);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Extrusora Init OK");
  delay(1000);
  lcd.clear();
}

void loop() {
  checkEncoder();
  stepper1.setSpeed(-300);
  stepper1.run();
  stepper2.setSpeed(-300);
  stepper2.run();

  int buttonState = digitalRead(ENCODER_SW);
  unsigned long currentTime = millis();
  if (buttonState == LOW && lastButtonState == HIGH && (currentTime - lastDebounceTime > debounceDelay)) {
    settingTemp = !settingTemp;
    Serial.print("Modo: ");
    Serial.println(settingTemp ? "Temperatura" : "RPM");
    lastDebounceTime = currentTime;
  }
  lastButtonState = buttonState;
  stepper1.run();
  // Leer posición del encoder
  int newPos = encoder.getPosition();
  if (newPos != lastPos) {
    int delta = newPos - lastPos;
    if (settingTemp) {
      targetTemp += delta;
      if (targetTemp < 0) targetTemp = 0;
    } else {
      rpm += delta * 10;
      if (rpm < 0) rpm = 0;
      // Aquí mover velocidad stepper si quieres
    }
    lastPos = newPos;
  }
  stepper1.run();
  // Leer voltaje y convertir a temperatura
  int rawADC = analogRead(SENSOR_PIN);
  float voltage = rawADC * (5.0 / 1023.0);
  temp = voltageToTemperature(voltage);
  inputTemp = temp;
  setpointTemp = targetTemp;

  // PID
  myPID.Compute();
  analogWrite(HEATER_PIN, (int)outputPWM);
  stepper1.run();
  if (currentTime - lastLcdUpdate >= lcdUpdateInterval) {
  bool changed = false;

  if (abs(temp - lastTemp) > 0.1) {
    lastTemp = temp;
    changed = true;
  }
  if (targetTemp != lastTargetTemp) {
    lastTargetTemp = targetTemp;
    changed = true;
  }
  if (rpm != lastRpm) {
    lastRpm = rpm;
    changed = true;
  }
  if (settingTemp != lastSettingTemp) {
    lastSettingTemp = settingTemp;
    changed = true;
  }

  if (changed) {
    // Evitar lcd.clear() para que no parpadee
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temp, 1);
    lcd.print("C O:");
    lcd.print(targetTemp);
    lcd.print("C   ");  // Espacios para limpiar resto

    lcd.setCursor(0, 1);
    lcd.print(settingTemp ? "Modo:T " : "Modo:R ");
    lcd.print("RPM:");
    lcd.print(rpm);
    lcd.print("   ");  // Espacios para limpiar resto

    // Serial
    //Serial.print("Temp: ");
    Serial.print(temp, 1);
    Serial.print(", ");
    Serial.println(targetTemp);
    //Serial.print(" C | RPM: ");
    //Serial.println(rpm);
  }

  lastLcdUpdate = currentTime;
}
  checkEncoder();
  stepper1.run();
  stepper2.run();
}

void checkEncoder() {
  encoder.tick();  // obligatorio si usas interrupciones
}
