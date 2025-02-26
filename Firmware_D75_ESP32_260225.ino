//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2);  // Change to 0x27 if address is different

//Thermistor needed libraries
#include <thermistor.h>           // Ensure this library is compatible with ESP32
thermistor therm1(34, 0);         // Connect thermistor to GPIO 34 (Analog pin on ESP32)

//I/O
int PWM_pin = 5;                  // GPIO 5 for PWM signal to the MOSFET driver
int speed_pot = 35;               // GPIO 35 for speed potentiometer (Analog pin on ESP32)
int but1 = 23;                    // GPIO 23 for button
int EN = 22;                      // GPIO 22 for stepper enable
int STEP = 18;                    // GPIO 18 for stepper step
int DIR = 19;                     // GPIO 19 for stepper direction
int LED = 2;                      // GPIO 2 for LED (built-in LED on ESP32)

//Variables
float set_temperature = 200;      // Default temperature setpoint
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated = 0;
float last_set_temperature = 0;
int max_PWM = 255;

//Stepper Variables
int max_speed = 1000;
int main_speed = 0;
bool but1_state = true;
bool activate_stepper = false;
int rotating_speed = 0;

#include <AccelStepper.h>
// Define a stepper and the pins it will use
AccelStepper stepper1(AccelStepper::DRIVER, STEP, DIR);

//PID constants
int kp = 90;   int ki = 30;   int kd = 80;

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed = 0;

// Timer for stepper motor
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  stepper1.runSpeed();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);     // Stepper driver is disabled
  stepper1.setMaxSpeed(max_speed);
  pinMode(but1, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(PWM_pin, OUTPUT);
  ledcSetup(0, 7812, 8);      // Configure PWM channel 0, 7812 Hz, 8-bit resolution
  ledcAttachPin(PWM_pin, 0);  // Attach PWM pin to channel 0

  Time = millis();

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize timer for stepper motor
  timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1 MHz)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);  // 1 ms interval
  timerAlarmEnable(timer);
}

void loop() {
  if (!digitalRead(but1) && but1_state) {
    but1_state = false;
    activate_stepper = !activate_stepper;
    delay(10);
  } else if (digitalRead(but1) && !but1_state) {
    but1_state = true;
  }

  if (activate_stepper) {
    digitalWrite(LED, HIGH);
    digitalWrite(EN, LOW);    // Activate stepper driver
    rotating_speed = map(analogRead(speed_pot), 0, 4095, main_speed, max_speed);  // ESP32 ADC is 12-bit
    stepper1.setSpeed(rotating_speed);
  } else {
    digitalWrite(EN, HIGH);    // Deactivate stepper driver
    digitalWrite(LED, LOW);
    stepper1.setSpeed(0);
  }

  // Read temperature
  temperature_read = therm1.analog2temp();

  // Calculate PID error
  PID_error = set_temperature - temperature_read + 6;
  PID_p = 0.01 * kp * PID_error;
  PID_i = 0.01 * PID_i + (ki * PID_error);

  // Calculate derivative
  timePrev = Time;
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000;
  PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);
  PID_value = PID_p + PID_i + PID_d;

  // Limit PID value
  if (PID_value < 0) PID_value = 0;
  if (PID_value > max_PWM) PID_value = max_PWM;

  // Write PWM signal
  ledcWrite(0, PID_value);  // Use ledcWrite for PWM on ESP32
  previous_error = PID_error;

  // Update LCD
  delay(250);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(temperature_read, 1);
  lcd.print(" S: ");
  lcd.print(rotating_speed);

  lcd.setCursor(0, 1);
  lcd.print("PID: ");
  lcd.print(PID_value);
}