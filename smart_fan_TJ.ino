// Smart Auto-Fan with Proximity and Speed Control
// Board: Arduino UNO R4 WiFi
//
// Modes (press the button to cycle):
//   OFF    - fan stopped
//   AUTO   - fan turns on when someone is within range and spins faster the closer they get
//            (optional: also speeds up when the room is warm, if the thermistor is connected)
//   MANUAL - fan speed set by the potentiometer
//
// Power: the power module (both jumpers on 5V, 9V battery) feeds BOTH breadboard rails.
//        The Arduino runs from USB and shares only GND with the breadboard.
//        Do NOT connect the Arduino's 5V pin to the rails.
//
// Wiring summary (see the wiring guide for exact holes):
//   Ultrasonic HC-SR04 : VCC/GND to top rails, TRIG D9, ECHO D10
//   OLED SSD1306 (I2C) : VCC/GND to top rails, SDA, SCL
//   Button             : D2 to one leg, diagonal leg to GND (uses internal pull-up)
//   Potentiometer      : middle pin A0, outer pins to top + and top - rails
//   LEDs (each with 220 ohm resistor): green D4, yellow D7, red D8
//   L293D motor driver : pin 1 (EN) D5 for PWM speed,
//                        pin 2 (IN1) tied to + rail and pin 7 (IN2) tied to - rail (fixed direction),
//                        pins 3 and 6 to the motor, pins 8 and 16 to + rails, pins 4 and 13 to GND
//   Thermistor: top + rail (40a) -> 10k resistor 40c-37c -> column 37 (A1) -> thermistor 37d-34d -> top - rail (34a)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pins ----------
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int BUTTON_PIN = 2;
const int POT_PIN = A0;
const int THERM_PIN = A1;
const int MOTOR_EN = 5;  // L293D pin 1, PWM speed control
// L293D pins 2 and 7 (direction) are wired straight to the rails, so no Arduino pins are needed
const int GREEN_LED_PIN = 4;
const int YELLOW_LED_PIN = 7;
const int RED_LED_PIN = 8;

// ---------- Settings you can tune ----------
const bool USE_THERMISTOR = true;  // set to false if the thermistor is not wired to A1
const float NEAR_CM = 15.0;        // at or closer than this: full speed
const float FAR_CM = 60.0;         // someone closer than this turns the fan on
const float OFF_CM = 70.0;         // fan turns off only after they move past this (prevents flickering)
const int MIN_RUN_PCT = 60;        // below this the motor stalls (the L293D eats ~1.5 V); raise it if the fan hums but won't spin
const float TEMP_START_C = 26.0;   // AUTO mode: fan starts helping above this temperature
const int RAMP_STEP_PCT = 4;       // how fast speed changes per loop (smooth ramping)

// ---------- Display ----------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ---------- State ----------
enum Mode { MODE_OFF,
            MODE_AUTO,
            MODE_MANUAL };
Mode mode = MODE_AUTO;
const char* modeNames[] = { "OFF", "AUTO", "MANUAL" };

float distanceCm = 999;  // smoothed distance
float tempC = NAN;
int targetPct = 0;   // speed we want
int currentPct = 0;  // speed the motor is actually at (ramps toward target)
bool autoRunning = false;

int lastButton = HIGH;
unsigned long lastPressMs = 0;

// ---------- Sensors ----------

// One ultrasonic reading in cm, or -1 if no echo came back (nothing in range)
float readDistanceOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, 30000UL);  // 30 ms timeout (~5 m)
  if (echoUs == 0) return -1;
  return echoUs * 0.0343 / 2.0;  // speed of sound 343 m/s, there and back
}

void updateDistance() {
  float d = readDistanceOnce();
  if (d < 0 || d > 400) d = 400;            // treat "no echo" as far away
  distanceCm = 0.7 * distanceCm + 0.3 * d;  // smooth out jumpy readings
}

// Thermistor: 10k NTC, Beta 3950, wired 5V -> 10k -> A1 -> thermistor -> GND
void updateTemperature() {
  if (!USE_THERMISTOR) return;
  int adc = analogRead(THERM_PIN);
  if (adc <= 0 || adc >= 1023) {
    tempC = NAN;
    return;
  }  // not connected
  float rTherm = 10000.0 * adc / (1023.0 - adc);
  float kelvin = 1.0 / (1.0 / 298.15 + log(rTherm / 10000.0) / 3950.0);
  tempC = kelvin - 273.15;
}

// ---------- Speed logic ----------

int autoSpeed() {
  // Turn on when someone comes within FAR_CM, off only once they pass OFF_CM
  if (!autoRunning && distanceCm <= FAR_CM) autoRunning = true;
  if (autoRunning && distanceCm > OFF_CM) autoRunning = false;

  int speed = 0;
  if (autoRunning) {
    if (distanceCm <= NEAR_CM) {
      speed = 100;
    } else {
      float closeness = (FAR_CM - distanceCm) / (FAR_CM - NEAR_CM);  // 0 = far, 1 = near
      if (closeness < 0) closeness = 0;
      speed = MIN_RUN_PCT + closeness * (100 - MIN_RUN_PCT);
    }
  }

  // Optional: warm room keeps the fan running even with nobody near
  if (USE_THERMISTOR && !isnan(tempC) && tempC > TEMP_START_C) {
    int tempSpeed = MIN_RUN_PCT + (tempC - TEMP_START_C) * 15;  // +15% per degree
    if (tempSpeed > 100) tempSpeed = 100;
    if (tempSpeed > speed) speed = tempSpeed;
  }
  return speed;
}

int manualSpeed() {
  int pct = map(analogRead(POT_PIN), 0, 1023, 0, 100);
  if (pct < 5) return 0;  // small dead zone at the bottom = off
  return map(pct, 5, 100, MIN_RUN_PCT, 100);
}

// ---------- Motor ----------

void driveMotor() {
  if (targetPct > 0 && currentPct == 0) {
    // Kick-start: a short full-power burst gets the motor past its starting friction
    analogWrite(MOTOR_EN, 255);
    delay(300);
    currentPct = MIN_RUN_PCT;
  }

  // Ramp smoothly toward the target instead of jumping
  if (targetPct > currentPct) {
    currentPct = min(targetPct, currentPct + RAMP_STEP_PCT);
  } else if (targetPct < currentPct) {
    currentPct = max(targetPct, currentPct - RAMP_STEP_PCT);
    if (targetPct == 0 && currentPct < MIN_RUN_PCT) currentPct = 0;  // below this it would just hum
  }

  analogWrite(MOTOR_EN, map(currentPct, 0, 100, 0, 255));
}

// ---------- Outputs ----------

void updateLeds() {
  // Bands start at MIN_RUN_PCT (60%), the slowest speed the motor actually runs at
  digitalWrite(GREEN_LED_PIN, currentPct > 0 && currentPct < 75);
  digitalWrite(YELLOW_LED_PIN, currentPct >= 75 && currentPct < 90);
  digitalWrite(RED_LED_PIN, currentPct >= 90);
}

void updateDisplay() {
  oled.clearDisplay();
  oled.setTextColor(WHITE);

  // Header
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("SMART FAN");
  oled.setCursor(128 - 6 * strlen(modeNames[mode]), 0);
  oled.print(modeNames[mode]);
  oled.drawLine(0, 10, 127, 10, WHITE);

  // Big speed number
  oled.setTextSize(3);
  oled.setCursor(0, 15);
  oled.print(currentPct);
  oled.print("%");

  // Speed bar
  oled.drawRect(0, 42, 128, 8, WHITE);
  oled.fillRect(2, 44, currentPct * 124 / 100, 4, WHITE);

  // Distance and temperature
  oled.setTextSize(1);
  oled.setCursor(0, 54);
  oled.print("Dist:");
  if (distanceCm >= 399) oled.print(" --");
  else {
    oled.print(distanceCm, 0);
    oled.print("cm");
  }
  if (USE_THERMISTOR && !isnan(tempC)) {
    oled.setCursor(74, 54);
    oled.print(tempC, 1);
    oled.print("C");
  }
  oled.display();
}

// ---------- Setup and loop ----------

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // Spin direction is fixed by the wiring (swap the two motor wires to reverse the fan)
  analogWrite(MOTOR_EN, 0);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found at 0x3C - check SDA, SCL, VCC, GND");
  } else {
    Serial.println("OLED found");
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(20, 28);
  oled.print("Smart Fan ready");
  oled.display();
  delay(800);

  // Motor self-test: full speed for 1.5 s at power-up.
  // If the fan does not spin here, the problem is wiring or power, not the sensor logic.
  oled.clearDisplay();
  oled.setCursor(22, 28);
  oled.print("Motor test...");
  oled.display();
  Serial.println("Motor self-test: full speed for 1.5 s");
  analogWrite(MOTOR_EN, 255);
  delay(1500);
  analogWrite(MOTOR_EN, 0);
}

void loop() {
  // Button: one press = next mode (200 ms debounce)
  int b = digitalRead(BUTTON_PIN);
  if (b == LOW && lastButton == HIGH && millis() - lastPressMs > 200) {
    lastPressMs = millis();
    mode = (Mode)((mode + 1) % 3);
    autoRunning = false;
  }
  lastButton = b;

  updateDistance();
  updateTemperature();

  if (mode == MODE_OFF) targetPct = 0;
  else if (mode == MODE_AUTO) targetPct = autoSpeed();
  else targetPct = manualSpeed();

  driveMotor();
  updateLeds();
  updateDisplay();

  // Open Tools > Serial Plotter to watch these live
  Serial.print("distance_cm:");
  Serial.print(distanceCm, 1);
  Serial.print(" target_pct:");
  Serial.print(targetPct);
  Serial.print(" speed_pct:");
  Serial.print(currentPct);
  Serial.print(" pwm:");
  Serial.println(map(currentPct, 0, 100, 0, 255));

  delay(50);
}
