#include <ECE3.h>

// ==================== Mode enum (from Main.h) ====================
enum mode { STRAIGHT, RIBBON };

// ==================== Track selection ====================
// Change this to STRAIGHT if you run the straight track
const mode currentMode = RIBBON;

// ==================== PIN constants (from Main.h) ====================
const uint8_t LEFT_NSLP  = 31;
const uint8_t LEFT_DIR   = 29;
const uint8_t LEFT_PWM   = 40;
const uint8_t RIGHT_NSLP = 11;
const uint8_t RIGHT_DIR  = 30;
const uint8_t RIGHT_PWM  = 39;

// YELLOW_LED is defined by Energia for MSP432 LaunchPad
// (used as crossbar indicator)

// ==================== PID constants (from Main.h) ====================
const double kP = 0.35;
const double kI = 0.00000000001;  // effectively ~0
const double kD = 2.0;

// ==================== IR constants (from Main.h) ====================
const uint8_t NUM_IRS = 8;

// Offset values are used to set the calibrated white value of every sensor to 0.
const uint16_t IR_OFFSET[NUM_IRS] = {
  450, 371, 371, 416, 393, 416, 498, 498
};

// Scaling values scale each sensor's raw input to a value between 0 and 1000.
const double IR_SCALE[NUM_IRS] = {
  0.48780488, 0.46970409, 0.46970409, 0.54734537,
  0.54704595, 0.47984645, 0.49950050, 0.49950050
};

// Give different weights depending on how far the sensor is from the center.
// 0 --> 7; right --> left
const short IR_WEIGHT[NUM_IRS] = { -8, -4, -2, -1, 1, 2, 4, 8 };

// ==================== Speed / track constants ====================

// Analog base speed of the RSLK before PID corrections.
const uint8_t BASE_SPEED = 40;

// Below the deadband range, wheels will not spin due to static friction.
const uint8_t DEADBAND = 11;

// Threshold of when a crossbar is detected based on summedInput.
const uint16_t BAR_THRESHOLD = 5000;

// Time after a crossbar is detected before another crossbar can be counted.
const uint16_t BAR_TIME = 750;  // ms

// Number of crossbars detected before a donut is performed based on track.
const uint8_t STRAIGHT_BARS = 1;
const uint8_t RIBBON_BARS   = 3;

// Amount of time to spin for (in loop cycles; not ms, as in original code).
const uint32_t SPIN_TIME = 180000;

// Analog PWM speed of both left and right wheels to spin.
const uint8_t SPIN_SPEED = 40;

// ==================== Globals (from Main.h) ====================

// Stores the raw IR readings from each sensor. 0 --> 7; right --> left.
uint16_t IR_Values[NUM_IRS];

// Stores calibrated input based on offset and scale to [0, 1000].
double calibratedInput;

// Stores the sum of all calibrated inputs (no fusion).
double summedInput;

// Stores the computed input value after calibration and fusion (weighted sum).
double fusedInput;

// PID output: if negative, RSLK is left of track; if positive, right of track.
double output;

// Setpoint of PID controller (we want fusedInput ≈ 0).
double targetValue = 0.0;

// Motor speeds based on BASE_SPEED and PID correction.
double leftSpeed, rightSpeed;

// Time of last crossbar detection.
unsigned long detectBarTime = 0;

// Number of crossbars seen.
uint8_t numBars = 0;

// Loop index.
size_t i;

// ==================== Simple PID state (replaces PID_v1) ====================
double pidPrevError = 0.0;
double pidIntegral  = 0.0;

// ==================== Function prototypes ====================
void goStraight();
void donut();

// ==================== setup() ====================
void setup() {

  pinMode(LEFT_NSLP,  OUTPUT);
  pinMode(LEFT_DIR,   OUTPUT);
  pinMode(LEFT_PWM,   OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM,  OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  // Enable motor drivers and set initial forward direction.
  digitalWrite(LEFT_NSLP,  HIGH);
  digitalWrite(LEFT_DIR,   LOW); // Forwards direction.
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR,  LOW); // Forwards direction.

  ECE3_Init();

  // If you want debug prints, uncomment:
  // Serial.begin(9600);

  // Wait 5 seconds before starting (to place robot).
  delay(5000);

  detectBarTime = millis();
}

// ==================== loop() ====================
void loop() {

  // 1) Read IR sensors: raw values 0 (white) to ~2500 (black)
  ECE3_read_IR(IR_Values);

  fusedInput = 0.0;
  summedInput = 0.0;

  // 2) Calibrate each sensor and compute fusedInput and summedInput
  for (i = 0; i < NUM_IRS; ++i) {
    // Calibrate to [0, 1000]
    int32_t raw = IR_Values[i];
    int32_t off = (int32_t)raw - (int32_t)IR_OFFSET[i];
    if (off < 0) off = 0;

    calibratedInput = (double)off * IR_SCALE[i];
    if (calibratedInput < 0)    calibratedInput = 0;
    if (calibratedInput > 1000) calibratedInput = 1000;

    // Weighted sum for PID input
    fusedInput += calibratedInput * (double)IR_WEIGHT[i];

    // Sum all sensors to detect crossbars
    summedInput += calibratedInput;
  }

  // 3) Check if we are on a crossbar
  if (summedInput >= BAR_THRESHOLD) {
    digitalWrite(YELLOW_LED, HIGH);

    // Do not count another crossbar if BAR_TIME has not elapsed.
    if (millis() - detectBarTime >= BAR_TIME) {
      numBars++;
      detectBarTime = millis();
    }

    // Do a donut (spin) when we've seen enough bars for this track.
    if ((currentMode == STRAIGHT && numBars == STRAIGHT_BARS) ||
        (currentMode == RIBBON   && numBars == RIBBON_BARS)) {
      donut();
    }

    // If on ribbon track and back at start/finish line, stop moving.
    if (currentMode == RIBBON && numBars >= RIBBON_BARS * 2) {
      while (true) {
        analogWrite(LEFT_PWM,  0);
        analogWrite(RIGHT_PWM, 0);
      }
    }

  } else {
    // Not under a crossbar → normal line following
    digitalWrite(YELLOW_LED, LOW);
    goStraight();
  }
}

// ==================== goStraight(): PID line following ====================
void goStraight() {

  // --- Simple PID computation (replaces PID_v1.Compute) ---

  // error = setpoint - input
  double error = targetValue - fusedInput;

  // Integral
  pidIntegral += error * kI;

  // Derivative
  double dError = error - pidPrevError;
  pidPrevError = error;

  // PID output
  double pidOut = kP * error + pidIntegral + kD * dError;

  // Limit output similarly to SetOutputLimits(-BASE_SPEED, BASE_SPEED)
  if (pidOut >  BASE_SPEED) pidOut =  BASE_SPEED;
  if (pidOut < -BASE_SPEED) pidOut = -BASE_SPEED;

  output = pidOut;

  // --- Map PID output to left/right speeds (same pattern as original) ---

  leftSpeed  = BASE_SPEED + output;
  rightSpeed = BASE_SPEED - output;

  // Clip to [0, 255]
  if (leftSpeed  < 0)   leftSpeed  = 0;
  if (leftSpeed  > 255) leftSpeed  = 255;
  if (rightSpeed < 0)   rightSpeed = 0;
  if (rightSpeed > 255) rightSpeed = 255;

  // Apply deadband: map 0..255 to 0 or [DEADBAND..255]
  // If speed is 0 → keep 0. Otherwise, remap.
  double leftPWM, rightPWM;

  if (leftSpeed <= 0.1) {
    leftPWM = 0;
  } else {
    leftPWM = map((long)leftSpeed, 0L, 255L, (long)DEADBAND, 255L);
  }

  if (rightSpeed <= 0.1) {
    rightPWM = 0;
  } else {
    rightPWM = map((long)rightSpeed, 0L, 255L, (long)DEADBAND, 255L);
  }

  // Move the motors (forward direction already set in setup)
  analogWrite(LEFT_PWM,  (int)leftPWM);
  analogWrite(RIGHT_PWM, (int)rightPWM);

  // Optional debug:
  // Serial.print("fusedInput="); Serial.print(fusedInput);
  // Serial.print(" output="); Serial.print(output);
  // Serial.print(" L="); Serial.print(leftPWM);
  // Serial.print(" R="); Serial.println(rightPWM);
}

// ==================== donut(): spin in place at crossbar ====================
void donut() {

  // Reverse right wheel direction for CCW spin
  digitalWrite(RIGHT_DIR, HIGH);  // LEFT_DIR stays LOW (forward)

  // Spin! (open-loop, same idea as original SPIN_TIME)
  for (i = 0; i < SPIN_TIME; ++i) {
    analogWrite(LEFT_PWM,  SPIN_SPEED);
    analogWrite(RIGHT_PWM, SPIN_SPEED);
  }

  // Restore right wheel direction to forward
  digitalWrite(RIGHT_DIR, LOW);
}
