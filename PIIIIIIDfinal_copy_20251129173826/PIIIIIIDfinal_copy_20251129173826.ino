#include <ECE3.h>

// ==================== Mode enum (from Main.h) ====================
enum mode { STRAIGHT, RIBBON };

// ==================== Track selection ====================
// Change this to STRAIGHT if you run the straight track
const mode currentMode = RIBBON;

const uint8_t  RIBBON_STOP_BAR = 6;

// ==================== PIN constants (from Main.h) ====================
const uint8_t LEFT_NSLP  = 31;
const uint8_t LEFT_DIR   = 29;
const uint8_t LEFT_PWM   = 40;
const uint8_t RIGHT_NSLP = 11;
const uint8_t RIGHT_DIR  = 30;
const uint8_t RIGHT_PWM  = 39;

// When both halves of the array see strong signal, treat it as "two lines"
// and bias toward the left branch.
const double DOUBLE_LINE_MIN = 600.0;   // Tune in lab if needed
const double LEFT_LINE_BIAS  = 1600.0;  // Positive fusedInput -> steer left

// ==================== PID constants (from Main.h) ====================
const double kP = 0.35;
const double kI = 0.00000000001;
const double kD = 2.0;

// ==================== IR constants (from Main.h) ====================
const uint8_t NUM_IRS = 8;
const uint16_t IR_OFFSET[NUM_IRS] = { 450, 371, 371, 416, 393, 416, 498, 498 };
const double IR_SCALE[NUM_IRS]    = { 0.48780488, 0.46970409, 0.46970409, 0.54734537, 0.54704595, 0.47984645, 0.49950050, 0.49950050 };
const short IR_WEIGHT[NUM_IRS]    = { -8, -4, -2, -1, 1, 2, 4, 8 };

// ==================== Speed / track constants ====================
const uint8_t  BASE_SPEED    = 40;
const uint8_t  DEADBAND      = 11;
const uint16_t BAR_THRESHOLD = 5000;
const uint16_t BAR_TIME      = 750;  // ms
const uint8_t  STRAIGHT_BARS = 1;
const uint8_t  RIBBON_BARS   = 3;

// Existing 360 donut constants
const uint32_t SPIN_TIME  = 180000; // ~360 degrees (tuned for your robot)
const uint8_t  SPIN_SPEED = 40;

// New constant for 225 degree spin
// You calibrated this value so the robot turns about 225 degrees:
const uint32_t SPIN_TIME_225 = 162500;

// ==================== Globals (from Main.h) ====================
uint16_t IR_Values[NUM_IRS];
double calibratedInput;
double summedInput;
double fusedInput;
double output;
double targetValue = 0.0;
double leftSpeed, rightSpeed;
unsigned long detectBarTime = 0;
uint8_t numBars = 0;
size_t i;

// Only do the 225 spin once
bool specialSpinComplete = false;

// ==================== Simple PID state ====================
double pidPrevError = 0.0;
double pidIntegral  = 0.0;

// ==================== Timer globals (NEW) ====================
unsigned long startTime = 0;     // <<< NEW
bool timerStarted      = false;  // <<< NEW

// ==================== Function prototypes ====================
void goStraight();
void donut();
void spin225CCW();

// ==================== setup() ====================
void setup() {
  pinMode(LEFT_NSLP,  OUTPUT);
  pinMode(LEFT_DIR,   OUTPUT);
  pinMode(LEFT_PWM,   OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM,  OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  digitalWrite(LEFT_NSLP,  HIGH);
  digitalWrite(LEFT_DIR,   LOW); // Forward
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR,  LOW); // Forward

  ECE3_Init();
  delay(5000);              // 5s to place the robot
  detectBarTime = millis(); // for bar debounce
}

// ==================== loop() ====================
void loop() {

  // --------- 15.5-second global timeout (NEW) ----------
  if (!timerStarted) {                    // first time we enter loop
    startTime = millis();                 // start the timer
    timerStarted = true;
  }

  if (millis() - startTime >= 15900) {    // 15.5 seconds elapsed
    analogWrite(LEFT_PWM,  0);           // stop both wheels
    analogWrite(RIGHT_PWM, 0);
    return;                              // do nothing else forever
  }
  // ------------------------------------------------------

  // 1) Read IR sensors
  ECE3_read_IR(IR_Values);

  fusedInput  = 0.0;
  summedInput = 0.0;

  // track how much signal is on the left half vs right half
  double leftTotal  = 0.0;
  double rightTotal = 0.0;

  // 2) Calibrate and compute fused input
  for (i = 0; i < NUM_IRS; ++i) {
    int32_t raw = IR_Values[i];
    int32_t off = (int32_t)raw - (int32_t)IR_OFFSET[i];
    if (off < 0) off = 0;

    calibratedInput = (double)off * IR_SCALE[i];
    if (calibratedInput < 0)    calibratedInput = 0;
    if (calibratedInput > 1000) calibratedInput = 1000;

    fusedInput  += calibratedInput * (double)IR_WEIGHT[i];
    summedInput += calibratedInput;

    // accumulate left half vs right half
    if (i < NUM_IRS / 2) {
      leftTotal += calibratedInput;
    } else {
      rightTotal += calibratedInput;
    }
  }

  // 3) Check if we are on a crossbar
  if (summedInput >= BAR_THRESHOLD) {
    // We are on black
    digitalWrite(YELLOW_LED, HIGH);

    unsigned long now = millis();
    // Count a bar only if we have been on black long enough
    if (now - detectBarTime >= BAR_TIME) {
      numBars++;
      detectBarTime = now;

      // Special 225 spin: only once, on the FIRST bar in RIBBON mode
      if (currentMode == RIBBON && !specialSpinComplete && numBars == 1) {
        spin225CCW();
        specialSpinComplete = true;

        // Reset PID state after big turn
        pidPrevError = 0.0;
        pidIntegral  = 0.0;
      }

      // Original donut logic only for STRAIGHT mode
      if (currentMode == STRAIGHT && numBars == STRAIGHT_BARS) {
        donut();
      }

      // Stop at end of ribbon (if you still want this)
      if (currentMode == RIBBON && numBars > RIBBON_STOP_BAR) {
        while (true) {
          analogWrite(LEFT_PWM,  0);
          analogWrite(RIGHT_PWM, 0);
        }
      }
    }

  } else {
    // We are on white / normal line area
    digitalWrite(YELLOW_LED, LOW);

    if (currentMode == RIBBON &&
        leftTotal > DOUBLE_LINE_MIN &&
        rightTotal > DOUBLE_LINE_MIN) {
      fusedInput = LEFT_LINE_BIAS;  // Positive => steer left with current PID wiring
    }

    goStraight();
  }
}

// ==================== spin225CCW() ====================
void spin225CCW() {
  // Counterclockwise: left wheel reverse, right wheel forward
  digitalWrite(LEFT_DIR,  HIGH); // Reverse left wheel
  digitalWrite(RIGHT_DIR, LOW);  // Forward right wheel

  // Simple open loop spin for calibrated time
  for (volatile uint32_t k = 0; k < SPIN_TIME_225; ++k) {
    analogWrite(LEFT_PWM,  SPIN_SPEED);
    analogWrite(RIGHT_PWM, SPIN_SPEED);
  }

  // Restore left wheel to forward for normal driving
  digitalWrite(LEFT_DIR, LOW);
}

// ==================== goStraight() ====================
void goStraight() {
  double error = targetValue - fusedInput;

  pidIntegral += error * kI;
  double dError = error - pidPrevError;
  pidPrevError = error;

  double pidOut = kP * error + pidIntegral + kD * dError;

  if (pidOut >  BASE_SPEED) pidOut =  BASE_SPEED;
  if (pidOut < -BASE_SPEED) pidOut = -BASE_SPEED;

  output = pidOut;

  leftSpeed  = BASE_SPEED + output;
  rightSpeed = BASE_SPEED - output;

  if (leftSpeed  < 0)   leftSpeed  = 0;
  if (leftSpeed  > 255) leftSpeed  = 255;
  if (rightSpeed < 0)   rightSpeed = 0;
  if (rightSpeed > 255) rightSpeed = 255;

  double leftPWM, rightPWM;

  if (leftSpeed <= 0.1) leftPWM = 0;
  else leftPWM = map((long)leftSpeed, 0L, 255L, (long)DEADBAND, 255L);

  if (rightSpeed <= 0.1) rightPWM = 0;
  else rightPWM = map((long)rightSpeed, 0L, 255L, (long)DEADBAND, 255L);

  analogWrite(LEFT_PWM,  (int)leftPWM);
  analogWrite(RIGHT_PWM, (int)rightPWM);
}

// ==================== donut() ====================
// Original full 360 clockwise spin (for STRAIGHT mode)
void donut() {
  digitalWrite(RIGHT_DIR, HIGH); // Reverse right wheel, left stays forward

  for (i = 0; i < SPIN_TIME; ++i) {
    analogWrite(LEFT_PWM,  SPIN_SPEED);
    analogWrite(RIGHT_PWM, SPIN_SPEED);
  }

  digitalWrite(RIGHT_DIR, LOW); // Restore forward
}






