#include <ECE3.h>
String dummy;

uint16_t sensorValues[8];

// --- your calibrations (added the 8th max = 2500; change to your real value) ---
uint16_t sensorMin[8] = {413, 413, 559, 504, 440, 596, 482, 546};
uint16_t sensorMax[8] = {2500, 2500, 2500, 2325, 2494, 1402, 2500, 2500};
float    sensorWeights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

// --- PID gains (yours kept) ---
float Kp = 0.4;
float Ki = 0.0;
float Kd = 3.0;

// --- PID state (yours kept) ---
float prevError = 0;
float integ = 0;
unsigned long prevTime = 0;

// --- IMPLEMENTATION: define your motor pins (edit to match your board) ---
#define L_PWM_PIN   5
#define L_DIR_PIN   4
#define R_PWM_PIN   6
#define R_DIR_PIN   7

// --- IMPLEMENTATION: forward declare our helper so we can call it anywhere ---
void setMotorSpeed(int left, int right);

void ECE3_SetMotorSpeed(int left, int right) {  // matches your usage exactly
  setMotorSpeed(left, right);
}

void setup() {
  ECE3_Init();
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  Serial.begin(9600);
  delay(1000);
}

// --- IMPLEMENTATION: your low-level motor writer, using your pin names ---
void setMotorSpeed(int left, int right) {
  int dirL = left >= 0 ? HIGH : LOW;
  int dirR = right >= 0 ? HIGH : LOW;
  analogWrite(L_PWM_PIN, abs(left));
  analogWrite(R_PWM_PIN, abs(right));
  digitalWrite(L_DIR_PIN, dirL);
  digitalWrite(R_DIR_PIN, dirR);
}

void loop() {
  int norm[8];
  int total = 0;
  float weightedSum = 0;

  ECE3_read_IR(sensorValues);
  for (int i = 0; i < 8; i++) {
    long raw = sensorValues[i];
    // guard against bad calibration (avoid /0)
    if (sensorMax[i] <= sensorMin[i]) sensorMax[i] = sensorMin[i] + 1;

    raw = constrain(raw, sensorMin[i], sensorMax[i]);
    // invert so 1000 = black line
    norm[i] = 1000 - 1000 * (raw - sensorMin[i]) / (sensorMax[i] - sensorMin[i]);
    norm[i] = constrain(norm[i], 0, 1000);
    total += norm[i];
    weightedSum += (float)norm[i] * sensorWeights[i];
  }

  // --- 2. Compute position (centroid) ---
  float position = 0;
  if (total > 0) position = weightedSum / total;

  // --- 3. Error (desired 0 = center) ---
  float error = -position; // your convention

  // --- 4. PID correction (yours kept; added tiny dt guard and integ clamp) ---
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0.0005f) dt = 0.0005f;
  prevTime = now;

  float P = Kp * error;
  integ += Ki * error * dt;
  // anti-windup clip (gentle)
  if (integ > 300) integ = 300;
  if (integ < -300) integ = -300;
  float D = Kd * (error - prevError) / dt;
  prevError = error;

  float correction = P + integ + D;

  // --- 5. Motor control (yours kept) ---
  int base = 90; // your base PWM
  int leftMotor  = base + (int)correction;
  int rightMotor = base - (int)correction;

  // allow reverse so it can pivot if needed
  leftMotor  = constrain(leftMotor,  -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  ECE3_SetMotorSpeed(leftMotor, rightMotor);

  // optional: brief delay to tame serial spam / PWM jitter
  delay(10);
}

