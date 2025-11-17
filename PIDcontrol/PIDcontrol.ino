#include <ECE3.h>
String dummy;

uint16_t sensorValues[8];

// --- Calibration values ---
uint16_t sensorMin[8] = {413, 413, 559, 504, 440, 596, 482, 546};
uint16_t sensorMax[8] = {2500, 2500, 2500, 2325, 2494, 1402, 2500, 2500};
float    sensorWeights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

// --- PID gains ---
float Kp = 0.4;
float Ki = 0.0;
float Kd = 3.0;

// --- PID state ---
float prevError = 0;
float integ = 0;
unsigned long prevTime = 0;

const int L_PWM_PIN = 40;
const int R_PWM_PIN = 39;
const int L_DIR_PIN = 29;
const int R_DIR_PIN = 30;
const int MOTOR_SLEEP_PIN = 31;

// Forward declaration
void setMotorSpeed(int left, int right);

void ECE3_SetMotorSpeed(int left, int right) {
  setMotorSpeed(left, right);
}


void StopMotors() {
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
  //digitalWrite(LED_RF, HIGH);   // if LED_RF exists
}


void setup() {
  ECE3_Init();

  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_SLEEP_PIN, OUTPUT); //new impl

  digitalWrite(MOTOR_SLEEP_PIN, HIGH);

  Serial.begin(9600);
  delay(1000);

  prevTime = millis();
}

// Low-level motor control
void setMotorSpeed(int left, int right) {
  int dirL = (left >= 0) ? HIGH : LOW;
  int dirR = (right >= 0) ? HIGH : LOW;

  analogWrite(L_PWM_PIN, abs(left));
  analogWrite(R_PWM_PIN, abs(right));

  digitalWrite(L_DIR_PIN, dirL);
  digitalWrite(R_DIR_PIN, dirR);
}

void loop() {
  int norm[8];
  int total = 0;
  float weightedSum = 0;

  // 1) Read IR sensors
  ECE3_read_IR(sensorValues);

  // 2) Normalize and compute weighted position
  for (int i = 0; i < 8; i++) {
    long raw = sensorValues[i];

    if (sensorMax[i] <= sensorMin[i])
      sensorMax[i] = sensorMin[i] + 1;   // avoid divide-by-zero

    raw = constrain(raw, sensorMin[i], sensorMax[i]);

    // Assume black line = lower reflect, so invert to make black ≈ 1000
    norm[i] = 1000 - 1000 * (raw - sensorMin[i]) / (sensorMax[i] - sensorMin[i]);
    norm[i] = constrain(norm[i], 0, 1000);

    total += norm[i];
    weightedSum += (float)norm[i] * sensorWeights[i];
  }

  float position = 0;
  if (total > 0) {
    position = weightedSum / total;  // roughly -15 to +15
  }

  // 3) Error (target center = 0)
  float error = -position;  // your original sign convention

  // 4) PID
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0.0005f) dt = 0.0005f;
  prevTime = now;

  float P = Kp * error;
  integ += Ki * error * dt;
  if (integ > 300) integ = 300;
  if (integ < -300) integ = -300;
  float D = Kd * (error - prevError) / dt;
  prevError = error;

  float correction = P + integ + D;

  // 5) Motor output
  int base = 90;     // adjust 80–140 depending on your car
  int leftMotor  = base + (int)correction;
  int rightMotor = base - (int)correction;

  leftMotor  = constrain(leftMotor,  -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  ECE3_SetMotorSpeed(leftMotor, rightMotor);

  // 6) Debug output
  Serial.print("pos=");
  Serial.print(position);
  Serial.print(" err=");
  Serial.print(error);
  Serial.print(" L=");
  Serial.print(leftMotor);
  Serial.print(" R=");
  Serial.println(rightMotor);

  delay(10);
}
