#include <ECE3.h>
String dummy;

uint16_t sensorValues[8];

uint16_t sensorMin[8] = {413, 413, 559, 504, 440, 596, 482, 546};
uint16_t sensorMax[8] = {2500, 2500, 2325, 2494, 1402, 2500, 2500};
float sensorWeights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

// PID gains (tune these)
float Kp = 0.4;
float Ki = 0.0;
float Kd = 3.0;

// PID state
float prevError = 0;
float integ = 0;
unsigned long prevTime = 0;

void setup() {
  ECE3_Init();
  Serial.begin(9600);
  delay(1000);
}

void setMotorSpeed(int left, int right) {
  int dirL = left >= 0 ? HIGH : LOW;
  int dirR = right >= 0 ? HIGH : LOW;
  analogWrite(L_PWM_PIN, abs(left));
  analogWrite(R_PWM_PIN, abs(right));
  digitalWrite(L_DIR_PIN, dirL);
  digitalWrite(R_DIR_PIN, dirR);
}


void loop() {
  // --- 1. Read and normalize sensors ---
  int norm[8];
  int total = 0;
  float weightedSum = 0;

  ECE3_read_IR(sensorValues);
  for (int i = 0; i < 8; i++) {
    long raw = sensorValues[i];
    raw = constrain(raw, sensorMin[i], sensorMax[i]);
    norm[i] = 1000 - 1000 * (raw - sensorMin[i]) / (sensorMax[i] - sensorMin[i]); // invert
    norm[i] = constrain(norm[i], 0, 1000);
    total += norm[i];
    weightedSum += (float)norm[i] * sensorWeights[i];
  }

  // --- 2. Compute position (centroid) ---
  float position = 0;
  if (total > 0) position = weightedSum / total;

  // --- 3. Compute error (desired 0 = center) ---
  // positive error → car left of line → steer right
  float error = -position;

  // --- 4. Compute PID correction ---
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  prevTime = now;

  float P = Kp * error;
  integ += Ki * error * dt;
  float D = Kd * (error - prevError) / dt;
  prevError = error;

  float correction = P + integ + D;

  // --- 5. Motor control ---
  int base = 90; // base PWM value
  int leftMotor = base + correction;
  int rightMotor = base - correction;

  leftMotor = constrain(leftMotor, 0, 255);
  rightMotor = constrain(rightMotor, 0, 255);

  ECE3_SetMotorSpeed(leftMotor, rightMotor);

  // --- 6. Debug output ---
  Serial.print("Err: "); Serial.print(error);
  Serial.print("\tPID: "); Serial.println(correction);

  delay(10);
}

