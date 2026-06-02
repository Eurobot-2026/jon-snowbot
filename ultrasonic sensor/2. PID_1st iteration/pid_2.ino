// ── Pin Definitions ──────────────────────────────────────
#define TRIG_PIN 9
#define ECHO_PIN 10
#define ENA      5   // PWM
#define IN1      6
#define IN2      7

// ── PID Parameters (tune these during testing) ───────────
float setPoint      = 20.0;  // target distance in cm
float Kp            = 3.0;
float Ki            = 0.2;
float Kd            = 2.0;

// ── State ─────────────────────────────────────────────────
float previousError = 0;
float integral      = 0;
unsigned long previousTime = 0;

// ── Distance Reading ──────────────────────────────────────
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

// ── Motor Control (supports forward AND reverse) ──────────
void setMotorSpeed(float pidOutput) {
  int speed = constrain(abs((int)pidOutput), 0, 255);

  if (pidOutput > 0) {
    // Object too far → move forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    // Object too close → reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // Dead zone: stop if error is tiny (avoids jitter)
  if (abs(pidOutput) < 5) speed = 0;

  analogWrite(ENA, speed);
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Start safe
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  previousTime = millis();
  Serial.println("PID Motor Test Started");
}

// ── Main Loop ─────────────────────────────────────────────
void loop() {
  float distance = readDistance();

  if (distance < 0) {
    setMotorSpeed(0);
    Serial.println("ERR: No sensor reading");
    delay(100);
    return;
  }

  // Clamp unrealistic readings (HC-SR04 range: 2–400 cm)
  if (distance > 400 || distance < 2) {
    Serial.print("ERR: Out of range: ");
    Serial.println(distance);
    delay(100);
    return;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  if (dt <= 0) dt = 0.01;

  // ── PID Calculation ──
  float error      = setPoint - distance;
  integral        += error * dt;
  integral         = constrain(integral, -100, 100);  // windup guard
  float derivative = (error - previousError) / dt;
  float pidOutput  = (Kp * error) + (Ki * integral) + (Kd * derivative);

  setMotorSpeed(pidOutput);

  // ── Serial Monitor Output ──
  Serial.print("Dist: ");    Serial.print(distance, 1);
  Serial.print(" cm | Err: "); Serial.print(error, 2);
  Serial.print(" | PID: ");  Serial.print(pidOutput, 1);
  Serial.print(" | PWM: ");  Serial.println(constrain(abs((int)pidOutput), 0, 255));

  previousError = error;
  previousTime  = currentTime;
  delay(50);
} 
