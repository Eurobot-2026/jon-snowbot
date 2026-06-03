
#define TRIG_PIN  9
#define ECHO_PIN  10
#define ENA       5    // PWM — controls SPEED of motor A
#define IN1       6    // Direction pin 1
#define IN2       7    // Direction pin 2

float setPoint = 20.0;  // Target distance in cm (robot tries to stay 20 cm from crate)
float Kp = 3.0;         
float Ki = 0.15;        
float Kd = 0.8;        


const float DEAD_ZONE    = 1.5;  
const int   MIN_PWM      = 40;    
const int   MAX_PWM      = 200;   
const float DERIV_FILTER = 0.25;  
const int   DIR_HOLD_MS  = 120;   


const int   FILTER_SIZE  = 5;
float distBuffer[FILTER_SIZE];
int   filterIndex        = 0;
bool  filterFull         = false;


float previousError  = 0;
float filteredDeriv  = 0;
float integral       = 0;
unsigned long previousTime = 0;

// ── Direction Hysteresis State ───────────────────────────────
bool  movingForward  = true;
unsigned long lastDirChange = 0;

// ── For plotter explanation ──────────────────────────────────
float pTerm = 0, iTerm = 0, dTerm = 0;  // Store each PID term separately


float rawDistance() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  if (dur == 0) return -1.0;
  return dur * 0.0343 / 2.0;  // Convert microseconds → cm
}

// Moving average: averages last 5 readings to remove noise
float smoothedDistance() {
  float raw = rawDistance();
  if (raw < 0) return -1.0;
  distBuffer[filterIndex] = raw;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  if (filterIndex == 0) filterFull = true;
  int count = filterFull ? FILTER_SIZE : filterIndex;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += distBuffer[i];
  return sum / count;
}


void stopMotor() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
}

void setMotorSpeed(float pidOutput) {
  // If error is tiny → stop (dead zone prevents constant hunting at setpoint)
  if (abs(pidOutput) < (Kp * DEAD_ZONE)) { stopMotor(); return; }

  // Direction hysteresis: don't flip direction too fast (main jitter fix)
  bool wantForward = (pidOutput > 0);
  unsigned long now = millis();
  if (wantForward != movingForward) {
    if ((now - lastDirChange) < (unsigned long)DIR_HOLD_MS) {
      stopMotor(); return;  // Too soon to flip — pause instead
    }
    movingForward = wantForward;
    lastDirChange = now;
  }

  // Map PID output to PWM range with minimum floor
  int pwm = map(abs((int)pidOutput), 0, 255, MIN_PWM, MAX_PWM);
  pwm     = constrain(pwm, MIN_PWM, MAX_PWM);

  if (movingForward) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(ENA, pwm);
}


void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  stopMotor();

  // Pre-fill filter buffer so first readings aren't garbage
  for (int i = 0; i < FILTER_SIZE; i++) distBuffer[i] = setPoint;
  filterFull   = true;
  previousTime = millis();

  // ── Print PID explanation to Serial Monitor on startup ──
  // Open Tools → Serial Monitor (9600 baud) to read this
  Serial.println("=========================================");
  Serial.println("  PID DISTANCE-FOLLOWING ROBOT");
  Serial.println("=========================================");
  Serial.println();
  Serial.println("HOW PID WORKS:");
  Serial.println("  P (Proportional) = reacts to HOW FAR from target");
  Serial.println("    → far away = high P = fast speed");
  Serial.println("    → close = low P = slow speed");
  Serial.println();
  Serial.println("  I (Integral) = corrects LONG-TERM drift");
  Serial.println("    → if stuck slightly off target for a while");
  Serial.println("    → I builds up and pushes motor harder");
  Serial.println();
  Serial.println("  D (Derivative) = reacts to RATE OF CHANGE");
  Serial.println("    → if approaching fast → D slows it down");
  Serial.println("    → prevents overshoot (kept LOW to avoid jitter)");
  Serial.println();
  Serial.println("GAINS: Kp=3.0  Ki=0.15  Kd=0.8");
  Serial.println("SETPOINT: 20 cm");
  Serial.println();
  Serial.println("=========================================");
  Serial.println("SERIAL PLOTTER — open Tools > Serial Plotter");
  Serial.println("You will see 5 live traces:");
  Serial.println("  1. Distance_cm  = where crate actually is");
  Serial.println("  2. SetPoint_cm  = target (flat line at 20)");
  Serial.println("  3. P_term       = proportional contribution");
  Serial.println("  4. Error_cm     = how far off target");
  Serial.println("  5. PWM_output   = motor power 0-200");
  Serial.println("=========================================");
  Serial.println();

  // ── Serial Plotter column headers ──
  // Arduino Serial Plotter reads the first line as trace names
  Serial.println("Distance_cm SetPoint_cm P_term Error_cm PWM_output");
}


void loop() {
  float distance = smoothedDistance();

  // Handle bad readings
  if (distance < 0) {
    stopMotor();
    Serial.println("0 20 0 0 0");  // keep plotter alive
    delay(100); return;
  }
  if (distance > 300 || distance < 2) { stopMotor(); delay(50); return; }

  // ── Time delta ──────────────────────────────────────────
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  if (dt < 0.005) return;   // skip if loop ran too fast
  if (dt > 0.5)   dt = 0.5; // clamp if MCU was busy

  // ── PID Calculation ─────────────────────────────────────
  float error = setPoint - distance;
  // Positive error = too far → move forward
  // Negative error = too close → move back

  // P term: instant reaction to current error
  pTerm = Kp * error;

  // I term: accumulates error over time (with windup guard)
  integral += error * dt;
  integral  = constrain(integral, -80, 80);
  iTerm     = Ki * integral;

  // D term: filtered rate of change (low-pass to reduce noise sensitivity)
  float rawDeriv = (error - previousError) / dt;
  filteredDeriv  = (DERIV_FILTER * rawDeriv) + ((1.0 - DERIV_FILTER) * filteredDeriv);
  dTerm          = Kd * filteredDeriv;

  // Final PID output = sum of all three terms
  float pidOutput = pTerm + iTerm + dTerm;
  pidOutput = constrain(pidOutput, -255, 255);

  setMotorSpeed(pidOutput);

  // ── Calculate actual PWM for display ────────────────────
  int pwmOut = 0;
  if (abs(pidOutput) >= (Kp * DEAD_ZONE)) {
    pwmOut = map(abs((int)pidOutput), 0, 255, MIN_PWM, MAX_PWM);
    pwmOut = constrain(pwmOut, MIN_PWM, MAX_PWM);
  }

  // ── Serial Plotter Output ───────────────────────────────
  // Format: "val1 val2 val3 val4 val5"
  // Trace 1: Distance_cm  — where robot actually sees the crate
  // Trace 2: SetPoint_cm  — flat line at 20 cm (the target)
  // Trace 3: P_term       — proportional contribution (shows PID reacting)
  // Trace 4: Error_cm     — gap between target and actual
  // Trace 5: PWM_output   — motor power being sent

  Serial.print(distance, 1);       // Trace 1: Distance
  Serial.print(" ");
  Serial.print(setPoint, 1);       // Trace 2: SetPoint (flat 20 cm line)
  Serial.print(" ");
  Serial.print(pTerm / 3.0, 1);    // Trace 3: P term (divided by 3 to fit same scale)
  Serial.print(" ");
  Serial.print(error, 1);          // Trace 4: Error
  Serial.print(" ");
  Serial.println(pwmOut);          // Trace 5: PWM

  previousError = error;
  previousTime  = currentTime;
  delay(50);  // 20 Hz loop — good for HC-SR04
}
