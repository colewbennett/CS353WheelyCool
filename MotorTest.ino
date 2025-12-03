// pin assignment
const int MIC_PIN = A0;

// L293D side A (Motor A)
const int ENA_PIN = 5;
const int IN1_PIN = 2;
const int IN2_PIN = 3;

// L293D side B (Motor B)
const int ENB_PIN = 6;
const int IN3_PIN = 4;
const int IN4_PIN = 7;

// loudness settings
// how many analog samples per loudness reading
const int MIC_SAMPLES      = 40;

// loudness threshold
const int TRIGGER_LEVEL    = 25;

// how many loudness readings to take during one listen window
const int LISTEN_READS     = 6;

// how many of those readings must exceed TRIGGER_LEVEL to count as a real loud sound
const int LISTEN_HITS_NEED = 2;

// motion stuff
const int MOTION_SPEED         = 120;
const unsigned long FORWARD_DURATION = 5000;  // ms
const unsigned long TURN_DURATION    = 5000;  // ms
const unsigned long PAUSE_DURATION   = 10000; // ms on loud sound


// how often to stop briefly to listen (while moving)
const unsigned long LISTEN_INTERVAL_MS = 700;

// time to wait after stopping before reading mic (let noise settle)
const unsigned long LISTEN_SETTLE_MS   = 80;

// states for motion
enum MotionState {
  STATE_FORWARD,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_PAUSE
};

MotionState currentState = STATE_FORWARD;
unsigned long stateEndTime = 0;
unsigned long lastStateChangeTime = 0;
unsigned long nextListenTime = 0;


// mic helpers


int readMicLoudness() {
  long sum = 0;

  // get DC offset for this instant
  for (int i = 0; i < MIC_SAMPLES; i++) {
    int raw = analogRead(MIC_PIN);
    sum += raw;
  }
  int offset = sum / MIC_SAMPLES;

  // average absolute deviation from that offset
  long totalDev = 0;
  for (int i = 0; i < MIC_SAMPLES; i++) {
    int raw = analogRead(MIC_PIN);
    int deviation = raw - offset;
    if (deviation < 0) deviation = -deviation;
    totalDev += deviation;
  }

  int avgDeviation = totalDev / MIC_SAMPLES;
  return avgDeviation;
}

// stops motors to listen, loud noise = true
bool listenForLoudSoundWindow() {
  Serial.println("LISTEN WINDOW: stopping to listen");

  // stop motors so the mic is not hearing them
  stopMotors();

  // let things settle a bit
  delay(LISTEN_SETTLE_MS);

  int loudCount = 0;

  for (int i = 0; i < LISTEN_READS; i++) {
    int loudness = readMicLoudness();
    Serial.print("  listen loudness: ");
    Serial.println(loudness);

    if (loudness > TRIGGER_LEVEL) {
      loudCount++;
    }

    delay(20);  // small delay between reads
  }

  Serial.print("  loud hits in window: ");
  Serial.println(loudCount);

  if (loudCount >= LISTEN_HITS_NEED) {
    Serial.println("  -> LOUD SOUND DETECTED");
    return true;
  } else {
    Serial.println("  -> no loud sound");
    return false;
  }
}

// helper functions

void setMotorA(int speed, int direction) {
  if (direction > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (direction < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  speed = constrain(speed, 0, 255);
  analogWrite(ENA_PIN, speed);
}

void setMotorB(int speed, int direction) {
  if (direction > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else if (direction < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }

  speed = constrain(speed, 0, 255);
  analogWrite(ENB_PIN, speed);
}

void stopMotors() {
  setMotorA(0, 0);
  setMotorB(0, 0);
}

void driveForward(int speed) {
  setMotorA(speed, 1);
  setMotorB(speed, 1);
}

// turn left in place: left wheel backward, right wheel forward
void turnLeft(int speed) {
  setMotorA(speed, -1);
  setMotorB(speed, 1);
}

// turn right in place: left wheel forward, right wheel backward
void turnRight(int speed) {
  setMotorA(speed, 1);
  setMotorB(speed, -1);
}

void changeState(MotionState newState, unsigned long duration) {
  currentState = newState;
  unsigned long now = millis();
  stateEndTime = now + duration;
  lastStateChangeTime = now;

  // schedule next listen window 
  nextListenTime = now + LISTEN_INTERVAL_MS;

  Serial.print("CHANGE STATE -> ");
  switch (newState) {
    case STATE_FORWARD: Serial.println("FORWARD"); break;
    case STATE_LEFT:    Serial.println("LEFT");    break;
    case STATE_RIGHT:   Serial.println("RIGHT");   break;
    case STATE_PAUSE:   Serial.println("PAUSE");   break;
  }
}

// SETUP

void setup() {
  pinMode(MIC_PIN, INPUT);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  stopMotors();

  Serial.begin(9600);

  unsigned long now = millis();
  currentState = STATE_FORWARD;
  stateEndTime = now + FORWARD_DURATION;
  lastStateChangeTime = now;
  nextListenTime = now + LISTEN_INTERVAL_MS;

  Serial.println("SETUP COMPLETE -> STATE: FORWARD");
}

// MAIN LOOP

void loop() {
  unsigned long now = millis();

  // periodically stop to listen
  if (currentState != STATE_PAUSE && now >= nextListenTime) {
    if (listenForLoudSoundWindow()) {
      // loud sound detected, 10s pause
      stopMotors();
      changeState(STATE_PAUSE, PAUSE_DURATION);
      return;
    } else {
      // next listening window
      nextListenTime = millis() + LISTEN_INTERVAL_MS;
    }
  }

  // states for movement / pause
  switch (currentState) {
    case STATE_FORWARD:
      driveForward(MOTION_SPEED);
      if (now >= stateEndTime) {
        changeState(STATE_LEFT, TURN_DURATION);
      }
      break;

    case STATE_LEFT:
      turnLeft(MOTION_SPEED);
      if (now >= stateEndTime) {
        changeState(STATE_RIGHT, TURN_DURATION);
      }
      break;

    case STATE_RIGHT:
      turnRight(MOTION_SPEED);
      if (now >= stateEndTime) {
        changeState(STATE_FORWARD, FORWARD_DURATION);
      }
      break;

    case STATE_PAUSE:
      stopMotors();
      if (now >= stateEndTime) {
        // after pausing restart the pattern with forward
        changeState(STATE_FORWARD, FORWARD_DURATION);
      }
      break;
  }

  delay(20);
}
