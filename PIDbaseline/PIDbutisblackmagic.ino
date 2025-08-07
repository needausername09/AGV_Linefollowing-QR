  #define NUM_SENSORS 5

  // IR sensor pins
  const int sensors[NUM_SENSORS] = {42, 41, 40, 39, 38};

  // Motor pins
  const int IN1 = 11;  // Left motor forward
  const int IN2 = 12;  // Left motor backward
  const int IN3 = 13;  // Right motor forward
  const int IN4 = 14; // Right motor backward

  // Speeds
  const int L255 = 255, L235 = 235, L215 = 215, L175 = 175, L155 = 155;
  const int R255 = 255, R235 = 235, R215 = 215, R175 = 175, R155 = 155;

  // Debounce settings
  int lastSensorRead[NUM_SENSORS] = {1, 1, 1, 1, 1};
  unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 10; // ms

  bool hasStarted = false;

  void setup() {
    Serial.begin(115200);

    // Attach sensor pins
    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(sensors[i], INPUT);
    }

    // Attach motor pins with LEDC
    ledcAttach(IN1, 5000, 8);
    ledcAttach(IN2, 5000, 8);
    ledcAttach(IN3, 5000, 8);
    ledcAttach(IN4, 5000, 8);
  }

  void stopMotors() {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }

  void moveMotors(int leftSpeed, int rightSpeed) {
    // Forward movement
    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0);

    analogWrite(IN3, rightSpeed);
    analogWrite(IN4, 0);
  }

  void loop() {
    int sensorValues[NUM_SENSORS];
    bool blackDetected = false;

    // Read sensors with debounce
    if ((millis() - lastDebounceTime) > debounceDelay) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        int reading = digitalRead(sensors[i]);
        if (reading != lastSensorRead[i]) {
          lastDebounceTime = millis();
          lastSensorRead[i] = reading;
        }
        sensorValues[i] = lastSensorRead[i];
        if (sensorValues[i] == 0) blackDetected = true;
      }
    }

    // Wait for first black to start
    if (!hasStarted && blackDetected) {
      hasStarted = true;
    }

    if (!hasStarted) {
      stopMotors();
      return;
    }

    // Check for intersection (all black)
    if (sensorValues[0] == 0 && sensorValues[1] == 0 &&
        sensorValues[2] == 0 && sensorValues[3] == 0 &&
        sensorValues[4] == 0) {
      stopMotors();
      return;
    }

    // Logic mapping
    if (sensorValues[0] == 0 && sensorValues[1] == 1) {
      moveMotors(L155, R255);  // Sharp right
    } else if (sensorValues[0] == 0 && sensorValues[1] == 0) {
      moveMotors(L175, R255);
    } else if (sensorValues[1] == 0 && sensorValues[2] == 1) {
      moveMotors(L215, R255);
    } else if (sensorValues[1] == 0 && sensorValues[2] == 0) {
      moveMotors(L235, R235);
    } else if (sensorValues[2] == 0 && sensorValues[3] == 1) {
      moveMotors(L255, R255);
    } else if (sensorValues[2] == 0 && sensorValues[3] == 0) {
      moveMotors(L255, R235);
    } else if (sensorValues[3] == 0 && sensorValues[4] == 1) {
      moveMotors(L255, R215);
    } else if (sensorValues[3] == 0 && sensorValues[4] == 0) {
      moveMotors(L255, R175);
    } else if (sensorValues[4] == 0 && sensorValues[3] == 1) {
      moveMotors(L255, R155);  // Sharp left
    } else {
      stopMotors();  // Fail-safe
    }
  }

  