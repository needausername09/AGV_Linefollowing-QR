String chidan = "SLR";    // L = Left, R = Right, S = Straight
int chidanIndex = 0;
#define NUM_SENSORS 5

// IR sensor pins
const int sensors[NUM_SENSORS] = {18, 17, 16, 15, 7};

// Motor pins
const int IN1 = 11;  // Left motor forward
const int IN2 = 12;  // Left motor backward
const int IN3 = 13;  // Right motor forward
const int IN4 = 14;  // Right motor backward

// Vitme pins
const int VITME_FWD = 5;   // vít me ra
const int VITME_BWD = 6;   // vít me vào

// Speeds
const int L255 = 255, L235 = 235, L215 = 215, L175 = 175, L155 = 155;
const int R255 = 255, R235 = 235, R215 = 215, R175 = 175, R155 = 155;

// Debounce
int lastSensorRead[NUM_SENSORS] = {1, 1, 1, 1, 1};
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 10;

bool hasStarted = false;

// Lost line
String lastDirection = "STOP";
unsigned long lostLineStartTime = 0;
const unsigned long lostLineLimit = 500; // ms

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i], INPUT);
  }

  ledcAttach(IN1, 5000, 8);
  ledcAttach(IN2, 5000, 8);
  ledcAttach(IN3, 5000, 8);
  ledcAttach(IN4, 5000, 8);

  pinMode(VITME_FWD, OUTPUT);
  pinMode(VITME_BWD, OUTPUT);
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  analogWrite(IN1, leftSpeed);
  analogWrite(IN2, 0);
  analogWrite(IN3, rightSpeed);
  analogWrite(IN4, 0);
}

void turnLeft() {
  moveMotors(L175, R255);
  delay(400);
}

void turnRight() {
  moveMotors(L255, R175);
  delay(400);
}

void goStraight() {
  moveMotors(L255, R255);
  delay(400);
}

// ==== Hàm vitme ====
void vitmeRun() {
  Serial.println("Running vitme...");

  // Đẩy hàng ra
  digitalWrite(VITME_FWD, HIGH);
  digitalWrite(VITME_BWD, LOW);
  delay(2000); // thời gian vít me chạy ra

  // Dừng
  digitalWrite(VITME_FWD, LOW);
  delay(500);

  // Thu hàng về
  digitalWrite(VITME_FWD, LOW);
  digitalWrite(VITME_BWD, HIGH);
  delay(2000); // thời gian vít me chạy vào

  // Dừng hẳn
  digitalWrite(VITME_BWD, LOW);
}

void loop() {
  int sensorValues[NUM_SENSORS];
  bool blackDetected = false;

  // Debounce đọc cảm biến
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

  // Chờ gặp line đen để bắt đầu
  if (!hasStarted && blackDetected) {
    hasStarted = true;
  }
  if (!hasStarted) {
    stopMotors();
    return;
  }

  // Intersection: tất cả đều đen
  if (sensorValues[0] == 0 && sensorValues[1] == 0 &&
      sensorValues[2] == 0 && sensorValues[3] == 0 &&
      sensorValues[4] == 0) {
    stopMotors();
    delay(100); // dừng ngắn để ổn định

    if (chidanIndex < chidan.length()) {
      char cmd = chidan[chidanIndex];
      chidanIndex++;

      if (cmd == 'L') {
        Serial.println("Turn LEFT");
        turnLeft();
      } else if (cmd == 'R') {
        Serial.println("Turn RIGHT");
        turnRight();
      } else if (cmd == 'S') {
        Serial.println("Go STRAIGHT");
        goStraight();
      }
    } else {
      // Hết chỉ dẫn → line đen cuối → chạy vitme
      Serial.println("End of instructions. Starting vitme...");
      vitmeRun();
      while (true) { stopMotors(); } // Dừng hẳn sau khi vitme xong
    }
    return;
  }

  // Lost line: tất cả trắng
  if (sensorValues[0] == 1 && sensorValues[1] == 1 &&
      sensorValues[2] == 1 && sensorValues[3] == 1 &&
      sensorValues[4] == 1) {
    if (lostLineStartTime == 0) lostLineStartTime = millis();
    if (millis() - lostLineStartTime <= lostLineLimit) {
      if (lastDirection == "LEFT") {
        moveMotors(L175, R255);
      } else if (lastDirection == "RIGHT") {
        moveMotors(L255, R175);
      } else if (lastDirection == "STRAIGHT") {
        moveMotors(L255, R255);
      } else {
        stopMotors();
      }
    } else {
      stopMotors();
    }
    return;
  } else {
    lostLineStartTime = 0;
  }

  // Line-follow mapping
  if (sensorValues[0] == 0 && sensorValues[1] == 1) {
    moveMotors(L155, R255);
    lastDirection = "RIGHT";
  } else if (sensorValues[0] == 0 && sensorValues[1] == 0) {
    moveMotors(L175, R255);
    lastDirection = "RIGHT";
  } else if (sensorValues[1] == 0 && sensorValues[2] == 1) {
    moveMotors(L215, R255);
    lastDirection = "RIGHT";
  } else if (sensorValues[1] == 0 && sensorValues[2] == 0) {
    moveMotors(L235, R235);
    lastDirection = "STRAIGHT";
  } else if (sensorValues[2] == 0 && sensorValues[3] == 1) {
    moveMotors(L255, R255);
    lastDirection = "STRAIGHT";
  } else if (sensorValues[2] == 0 && sensorValues[3] == 0) {
    moveMotors(L255, R235);
    lastDirection = "LEFT";
  } else if (sensorValues[3] == 0 && sensorValues[4] == 1) {
    moveMotors(L255, R215);
    lastDirection = "LEFT";
  } else if (sensorValues[3] == 0 && sensorValues[4] == 0) {
    moveMotors(L255, R175);
    lastDirection = "LEFT";
  } else if (sensorValues[4] == 0 && sensorValues[3] == 1) {
    moveMotors(L255, R155);
    lastDirection = "LEFT";
  } else {
    stopMotors();
    lastDirection = "STOP";
  }
}
