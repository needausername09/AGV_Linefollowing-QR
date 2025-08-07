#define NUM_SENSORS 5
String chidan = "";         
int chidanIndex = 0;        
String qrPayload = "";  

//WEACT ESP32S3 N16R8
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"

TaskHandle_t QRCodeReader_Task;

struct QRCodeData
{
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};

struct quirc *q = NULL;
uint8_t *image = NULL;
camera_fb_t * fb = NULL;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
struct QRCodeData qrCodeData;
     

void preprocessImage(uint8_t *buf, int width, int height) {
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      int idx = y * width + x;
      buf[idx] = (buf[idx - width] + buf[idx + width] + buf[idx - 1] + buf[idx + 1] + buf[idx]) / 5;
    }
  }
}

  // IR sensor pins
  const int sensors[NUM_SENSORS] = {42, 41, 40, 39, 38};

  // Motor pins
  const int IN1 = 11;  // Left motor forward
  const int IN2 = 12;  // Left motor backward
  const int IN3 = 13;  // Right motor forward
  const int IN4 = 14; // Right motor backward

  // Speeds
  const int L255 = 255, L235 = 235, L215 = 215, L195 = 195, L175 = 175;
  const int R255 = 255, R235 = 235, R215 = 215, R195 = 195, R175 = 175;

  // Debounce settings
  int lastSensorRead[NUM_SENSORS] = {1, 1, 1, 1, 1};
  unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 10; // ms

  bool hasStarted = false;

  // Đi tiếp theo đúng hướng config
  unsigned long lostLineStartTime = 0;
  bool isLineLost = false;
  int lastLeftSpeed = 0;
  int lastRightSpeed = 0;
  const unsigned long LOST_LINE_TIMEOUT = 500; // milliseconds




void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.printf("PSRAM total: %d, free: %d\n", ESP.getPsramSize(), ESP.getFreePsram());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_aec2(s, 1);

  q = quirc_new();
  if (q == NULL) {
    Serial.println("Failed to create quirc object");
    return;
  }

  if (quirc_resize(q, 640, 480) != 0) {
    Serial.println("quirc_resize failed");
    quirc_destroy(q);
    return;
  }

  Serial.println("Configure and initialize the camera successfully.");

  xTaskCreatePinnedToCore(QRCodeReader, "QRCodeReader_Task", 32768, NULL, 2, &QRCodeReader_Task, 0);

      for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(sensors[i], INPUT);
    }

    // Attach motor pins with LEDC
    ledcAttach(IN1, 5000, 8);
    ledcAttach(IN2, 5000, 8);
    ledcAttach(IN3, 5000, 8);
    ledcAttach(IN4, 5000, 8);
}

void QRCodeReader(void *pvParameters) {
  Serial.println("QRCodeReader started on core ");
  Serial.println(xPortGetCoreID());
  while (chidan == "") {
    fb = esp_camera_fb_get();
    image = quirc_begin(q, NULL, NULL);
    preprocessImage(fb->buf, fb->width, fb->height);
    memcpy(image, fb->buf, fb->len);
    quirc_end(q);
    int count = quirc_count(q);
    for (int i = 0; i < count; i++) {
      quirc_extract(q, i, &code);
      err = quirc_decode(&code, &data);
      if (err) {
        Serial.printf("Decoding FAILED: %s\n", quirc_strerror(err));
      } else {
        //Serial.println("Decoding successful:");
        dumpData(&data);
      }
    }

    esp_camera_fb_return(fb);
    fb = NULL;
    image = NULL;
    delay(10);
  }
  Serial.println("QR code read. Cleaning up camera and QR resources...");
  esp_camera_deinit();
  quirc_destroy(q);
  vTaskDelete(NULL);
}

void dumpData(const struct quirc_data *data)
{
  //Serial.printf("Version: %d\n", data->version);
  //Serial.printf("ECC level: %c\n", "MLHQ"[data->ecc_level]);
  //Serial.printf("Mask: %d\n", data->mask);
  //Serial.printf("Length: %d\n", data->payload_len);
  Serial.printf("%s\n", data->payload);
  qrPayload = String((char*)data->payload);
  chidan = qrPayload;
  Serial.print(qrPayload);
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

  // Wait for first black line to start
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

    if (chidanIndex < chidan.length()) {
      char lenh = chidan[chidanIndex];
      Serial.print("Intersection ");
      Serial.print(chidanIndex + 1);
      Serial.print(": Lenh = ");
      Serial.println(lenh);

      if (lenh == 'L') {
        analogWrite(IN1, 0);
        analogWrite(IN2, 200);
        analogWrite(IN3, 200);
        analogWrite(IN4, 0);
        delay(400);
      } else if (lenh == 'R') {
        analogWrite(IN1, 200);
        analogWrite(IN2, 0);
        analogWrite(IN3, 0);
        analogWrite(IN4, 200);
        delay(400);
      } else if (lenh == 'F') {
        moveMotors(L255, R255);
        delay(400);
      }

      chidanIndex++;
    } else {
      // Stop after final instruction at next intersection
      Serial.println("Da thuc hien het chi dan. Dung tai giao lo tiep theo.");
      while (true) {
        stopMotors();
      }
    }
    return;
  }

  // ❌ All sensors see white (off the line)
  if (!blackDetected) {
    if (!isLineLost) {
      // Start timer when we just lost the line
      isLineLost = true;
      lostLineStartTime = millis();
      moveMotors(lastLeftSpeed, lastRightSpeed);  // Continue in last direction
    } else if (millis() - lostLineStartTime <= LOST_LINE_TIMEOUT) {
      // Still within the allowed continuation time
      moveMotors(lastLeftSpeed, lastRightSpeed);
    } else {
      // Timeout reached: stop
      stopMotors();
    }
    return;
  }

  // ✅ We are on the line
  isLineLost = false;

  // Logic mapping
  if (sensorValues[0] == 0 && sensorValues[1] == 1) {
    lastLeftSpeed = L175; lastRightSpeed = R255;
  } else if (sensorValues[0] == 0 && sensorValues[1] == 0) {
    lastLeftSpeed = L195; lastRightSpeed = R255;
  } else if (sensorValues[1] == 0 && sensorValues[2] == 1) {
    lastLeftSpeed = L215; lastRightSpeed = R255;
  } else if (sensorValues[1] == 0 && sensorValues[2] == 0) {
    lastLeftSpeed = L235; lastRightSpeed = R235;
  } else if (sensorValues[2] == 0 && sensorValues[3] == 1) {
    lastLeftSpeed = L255; lastRightSpeed = R255;
  } else if (sensorValues[2] == 0 && sensorValues[3] == 0) {
    lastLeftSpeed = L255; lastRightSpeed = R235;
  } else if (sensorValues[3] == 0 && sensorValues[4] == 1) {
    lastLeftSpeed = L255; lastRightSpeed = R215;
  } else if (sensorValues[3] == 0 && sensorValues[4] == 0) {
    lastLeftSpeed = L255; lastRightSpeed = R195;
  } else if (sensorValues[4] == 0 && sensorValues[3] == 1) {
    lastLeftSpeed = L255; lastRightSpeed = R175;
  } else {
    stopMotors();  // Fail-safe
    return;
  }

  moveMotors(lastLeftSpeed, lastRightSpeed);
}
