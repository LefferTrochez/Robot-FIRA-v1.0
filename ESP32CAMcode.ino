#include "esp_camera.h"
#include <WiFi.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

const char* ssid = "ML-042";
const char* password = "Leffersito";

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define PWM_OUT_PIN  14
#define PWM_CHANNEL   0
#define MODE_PIN     12

WiFiServer server(80);

int pwmMotor = 100;
bool useBluetooth = false;
bool lastMode = false;

bool initCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;
  return esp_camera_init(&config) == ESP_OK;
}

void startWiFiCamera() {
  if (!initCamera()) {
    Serial.println("Error inicializando la cámara");
  }
  server.begin();
  Serial.println("Cámara y servidor WiFi iniciados");
}

void startBluetooth() {
  Dabble.begin("ESP32CAM_BT");
  ledcSetup(PWM_CHANNEL, 1000, 8);
  ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);
  Serial.println("Bluetooth y PWM iniciados");
}

void stopBluetooth() {
  Serial.println("Bluetooth desactivado");
}

void setup() {
  Serial.begin(115200);
  pinMode(MODE_PIN, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WiFi conectado");
  Serial.println(WiFi.localIP());

  startWiFiCamera();

  useBluetooth = digitalRead(MODE_PIN) == HIGH;
  lastMode = useBluetooth;

  if (useBluetooth) {
    startBluetooth();
  }
}

void loop() {
  useBluetooth = digitalRead(MODE_PIN) == HIGH;

  if (useBluetooth != lastMode) {
    Serial.println("Cambio de modo detectado");

    if (useBluetooth) {
      Serial.println("Cambiando a modo Bluetooth");
      startBluetooth();
    } else {
      Serial.println("Cambiando a modo WiFi (Bluetooth apagado)");
      stopBluetooth();
      ledcWrite(PWM_CHANNEL, 0); 
    }

    lastMode = useBluetooth;
  }

  Serial.print(" Modo actual: ");
  Serial.println(useBluetooth ? "Bluetooth" : "WiFi");

  WiFiClient client = server.available();
  if (client) {
    while (!client.available()) delay(1);
    String req = client.readStringUntil('\r');
    while (client.available()) client.read();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();

    while (client.connected() && digitalRead(MODE_PIN) == LOW) {  
      camera_fb_t* fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println(" Error capturando imagen");
        break;
      }

      client.println("--frame");
      client.println("Content-Type: image/jpeg");
      client.printf("Content-Length: %d\r\n\r\n", fb->len);
      client.write(fb->buf, fb->len);
      client.println();

      esp_camera_fb_return(fb);
      delay(1);
    }

    client.stop();
  }

  if (useBluetooth) {
    Dabble.processInput();

    int pwmValue = 0;

    if (GamePad.isUpPressed() || GamePad.getJoystickData('Y') > 3) {
      pwmValue = map(pwmMotor, 0, 255, 200, 255);
    } else if (GamePad.isDownPressed() || GamePad.getJoystickData('Y') < -3) {
      pwmValue = map(pwmMotor, 0, 255, 100, 150);
    } else if (GamePad.isSquarePressed()) {
      pwmValue = map(pwmMotor, 0, 255, 50, 100);
    } else if (GamePad.isCirclePressed()) {
      pwmValue = map(pwmMotor, 0, 255, 150, 200);
    } else {
      pwmValue = 0;
    }

    if (GamePad.isTrianglePressed()) {
      pwmValue = 15;
      Serial.println(" Aumentar velocidad");
    }

    if (GamePad.isCrossPressed()) {
      pwmValue = 25;
      Serial.println(" Disminuir velocidad");
    }

    Serial.printf("[ESP32-CAM] PWM enviado: %d\n", pwmValue);
    ledcWrite(PWM_CHANNEL, pwmValue);
  }

  delay(100); 
}
