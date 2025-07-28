  #include <WiFi.h>
  #include <micro_ros_arduino.h>
  #include <rcl/rcl.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <std_msgs/msg/bool.h>
  #include <std_msgs/msg/float32_multi_array.h>
  #include <rosidl_runtime_c/string_functions.h>
  #include <VL53L0X.h>
  #include <Adafruit_INA219.h>
  #include <Wire.h>
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>

  char ssid[]     = "ML-042";
  char password[] = "Leffersito";
  char agent_ip[] = "192.168.0.103";
  const uint16_t agent_port = 8888;

  #define STBY_PIN  23  
  #define AIN1_PIN  4
  #define AIN2_PIN  2
  #define PWMA_PIN 15
  #define BIN1_PIN  5
  #define BIN2_PIN 18
  #define PWMB_PIN 19
  #define PWMA_CH 0
  #define PWMB_CH 1
  #define ACT1_PIN 13  
  #define ACT1_CH 2
  #define ACT2_CH 3
  #define ENC_A_CHA 35
  #define ENC_A_CHB 32
  #define ENC_B_CHA 33
  #define ENC_B_CHB 25
  #define LED_CONTROL    26
  #define LED_MICROROS   27
  #define BUZZER_PIN     12
  #define BUZZER_CH       0
  #define PCA_ADDR 0x70
  #define BTN_PIN 39
  #define TX2_PIN 17 
  #define PWM_IN_PIN 14

  int velocidad = 150;  
  bool controlMode = true;  
  bool lastBtnState = HIGH;
  uint8_t clickCount = 0;
  uint32_t pressStart = 0, lastEdgeTime = 0, firstClickTime = 0;
  bool longPressHandled = false;
  int enc_a_cha_val, enc_a_chb_val;
  int enc_b_cha_val, enc_b_chb_val;
  volatile int enc_a_A_val = 0;
  volatile int enc_b_A_val = 0;
  volatile int enc_a_B_val = 0;
  volatile int enc_b_B_val = 0;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;
  rcl_subscription_t sub_stby;
  rcl_subscription_t sub_motor_a;
  rcl_subscription_t sub_motor_b;
  rcl_publisher_t pub_imu;
  std_msgs__msg__Float32MultiArray msg_imu;
  Adafruit_MPU6050 mpu;
  uint32_t last_imu_time = 0;
  const uint32_t IMU_PERIOD_MS = 200;
  rcl_publisher_t pub_encoders;
  std_msgs__msg__Float32MultiArray msg_encoders;
  uint32_t last_enc_time = 0;
  const uint32_t ENC_PERIOD_MS = 100;
  rcl_publisher_t pub_lidar;     
  rcl_publisher_t pub_distance;  
  rcl_publisher_t pub_power;     
  std_msgs__msg__Float32MultiArray msg_distance;
  std_msgs__msg__Float32MultiArray msg_power;
  rcl_subscription_t sub_actuators;
  std_msgs__msg__Float32MultiArray msg_actuators;
  std_msgs__msg__Bool msg_stby;
  std_msgs__msg__Float32MultiArray msg_motor_a;
  std_msgs__msg__Float32MultiArray msg_motor_b;

  VL53L0X vl53;
  Adafruit_INA219 ina219;

  #define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { Serial.printf("❌ ERROR en línea %d: Código %d\n", __LINE__, rc); return; } }
  #define RCSOFT(fn)  { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { Serial.printf("-- soft err %d\n", rc); } }

  void stby_cb(const void *msg_in) {
    bool en = ((std_msgs__msg__Bool *)msg_in)->data;
    Serial.printf("[STBY] %s\n", en ? "ON" : "OFF");
    digitalWrite(STBY_PIN, en);
  }

  void motor_a_cb(const void *msg_in) {
    auto *msg = (std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size < 3) return;
    float pwm = msg->data.data[0];
    bool ain1 = msg->data.data[1];
    bool ain2 = msg->data.data[2];

    Serial.printf("[MOTOR A] PWM: %.0f | AIN1: %d | AIN2: %d\n", pwm, ain1, ain2);

    digitalWrite(AIN1_PIN, ain1);
    digitalWrite(AIN2_PIN, ain2);
    ledcWrite(PWMA_CH, (int)pwm);
  }

  void motor_b_cb(const void *msg_in) {
    auto *msg = (std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size < 3) return;
    float pwm = msg->data.data[0];
    bool bin1 = msg->data.data[1];
    bool bin2 = msg->data.data[2];

    Serial.printf("[MOTOR B] PWM: %.0f | BIN1: %d | BIN2: %d\n", pwm, bin1, bin2);

    digitalWrite(BIN1_PIN, bin1);
    digitalWrite(BIN2_PIN, bin2);
    ledcWrite(PWMB_CH, (int)pwm);
  }

  void beep(int freq, int ms) {
    ledcWriteTone(BUZZER_CH, freq);
    delay(ms);
    ledcWriteTone(BUZZER_CH, 0);
    delay(40);
  }

  void soundReset()        { beep(1000, 300); }
  void soundControlMode()  { beep(1500, 150); beep(1500, 150); }
  void soundMicroROSMode() { beep(2000, 100); beep(2000, 100); beep(2000, 100); }

  void adelante(int pwmA, int pwmB) {
    digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWMA_CH, pwmA); ledcWrite(PWMB_CH, pwmB);
  }

  void atras(int pwmA, int pwmB) {
    digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, LOW); digitalWrite(BIN2_PIN, HIGH);
    ledcWrite(PWMA_CH, pwmA); ledcWrite(PWMB_CH, pwmB);
  }

  void derecha(int pwmA, int pwmB) {
    digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, LOW);  digitalWrite(BIN2_PIN, HIGH);
    ledcWrite(PWMA_CH, pwmA); ledcWrite(PWMB_CH, pwmB);
  }

  void izquierda(int pwmA, int pwmB) {
    digitalWrite(AIN1_PIN, LOW);  digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWMA_CH, pwmA); ledcWrite(PWMB_CH, pwmB);
  }

  void parar() {
    digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, LOW); digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWMA_CH, 0); ledcWrite(PWMB_CH, 0);
  }

  void setControlMode() {
    controlMode = true;
    digitalWrite(TX2_PIN, HIGH);
    digitalWrite(LED_CONTROL, HIGH);
    digitalWrite(LED_MICROROS, LOW);
    Serial.println(" Control Mode (Bluetooth) activo");
    soundControlMode();
  }

  void setMicroROSMode() {
    controlMode = false;
    digitalWrite(TX2_PIN, LOW);
    digitalWrite(LED_CONTROL, LOW);
    digitalWrite(LED_MICROROS, HIGH);
    Serial.println(" microROS Mode (WiFi) activo");
    soundMicroROSMode();
  }

  void checkButton() {
    bool btnState = digitalRead(BTN_PIN);
    uint32_t now = millis();

    if (lastBtnState == HIGH && btnState == LOW && (now - lastEdgeTime > 40)) {
      pressStart = now;
      lastEdgeTime = now;
      longPressHandled = false;
    }

    if (btnState == LOW && !longPressHandled && (now - pressStart >= 2000)) {
      Serial.println(" Long press → Reset");
      soundReset();  
      delay(100);
      ESP.restart();  
      longPressHandled = true;
    }

    if (lastBtnState == LOW && btnState == HIGH && (now - lastEdgeTime > 40)) {
      lastEdgeTime = now;
      if (!longPressHandled) {
        clickCount++;
        if (clickCount == 1) firstClickTime = now;  
      }
    }

    if (clickCount > 0 && (now - firstClickTime > 600)) {
      if (clickCount == 1) {
        setControlMode();   
      }
      else if (clickCount == 2) {
        setMicroROSMode(); 
      }
      else {
        Serial.printf("ℹ️  %u clicks (sin acción)\n", clickCount);
      }
      clickCount = 0;
    }

    lastBtnState = btnState;
  }

  void actuators_cb(const void *msg_in) {
    auto *msg = (std_msgs__msg__Float32MultiArray *)msg_in;
    if (msg->data.size < 2) return;

    float pwm1 = msg->data.data[0];
    float pwm2 = msg->data.data[1];

    Serial.printf("[ACTUATORS] PWM1: %.0f | PWM2: %.0f\n", pwm1, pwm2);

    ledcWrite(ACT1_CH, (int)pwm1);
  }

  void selectChannel(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(10);  
  }

  void setup() {
    Serial.begin(115200); delay(300);
    pinMode(PWM_IN_PIN, INPUT);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, LOW);
    pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT);
    ledcSetup(PWMA_CH, 1000, 8); ledcAttachPin(PWMA_PIN, PWMA_CH);
    ledcSetup(PWMB_CH, 1000, 8); ledcAttachPin(PWMB_PIN, PWMB_CH);
    ledcSetup(ACT1_CH, 1000, 8);
    ledcAttachPin(ACT1_PIN, ACT1_CH);
    ledcSetup(BUZZER_CH, 1000, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);
    pinMode(LED_CONTROL, OUTPUT);
    pinMode(LED_MICROROS, OUTPUT);
    digitalWrite(LED_CONTROL, LOW);
    digitalWrite(LED_MICROROS, LOW);
    pinMode(ENC_A_CHA, INPUT);
    pinMode(ENC_A_CHB, INPUT);
    pinMode(ENC_B_CHA, INPUT);
    pinMode(ENC_B_CHB, INPUT);
    pinMode(BTN_PIN, INPUT);
    pinMode(TX2_PIN, OUTPUT);
    setMicroROSMode();

    Wire.begin();
    selectChannel(7);  
    if (!mpu.begin()) {
      Serial.println(" MPU6050 no encontrado. Verifica la conexión.");
      while (1) delay(100);
    }

    for (int ch : {5, 6, 0, 1}) {
      selectChannel(ch);
      if (!vl53.init()) {
        Serial.printf(" VL53L0X no detectado en canal %d\n", ch);
      } else {
        vl53.setTimeout(500);
        vl53.startContinuous();
        Serial.printf(" VL53L0X iniciado en canal %d\n", ch);
      }
    }

    selectChannel(2);
    if (!ina219.begin()) {
      Serial.println(" INA219 no detectado en canal 2");
    } else {
      Serial.println(" INA219 listo en canal 2");
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(" IMU inicializado correctamente");
    Serial.print("Conectando Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(300); Serial.print(".");
    }
    Serial.printf(" OK (%s)\n", WiFi.localIP().toString().c_str());
    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "robot_fira_4_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&pub_lidar, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "lidar_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_distance);
    msg_distance.data.data = (float*)malloc(4 * sizeof(float));
    msg_distance.data.size = 4;
    msg_distance.data.capacity = 4;
    RCCHECK(rclc_publisher_init_default(&pub_distance, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "distance_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_power);
    msg_power.data.data = (float*)malloc(2 * sizeof(float));  // Voltaje, Corriente
    msg_power.data.size = 2;
    msg_power.data.capacity = 2;
    RCCHECK(rclc_publisher_init_default(&pub_power, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "power_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_encoders);
    msg_encoders.data.data = (float*) malloc(4 * sizeof(float));
    msg_encoders.data.size = 4;
    msg_encoders.data.capacity = 4;
    RCCHECK(rclc_publisher_init_default(&pub_encoders,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),"encoders_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_imu);
    msg_imu.data.data = (float*)malloc(6 * sizeof(float));
    msg_imu.data.size = 6;
    msg_imu.data.capacity = 6;
    RCCHECK(rclc_publisher_init_default(&pub_imu,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),"imu_4" ));
    std_msgs__msg__Bool__init(&msg_stby);
    RCCHECK(rclc_subscription_init_default(&sub_stby, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),"stby_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_motor_a);
    msg_motor_a.data.data = (float*)malloc(3 * sizeof(float));
    msg_motor_a.data.size = 3; msg_motor_a.data.capacity = 3;
    RCCHECK(rclc_subscription_init_default(&sub_motor_a, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),"motor_a_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_motor_b);
    msg_motor_b.data.data = (float*)malloc(3 * sizeof(float));
    msg_motor_b.data.size = 3; msg_motor_b.data.capacity = 3;
    RCCHECK(rclc_subscription_init_default(&sub_motor_b, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),"motor_b_4"));
    std_msgs__msg__Float32MultiArray__init(&msg_actuators);
    msg_actuators.data.data = (float*) malloc(3 * sizeof(float));
    msg_actuators.data.size = 2;
    msg_actuators.data.capacity = 2;
    RCCHECK(rclc_subscription_init_default(&sub_actuators,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),"actuators_4"));
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor,&sub_actuators,&msg_actuators,actuators_cb,ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_stby, &msg_stby, stby_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_a, &msg_motor_a, motor_a_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_b, &msg_motor_b, motor_b_cb, ON_NEW_DATA));
    Serial.println(" Listo. Esperando mensajes...");
  }

  void loop() {

    checkButton();

    if (controlMode) {
        int pwmValue = pulseIn(PWM_IN_PIN, HIGH, 25000);
        Serial.printf("[ESP32 Normal] PWM recibido: %d\n", pwmValue);

        if (pwmValue >= 50 && pwmValue <= 70) {  
            velocidad += 10;
            if (velocidad > 255) velocidad = 255;
            Serial.printf(" Velocidad aumentada a: %d\n", velocidad);
            parar();
        } else if (pwmValue >= 90 && pwmValue <= 110) {  
            velocidad -= 10;
            if (velocidad < 0) velocidad = 0;
            Serial.printf(" Velocidad disminuida a: %d\n", velocidad);
            parar();
        } else if (pwmValue > 800) {  
            adelante(velocidad, velocidad);
        } else if (pwmValue > 600) {  
            derecha(velocidad, velocidad);
        } else if (pwmValue > 400) { 
            atras(velocidad, velocidad);
        } else if (pwmValue > 200) {  
            izquierda(velocidad, velocidad);
        } else {  
            parar();
        }

        delay(100);  
        return;
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    uint32_t now = millis();
    if (now - last_imu_time >= IMU_PERIOD_MS) {
      selectChannel(7); 
      sensors_event_t acc, gyro, temp;
      mpu.getEvent(&acc, &gyro, &temp);
      msg_imu.data.data[0] = acc.acceleration.x;
      msg_imu.data.data[1] = acc.acceleration.y;
      msg_imu.data.data[2] = acc.acceleration.z;
      msg_imu.data.data[3] = gyro.gyro.x;
      msg_imu.data.data[4] = gyro.gyro.y;
      msg_imu.data.data[5] = gyro.gyro.z;
      Serial.printf("IMU | Acc[%.2f %.2f %.2f] | Gyro[%.2f %.2f %.2f]\n",
        msg_imu.data.data[0], msg_imu.data.data[1], msg_imu.data.data[2],
        msg_imu.data.data[3], msg_imu.data.data[4], msg_imu.data.data[5]);
      RCSOFT(rcl_publish(&pub_imu, &msg_imu, NULL));
      last_imu_time = now;
    }

    if (now - last_enc_time >= ENC_PERIOD_MS) {
      enc_a_cha_val = digitalRead(ENC_A_CHA);
      enc_a_chb_val = digitalRead(ENC_A_CHB);
      enc_b_cha_val = digitalRead(ENC_B_CHA);
      enc_b_chb_val = digitalRead(ENC_B_CHB);
      msg_encoders.data.data[0] = enc_a_cha_val;
      msg_encoders.data.data[1] = enc_a_chb_val;
      msg_encoders.data.data[2] = enc_b_cha_val;
      msg_encoders.data.data[3] = enc_b_chb_val;
      Serial.printf("ENC | A:[%d %d]  B:[%d %d]\n",
        enc_a_cha_val, enc_a_chb_val, enc_b_cha_val, enc_b_chb_val);
      RCSOFT(rcl_publish(&pub_encoders, &msg_encoders, NULL));
      last_enc_time = now;
    }

    static uint32_t last_dist_time = 0;
    if (now - last_dist_time >= 200) {
      int chs[] = {5, 6, 0, 1};
      for (int i = 0; i < 4; i++) {
        selectChannel(chs[i]);
        msg_distance.data.data[i] = vl53.readRangeContinuousMillimeters();
      }
      RCSOFT(rcl_publish(&pub_distance, &msg_distance, NULL));
      Serial.printf("DIST | %d %d %d %d\n",
        (int)msg_distance.data.data[0],
        (int)msg_distance.data.data[1],
        (int)msg_distance.data.data[2],
        (int)msg_distance.data.data[3]);
      last_dist_time = now;
    }

    static uint32_t last_power_time = 0;
    if (now - last_power_time >= 500) {
      selectChannel(2);
      msg_power.data.data[0] = ina219.getBusVoltage_V();
      msg_power.data.data[1] = ina219.getCurrent_mA();
      RCSOFT(rcl_publish(&pub_power, &msg_power, NULL));
      Serial.printf("POWER | V: %.2f | I: %.2f\n",
        msg_power.data.data[0],
        msg_power.data.data[1]);
      last_power_time = now;
    }

    std_msgs__msg__Float32MultiArray msg_lidar;
    std_msgs__msg__Float32MultiArray__init(&msg_lidar);
    msg_lidar.data.size = 0;
    msg_lidar.data.capacity = 0;
    msg_lidar.data.data = NULL;
    RCSOFT(rcl_publish(&pub_lidar, &msg_lidar, NULL));

    delay(10);
  }
