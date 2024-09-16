#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdio.h>
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <NoDelay.h>
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// #include "Wire.h"
// #endif

// CONSTANTS
#define INTERRUPT_PIN 2
#define INTERNAL_LED_PIN 13
#define DELAY_MS 5
#define TIMER_TIMEOUT_MS 1000
#define RX_PIN 16
#define TX_PIN 17
#define TX_BUFFER_SIZE 16
#define RX_BUFFER_SIZE 255


// MOTOR CONSTANTS
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 8;
const float NO_LOAD_SPEED = 7.50f;
const float STALL_TORQUE = 1.05f;

#define DEBUG_MOTOR
#define DEBUG_IMU

// Motor constants
float prev_yaw = .0f;
float curr_yaw = .0f;
unsigned long timer_cb_curr_time;
unsigned long timer_cb_prev_time;

unsigned long motor_cb_curr_time;
unsigned long motor_cb_prev_time;

unsigned long prev_time = 0;
unsigned long curr_time = 0;
float yaw_velocity = .0f;

struct Motor {
  int encoder;  // yellow
  int pwm;      // blue
  int dir;      // white
};

volatile int pulse_count1 = 0;
volatile int pulse_count2 = 0;

const Motor motor1 = { 18, 5, 19 };   // left
const Motor motor2 = { 12, 14, 27 };  // right


TaskHandle_t task_handle;  // gives the ability to handle a task
noDelay Motortimer(5);     //Creats a noDelay varible set to 1000ms

// IMU CONSTANTS
MPU6050 mpu;
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;               // set true if DMP init was successful
uint8_t mpuIntStatus;                // holds actual interrupt status byte from MPU
uint8_t devStatus;                   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                 // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                  // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];              // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Function prototypes
void setup_mpu();
void setup_motors();
void setup_rtos();


void task_imu(void *pvParameters);
void task_motor(void *pvParameters);
void motor_cmd_cb(const void *msgin);
void apply_torque(float torque, const Motor &motor);

void test_imu();
void test_motors();

void setup() {
  // configure LED for output
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  digitalWrite(INTERNAL_LED_PIN, HIGH);
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  setup_mpu();
  setup_motors();
  // uart  config
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  prev_time = millis();
  timer_cb_curr_time = millis();
  motor_cb_curr_time = millis();
  setup_rtos();
}

void loop() {
  delay(DELAY_MS);
  // test_imu();
}


void setup_rtos() {
  xTaskCreatePinnedToCore(task_imu, "task_imu", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(task_motor, "task_motor", 4096, NULL, 5, NULL, 0);
}


void update_imu_data() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

void setup_mpu() {
  Serial.println("Setup : IMU");
  pinMode(INTERRUPT_PIN, INPUT);
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // TODO : IMPLEMENT ERROR  HANDLING MECHANISM

  // gyro offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // factory setting is 1688;

  // check if Initialization was successful
  if (devStatus != 0) {
    Serial.printf("DMP Init Failde code :%d", devStatus);
    //
  }
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.print("Enabling interrupt detection (Arduino external interrupt ");
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println("...");
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void setup_motors() {
  // left
  pinMode(motor1.encoder, INPUT_PULLUP);
  pinMode(motor2.encoder, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor1.encoder), ISR_L_ENCODER_CB, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.encoder), ISR_R_ENCODER_CB, RISING);

  ledcAttach(motor1.pwm, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motor2.pwm, PWM_FREQ, PWM_RESOLUTION);

  pinMode(motor1.dir, OUTPUT);
  pinMode(motor2.dir, OUTPUT);

  ledcWrite(motor1.pwm, 255);
  ledcWrite(motor2.pwm, 255);

  delay(1000);

  ledcWrite(motor1.pwm, 0);
  ledcWrite(motor2.pwm, 0);
}





// ================================================================
// ===               CALLBACKS                 ===
// ================================================================

void task_imu(void *pvParameters) {
  char send_buf[TX_BUFFER_SIZE];
  while (1) {
    if (!dmpReady) {
      continue;
    }

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      continue;
    }
    timer_cb_curr_time = millis();

    update_imu_data();
    curr_yaw = ypr[0];
    float dt = (timer_cb_curr_time - timer_cb_prev_time) * 0.001f;
    yaw_velocity = (curr_yaw - prev_yaw) / dt;

    //send the data
    // clear the buffer
    for (size_t i = 0; i < TX_BUFFER_SIZE; i++) {
      send_buf[i] = 0x00;
    }
    send_buf[0] = 's';  // starting character
    send_buf[1] = 'f';  // the type of data being sent
    send_buf[2] = 2;    // sending 2 values, roll and angular velocity
    memcpy(send_buf + 3, ypr + 1, sizeof(float));
    memcpy(send_buf + 7, &yaw_velocity, sizeof(float));
    send_buf[15] = 'e';
    Serial2.write(send_buf, TX_BUFFER_SIZE);
#ifdef DEBUG_IMU
    // float time_step = (float)last_call_time * 0.000001f;
    // printf("Last callback time: %ld\n", time_step);
    // Serial.printf(" IMU VALUES :\tTime step : %f\t Quaternions :%f:%f:%f:%f:%f\t%f\tAngular velocity:%f\n", dt, q.x, q.y, q.z, q.w, yaw_velocity);
    Serial.printf(" IMU VALUES :\Roll : %f\t%f\n", ypr[1], yaw_velocity);
#endif

    prev_yaw = curr_yaw;
    timer_cb_prev_time = timer_cb_curr_time;

  }
}
void task_motor(void *pvParameters) {
  char rx_buf[RX_BUFFER_SIZE];
  int rx_bytes;
  float motor_cmd_1;
  float motor_cmd_2;

  while (1) {
    // clear buffer
    for (size_t i = 0; i < RX_BUFFER_SIZE; i++) {
      rx_buf[i] = 0x00;
    }
    rx_bytes = Serial2.readBytes(rx_buf, RX_BUFFER_SIZE);

    if (rx_bytes <= 0) {
      set_torque(0, motor1);
      set_torque(0, motor2);
      Serial.println("No bytes received");
      continue;
    }
    memcpy(&motor_cmd_1, rx_buf + 3, sizeof(float));
    memcpy(&motor_cmd_2, rx_buf + 7, sizeof(float));
#ifdef DEBUG_MOTOR
    Serial.printf("Received : %d\t %f\%f\n", rx_bytes, motor_cmd_1, motor_cmd_2);
#endif

      set_torque(motor_cmd_1, motor1);
      set_torque(motor_cmd_2, motor2);
  }
}
// ================================================================
// ===               ISRS               ===
// ================================================================

void dmpDataReady() {
  mpuInterrupt = true;
}
void ISR_L_ENCODER_CB() {
  pulse_count1++;
}

void ISR_R_ENCODER_CB() {
  pulse_count2;
}



void test_imu() {

  if (!dmpReady) {
    return;
  }

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    return;
  }
  update_imu_data();
  Serial.printf("IMU VALUES : %f\t%f\t%f\t%\t%f\tRoll : %f\n", q.x, q.y, q.z, q.w, ypr[1]);
}

void test_motors() {
  for (float i = -1.f; i < 1.f; i += 0.05f) {
    set_torque(i, motor1);

    set_torque(i, motor2);
    Serial.printf("Torque %f", i);
    delay(1000);
  }
  ledcWrite(motor1.pwm, 0);
  ledcWrite(motor2.pwm, 0);
}

void set_torque(float torque, const Motor &motor) {
  float speed = ((torque) / STALL_TORQUE);
  digitalWrite(motor.dir, (speed > 0) ? HIGH : LOW);

int torque_int = (int) (torque * 1023.0);
  int pwm = map(abs(torque_int), 0,1023, 80, 255);
  // int pwm = (int)((abs(speed) * 150.f) + 95.f);

  Serial.print("PWM: ");
  Serial.println(pwm);
  ledcWrite(motor.pwm, pwm);
}
