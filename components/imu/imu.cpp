#include "imu.h"

#include <esp_log.h>

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

void IMU::TimerCallback(void *arg) {
  IMU *imu = (IMU *)arg;
  MPU6050 *mpu = &imu->mpu;

  Quaternion q;              // [w, x, y, z]         quaternion container
  VectorFloat gravity;       // [x, y, z]            gravity vector
  float ypr[3];              // [yaw, pitch, roll]   yaw/pitch/roll container
  uint16_t packetSize = 42;  // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;        // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64];    // FIFO storage buffer
  uint8_t mpuIntStatus;      // holds actual interrupt status byte from MPU

  mpuIntStatus = mpu->getIntStatus();
  fifoCount = mpu->getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu->resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    mpu->getFIFOBytes(fifoBuffer, packetSize);
    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

    IMUData data{};
    data.gravity = gravity;
    data.orientation = VectorFloat(ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG,
                                   ypr[0] * RAD_TO_DEG);
    mpu->dmpGetAccel(&data.linear_acc, fifoBuffer);
    mpu->dmpGetGyro(&data.ang_vel, fifoBuffer);

    for (auto &&cb : imu->callbacks) cb.second(data);
  }
}

IMU::~IMU() {
  for (auto &&cb : callbacks) unsubscribe(cb.first);
}

void IMU::init() {
  mpu.initialize();
  mpu.dmpInitialize();

  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788);
  mpu.CalibrateAccel(6);
  mpu.calibrateGyro(6);

  mpu.setDMPEnabled(true);
}

void IMU::start(uint64_t period_us) {
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = IMU::TimerCallback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "periodic",
  };

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, period_us));
}

void IMU::stop() { ESP_ERROR_CHECK(esp_timer_stop(periodic_timer)); }

void IMU::subscribe(std::string id, IMUDataCallback callback) {
  // add callback to map
  callbacks[id] = callback;
}

void IMU::unsubscribe(std::string id) {
  // remove callback from map
  if (callbacks.find(id) != callbacks.end()) {
    callbacks.erase(id);
  }
}
