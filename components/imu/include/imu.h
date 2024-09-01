#pragma once

#include <esp_timer.h>

#include <functional>
#include <string>

#include "MPU6050.h"
#include "helper_3dmath.h"

struct IMUData {
  VectorFloat gravity;
  VectorInt16 linear_acc;
  VectorInt16 ang_vel;
  VectorFloat orientation;
};

typedef std::function<void(IMUData)> IMUDataCallback;

class IMU {
 public:
  IMU(const uint8_t dev_addr) : mpu(dev_addr) {};
  ~IMU();

  void init();
  void start(uint64_t period);
  void stop();

  void subscribe(std::string id, IMUDataCallback callback);
  void unsubscribe(std::string id);

 private:
  static void TimerCallback(void *arg);

  MPU6050 mpu;
  esp_timer_handle_t periodic_timer;
  std::unordered_map<std::string, IMUDataCallback> callbacks;
};
