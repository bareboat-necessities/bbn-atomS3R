#include <M5AtomS3.h>
#include <M5Unified.h>
#include <Arduino.h>

#include "Quaternion.h"
#include "Mahony_AHRS.h"

Mahony_AHRS_Vars mahony;

const char* imu_name;

unsigned long last_update = 0UL, now = 0UL;

int samples = 0;

void read_and_processIMU_data() {
  m5::imu_3d_t accel;
  M5.Imu.getAccel(&accel.x, &accel.y, &accel.z);

  m5::imu_3d_t gyro;
  M5.Imu.getGyro(&gyro.x, &gyro.y, &gyro.z);

  m5::imu_3d_t mag;
  M5.Imu.getMag(&mag.x, &mag.y, &mag.z);

  now = micros();
  float delta_t = (now - last_update) / 1000000.0;  // time step sec
  last_update = now;

  float pitch, roll, yaw;
  Quaternion quaternion;

  mahony_AHRS_update_mag(&mahony, gyro.x * DEG_TO_RAD, gyro.y * DEG_TO_RAD, gyro.z * DEG_TO_RAD,
                         accel.x, accel.y, accel.z, mag.x, mag.y, mag.z,
                         &pitch, &roll, &yaw, delta_t);
  Quaternion_set(mahony.q0, mahony.q1, mahony.q2, mahony.q3, &quaternion);

  samples++;
  if (samples >= 50) {
    samples = 0;
    Serial.printf("a.x:%.4f", accel.x);
    Serial.printf(",a.y:%.4f", accel.y);
    Serial.printf(",a.z:%.4f", accel.z);
    Serial.printf(", g.x:%.4f", gyro.x);
    Serial.printf(",g.y:%.4f", gyro.y);
    Serial.printf(",g.z:%.4f", gyro.z);
    Serial.printf(", m.x:%.4f", mag.x);
    Serial.printf(",m.y:%.4f", mag.y);
    Serial.printf(",m.z:%.4f", mag.z);
    Serial.printf(", pitch:%.4f", pitch);
    Serial.printf(",roll:%.4f", roll);
    Serial.printf(",yaw:%.4f", yaw);
    Serial.println();
  }
}

void repeatMe() {
  static uint32_t prev_sec = 0;
  auto imu_update = M5.Imu.update();
  if (imu_update) {
    read_and_processIMU_data();
  }
  int32_t sec = millis() / 1000;
  if (prev_sec != sec) {
    prev_sec = sec;
    if ((sec & 7) == 0) {
      // prevent WDT.
      vTaskDelay(1);
    }
  }
}

void setup() {
  auto cfg = M5.config();
  AtomS3.begin(cfg);
  Serial.begin(115200);

  auto imu_type = M5.Imu.getType();
  switch (imu_type) {
    case m5::imu_none:        imu_name = "not found";   break;
    case m5::imu_sh200q:      imu_name = "sh200q";      break;
    case m5::imu_mpu6050:     imu_name = "mpu6050";     break;
    case m5::imu_mpu6886:     imu_name = "mpu6886";     break;
    case m5::imu_mpu9250:     imu_name = "mpu9250";     break;
    case m5::imu_bmi270:      imu_name = "bmi270";      break;
    default:                  imu_name = "unknown";     break;
  };

  if (imu_type == m5::imu_none) {
    Serial.println("Imu not found!");
    for (;;) {
      delay(1);
    }
  }
  Serial.println(imu_name);
  last_update = micros();

  float twoKp = (2.0f * 1.0f);
  float twoKi = (2.0f * 0.0001f);
  mahony_AHRS_init(&mahony, twoKp, twoKi);
}

void loop() {
  AtomS3.update();
  repeatMe();
  delayMicroseconds(3000);
}
