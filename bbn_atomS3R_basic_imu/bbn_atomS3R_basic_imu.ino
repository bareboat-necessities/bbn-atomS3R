#include <M5AtomS3.h>
#include <M5Unified.h>
#include <Arduino.h>

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
    Serial.println();
  }

  now = micros();
  float delta_t = (now - last_update) / 1000000.0;  // time step sec
  last_update = now;
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
}

void loop() {
  AtomS3.update();
  repeatMe();
  delayMicroseconds(3000);
}
