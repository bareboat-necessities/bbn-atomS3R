#include <M5AtomS3.h>
#include <M5Unified.h>
#include <Arduino.h>

#include "Quaternion.h"
#include "Mahony_AHRS.h"

Mahony_AHRS_Vars mahony;

const char* imu_name;

unsigned long last_update = 0UL, now = 0UL;

int samples = 0;

// Function to calculate the tilt-compensated compass heading in degrees
float calculate_heading(float mx, float my, float mz, float ax, float ay, float az) {
  float heading;
  float pitch, roll;

  // Calculate pitch (rotation around y-axis) in radians
  pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // Calculate roll (rotation around x-axis) in radians
  roll = atan2(ay, az);

  // Tilt compensation: adjust magnetometer readings for pitch and roll
  float xh = mx * cos(pitch) + mz * sin(pitch);
  float yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
  float zh = -mx * cos(roll) * sin(pitch) + my * sin(roll) + mz * cos(roll) * cos(pitch);

  // Calculate the heading in radians using the tilt-compensated x and y components
  heading = atan2(yh, xh);

  // Convert radians to degrees
  heading = heading * 180 / PI;

  // Normalize the heading to a range of 0 to 360 degrees
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

void read_and_processIMU_data() {

  m5::IMU_Class::imu_data_t data;
  M5.Imu.getImuData(&data);

  now = micros();
  float delta_t = (now - last_update) / 1000000.0;  // time step sec
  last_update = now;

  float pitch = .0f, roll = .0f, yaw = .0f;
  //Quaternion quaternion;

  mahony_AHRS_update_mag(&mahony, data.gyro.x * DEG_TO_RAD, data.gyro.y * DEG_TO_RAD, data.gyro.z * DEG_TO_RAD,
                         data.accel.x, data.accel.y, data.accel.z, data.mag.x, data.mag.y, data.mag.z,
                         &pitch, &roll, &yaw, delta_t);
  //mahony_AHRS_update(&mahony, data.gyro.x * DEG_TO_RAD, data.gyro.y * DEG_TO_RAD, data.gyro.z * DEG_TO_RAD,
  //                   data.accel.x, data.accel.y, data.accel.z,
  //                   &pitch, &roll, &yaw, delta_t);
  //Quaternion_set(mahony.q0, mahony.q1, mahony.q2, mahony.q3, &quaternion);

  if (yaw < 0) {
    yaw = -yaw;
  }
  else {
    yaw = 360.0 - yaw;
  }

  samples++;
  if (samples >= 10) {
    samples = 0;
    Serial.printf("head:%.4f", calculate_heading(data.mag.x, data.mag.y, data.mag.z, 
                                                 data.accel.x, data.accel.y, data.accel.z));
    Serial.printf(",yaw:%.4f", yaw);
    Serial.printf(",roll:%.4f", roll);
    Serial.printf(",pitch:%.4f", pitch);
    Serial.printf(",ax:%.4f", data.accel.x);
    Serial.printf(",ay:%.4f", data.accel.y);
    Serial.printf(",az:%.4f", data.accel.z);
    //Serial.printf(",gx:%.4f", data.gyro.x);
    //Serial.printf(",gy:%.4f", data.gyro.y);
    //Serial.printf(",gz:%.4f", data.gyro.z);
    Serial.printf(",mx:%.4f", data.mag.x);
    Serial.printf(",my:%.4f", data.mag.y);
    Serial.printf(",mz:%.4f", data.mag.z);
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

void printCalib() {
  size_t index = 0;
  Serial.println("Calibration data:");
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j, ++index) {
      Serial.printf("%.4d ", M5.Imu.getOffsetData(index));
    }
    Serial.println();
  }
}

void setup() {
  auto cfg = M5.config();
  AtomS3.begin(cfg);
  Serial.begin(38400);

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

  float twoKp = (2.0f * 8.0f);
  float twoKi = (2.0f * 0.0001f);
  mahony_AHRS_init(&mahony, twoKp, twoKi);

  if (!M5.Imu.loadOffsetFromNVS()) {
    //startCalibration();
  }
  else {
    printCalib();
  }
}

void loop() {
  AtomS3.update();
  repeatMe();
  delayMicroseconds(25000);
}
