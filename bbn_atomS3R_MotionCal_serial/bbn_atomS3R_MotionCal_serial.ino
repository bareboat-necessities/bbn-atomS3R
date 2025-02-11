#include <M5AtomS3.h>
#include <M5Unified.h>
#include <Arduino.h>

/*
   Sketch to upload IMU data into MotionCal GUI: https://github.com/PaulStoffregen/MotionCal
   or https://github.com/bareboat-necessities/bbn-MotionCal
*/

const char* imu_name;

unsigned long last_update = 0UL, now = 0UL;

int samples = 0;

void read_and_processIMU_data() {
  /*
  m5::imu_3d_t accel;
  M5.Imu.getAccel(&accel.x, &accel.y, &accel.z);

  m5::imu_3d_t gyro;
  M5.Imu.getGyro(&gyro.x, &gyro.y, &gyro.z);

  m5::imu_3d_t mag;
  M5.Imu.getMag(&mag.x, &mag.y, &mag.z);
  */

  samples++;
  if (samples >= 1) {
    samples = 0;

    // Send raw sensor data to the MotionCal GUI
    // Format: "Raw:<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>,<mag_x>,<mag_y>,<mag_z>"
    Serial.print("Raw:");
    for (int i = 0; i < 9; i++) {
      Serial.print(M5.Imu.getRawData(i));
      if (i < 8) Serial.print(",");
    }
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
  //USB.begin();
  Serial.begin();
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
  Serial.println("MotionCal calibration sketch started.");
  last_update = micros();
}

void loop() {
  AtomS3.update();
  if (Serial.available()) {
    String data = Serial.readString();
    Serial.print("Received: ");
    Serial.println(data);
  }
  else {
    //repeatMe();
    delayMicroseconds(50000);
  }
}
