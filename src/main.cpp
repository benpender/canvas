#include <Arduino.h>
#include <Wire.h>
#include <SensorQMI8658.hpp>

// Define the I2C pins for the Waveshare ESP32-S3-Matrix
#define IMU_SDA 11
#define IMU_SCL 12

// Define the sensor object
SensorQMI8658 qmi;

// Struct to hold sensor data
IMUdata acc;
IMUdata gyr;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Wait for Serial to be ready (useful for USB-C debugging)
  // You might want to remove this if you want it to run without a PC connected immediately
  delay(2000); 
  Serial.println("Starting ESP32-S3-Matrix IMU Demo...");

  // Initialize the QMI8658 sensor
  // Usage: .begin(WireInstance, Address, SDA_Pin, SCL_Pin)
  // Address is usually 0x6B for this board
  if (!qmi.begin(Wire, QMI8658_ADDRESS_HIGH, IMU_SDA, IMU_SCL)) {
    Serial.println("Failed to find QMI8658 - Check your wiring or pin definitions!");
    while (1) { delay(1000); }
  }

  Serial.println("QMI8658 Sensor found!");
  
  // Optional: Configure sensor (Rates, Scales, etc.)
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
  qmi.configGyroscope(SensorQMI8658::GYRO_RANGE_512DPS, SensorQMI8658::GYRO_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
  
  Serial.println("Sensor Configured. Reading data...");
}

void loop() {
  // Check if data is available from the sensor
  if (qmi.getDataReady()) {
    // Read the data into our structs
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z) && 
        qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
      
      // Print "Rotation" (Gyroscope dps)
      Serial.print("Rotation (Gyro) [x, y, z]: ");
      Serial.print(gyr.x); Serial.print(", ");
      Serial.print(gyr.y); Serial.print(", ");
      Serial.print(gyr.z);
      
      Serial.print("  |  ");

      // Print "Position/Tilt" (Accelerometer g)
      Serial.print("Tilt (Accel) [x, y, z]: ");
      Serial.print(acc.x); Serial.print(", ");
      Serial.print(acc.y); Serial.print(", ");
      Serial.println(acc.z);
    }
  }

  // Wait 1 second before next log
  delay(1000);
}
