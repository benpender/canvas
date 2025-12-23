#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <QMI8658.h>

struct IMUdata {
    float x;
    float y;
    float z;
};

class SensorQMI8658 {
public:
    enum AccRange {
        ACC_RANGE_2G  = QMI8658_ACCEL_RANGE_2G,
        ACC_RANGE_4G  = QMI8658_ACCEL_RANGE_4G,
        ACC_RANGE_8G  = QMI8658_ACCEL_RANGE_8G,
        ACC_RANGE_16G = QMI8658_ACCEL_RANGE_16G
    };

    enum AccODR {
        ACC_ODR_8000Hz  = QMI8658_ACCEL_ODR_8000HZ,
        ACC_ODR_4000Hz  = QMI8658_ACCEL_ODR_4000HZ,
        ACC_ODR_2000Hz  = QMI8658_ACCEL_ODR_2000HZ,
        ACC_ODR_1000Hz  = QMI8658_ACCEL_ODR_1000HZ,
        ACC_ODR_500Hz   = QMI8658_ACCEL_ODR_500HZ,
        ACC_ODR_250Hz   = QMI8658_ACCEL_ODR_250HZ,
        ACC_ODR_125Hz   = QMI8658_ACCEL_ODR_125HZ,
        ACC_ODR_62_5Hz  = QMI8658_ACCEL_ODR_62_5HZ,
        ACC_ODR_31_25Hz = QMI8658_ACCEL_ODR_31_25HZ
    };

    enum LpfMode {
        LPF_MODE_0 = 0,
        LPF_MODE_1,
        LPF_MODE_2
    };

    SensorQMI8658() = default;

    bool begin(TwoWire &wire, uint8_t address, int sdaPin, int sclPin) {
        wire.begin(sdaPin, sclPin);
        bus = &wire;
        return driver.begin(wire, address);
    }

    void configAccelerometer(AccRange range, AccODR odr, LpfMode /*lpfMode*/) {
        driver.setAccelRange(static_cast<QMI8658_AccelRange>(range));
        driver.setAccelODR(static_cast<QMI8658_AccelODR>(odr));
    }

    void enableAccelerometer() {
        driver.enableAccel(true);
    }

    void enableGyroscope() {
        driver.enableGyro(true);
    }

    bool getDataReady() {
        return driver.isDataReady();
    }

    bool getAccelerometer(float &x, float &y, float &z) {
        // Return acceleration in mg to match Arduino sketch expectations
        return driver.readAccelMG(x, y, z);
    }

private:
    TwoWire* bus = nullptr;
    QMI8658 driver;
};
