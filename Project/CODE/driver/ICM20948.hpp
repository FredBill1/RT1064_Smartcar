#ifndef _ICM20948_hpp
#define _ICM20948_hpp

#undef MAX
#undef MIN
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Invn/Devices/Drivers/Icm20948/Icm20948.h"
#undef MAX
#undef MIN
#include <rtthread.h>
extern "C" {
#include "zf_spi.h"
}

class ICM20948 : private inv_icm20948_serif, protected inv_icm20948 {
 public:
    using callback_t = void (*)(const float* data, uint64_t timestamp_us);

 protected:
    void setupCallbacks();
    // clang-format off
    callback_t cb_uncal_mag        = nullptr; // float[6]: data[3], bias[3]
    callback_t cb_uncal_gyro       = nullptr; // float[6]: data[3], bias[3]
    callback_t cb_accel            = nullptr; // float data[3]
    callback_t cb_gyro             = nullptr; // float data[3]
    callback_t cb_mag              = nullptr; // float data[3]
    callback_t cb_gravity          = nullptr; // float data[3]
    callback_t cb_linear_accel     = nullptr; // float data[3]
    callback_t cb_rpy_orientation  = nullptr; // float data[3]
    callback_t cb_6DOF_orientation = nullptr; // float data[4]
    callback_t cb_9DOF_orientation = nullptr; // float data[4]
    callback_t cb_mag_orientation  = nullptr; // float data[4]
    // clang-format on

 private:
    const SPIN_enum SPI_N;
    const SPI_PIN_enum SCK, MOSI, MISO, CS;
    const PIN_enum INT;
    uint8_t _buf[INV_MAX_SERIAL_WRITE + 1];

 protected:
    static int spi_read(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
    static int spi_write(void* context, uint8_t reg, const uint8_t* buf, uint32_t len);
    static void build_sensor_event_data(void* context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void* data,
                                        const void* arg);
    void build_sensor_event_data(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void* data, const void* arg);

    int setup();
    int selftest();

 public:
    ICM20948(SPIN_enum spi_n, SPI_PIN_enum sck, SPI_PIN_enum mosi, SPI_PIN_enum miso, SPI_PIN_enum cs, PIN_enum Int);
    void init();

    int enableSensor(inv_icm20948_sensor sensor);
    int disableSensor(inv_icm20948_sensor sensor);
    int setSensorPeriod(inv_icm20948_sensor sensor, uint32_t period);
    int enableSetSensor(inv_icm20948_sensor sensor, uint32_t period);
    void enableSetSensors();

    int readSensor();
};

#endif  // _ICM20948_hpp