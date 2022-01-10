#ifndef _ICM20948_hpp
#define _ICM20948_hpp

#undef MAX
#undef MIN
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Invn/Devices/Drivers/Icm20948/Icm20948.h"
#undef MAX
#undef MIN

extern "C" {
#include "zf_spi.h"
}

enum OrientationDOF {
    Orientation6DOF,
    Orientation9DOF,
};

// Default = +/- 4g. Valid ranges: 2, 4, 8, 16
enum AccelerometerFSR {
    AccelFSR2g = 2,
    AccelFSR4g = 4,
    AccelFSR8g = 8,
    AccelFSR16g = 16,
};

// Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
enum GyroscopeFSR {
    GyroFSR250dps = 250,
    GyroFSR500dps = 500,
    GyroFSR1000dps = 1000,
    GyroFSR2000dps = 2000,
};

class ICM20948 : private inv_icm20948_serif, protected inv_icm20948 {
 private:
    const SPIN_enum SPI_N;
    const SPI_PIN_enum SCK, MOSI, MISO, CS;
    uint8_t _buf[INV_MAX_SERIAL_WRITE + 1];

 protected:
    static int spi_read(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
    static int spi_write(void* context, uint8_t reg, const uint8_t* buf, uint32_t len);
    static void build_sensor_event_data(void* context, enum inv_icm20948_sensor sensortype, uint64_t timestamp,
                                        const void* data, const void* arg);

 public:
    //  Linear acceleration [x,y,z] in m/s^2
    volatile float _acc[3];
    //  Gyroscope readings in degrees-per-second [x,y,z]
    volatile float _gyro[3];
    //  Magnetometer readings [x,y,z] in
    volatile float _mag[3];
    //  Gravity vector
    volatile float _gv[3];
    //  Quaternions(w,x,y,z) and their accuracy
    volatile float _quat9DOF[4];
    volatile float _quat9DOFaccuracy;
    volatile float _quat6DOF[4];
    volatile float _quat6DOFaccuracy;

 public:
    ICM20948(SPIN_enum spi_n, SPI_PIN_enum sck, SPI_PIN_enum mosi, SPI_PIN_enum miso, SPI_PIN_enum cs);
    int init();

    int enableSensor(inv_icm20948_sensor sensor, uint32_t period);
    int disableSensor(inv_icm20948_sensor sensor);
    int enableAllSensors(uint32_t period);

    int readSensor();
};

#endif  // _ICM20948_hpp