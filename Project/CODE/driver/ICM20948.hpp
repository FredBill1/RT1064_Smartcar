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
#include "rosRT/msgs/sensor_msgs.hpp"
#include "rosRT/rosRT.hpp"
#include "utils/ClassPlaceHolder.hpp"
#include "utils/MQueue.hpp"

class ICM20948 : private inv_icm20948_serif, protected inv_icm20948 {
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

    utils::ClassPlaceHolder<rosRT::Publisher> _pub_uncal_mag;         // 未校准磁场
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_uncal_gyro;        // 未校准陀螺仪
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_accel;             // 加速度计
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_gyro;              // 陀螺仪
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_mag;               // 磁场
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_gravity;           // 重力
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_linear_accel;      // 线加速度
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_rpy_orientation;   // 9DOF RPY
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_6DOF_orientation;  // 6DOF位姿
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_9DOF_orientation;  // 9DOF位姿
    utils::ClassPlaceHolder<rosRT::Publisher> _pub_mag_orientation;   // 地磁位姿

    int setup();
    int selftest();
    void initPubs();

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