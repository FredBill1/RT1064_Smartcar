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
#include "utils/MQueue.hpp"

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
    const PIN_enum INT;
    uint8_t _buf[INV_MAX_SERIAL_WRITE + 1];

 protected:
    static int spi_read(void* context, uint8_t reg, uint8_t* buf, uint32_t len);
    static int spi_write(void* context, uint8_t reg, const uint8_t* buf, uint32_t len);
    static void build_sensor_event_data(void* context, enum inv_icm20948_sensor sensortype, uint64_t timestamp,
                                        const void* data, const void* arg);
    void build_sensor_event_data(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void* data,
                                 const void* arg);

    union {
        rosRT::msgs::QuaternionStamped quaternion_stamped;
    } _msg_buf;

    rosRT::Publisher _pub_uncal_gyro{"imu/uncal_gyro", sizeof(int), 1};
    rosRT::Publisher _pub_uncal_mag{"imu/uncal_mag", sizeof(int), 1};
    rosRT::Publisher _pub_gyro{"imu/gyro", sizeof(int), 1};
    rosRT::Publisher _pub_gravity{"imu/gravity", sizeof(int), 1};
    rosRT::Publisher _pub_linear_accel{"imu/linear_accel", sizeof(int), 1};
    rosRT::Publisher _pub_accel{"imu/accel", sizeof(int), 1};
    rosRT::Publisher _pub_mag{"imu/accel", sizeof(int), 1};
    rosRT::Publisher _pub_mag_orientation{"imu/mag_orientation", sizeof(int), 1};
    rosRT::Publisher _pub_6DOF_orientation{"imu/6DOF_orientation", sizeof(rosRT::msgs::QuaternionStamped), 1};
    rosRT::Publisher _pub_9DOF_orientation{"imu/9DOF_orientation", sizeof(rosRT::msgs::QuaternionStamped), 1};

 public:
    //  Linear acceleration [x,y,z] in m/s^2
    float _acc[3];
    //  Gyroscope readings in degrees-per-second [x,y,z]
    float _gyro[3];
    //  Magnetometer readings [x,y,z] in
    float _mag[3];
    //  Gravity vector
    float _gv[3];
    //  Quaternions(w,x,y,z) and their accuracy
    float _quat9DOF[4];
    float _quat9DOFaccuracy;
    float _quat6DOF[4];
    float _quat6DOFaccuracy;

    using VECT3 = float[3];
    using VECT4 = float[4];
    struct VECT_BIAS {
        VECT3 vect; /**< x,y,z vector data */
        VECT3 bias; /**< x,y,z bias vector data (for uncal sensor variant) */
    };
    struct QUAT_ACCURACY {
        VECT4 quat;     /**< w,x,y,z quaternion data */
        float accuracy; /**< quaternion accuracy */
    };

    // SensorDataBuf<VECT_BIAS> UNCAL_GYROSCOPE;
    // SensorDataBuf<VECT_BIAS> UNCAL_MAGNETOMETER;
    // SensorDataBuf<VECT3> GYROSCOPE;
    // SensorDataBuf<VECT3> GRAVITY;
    // SensorDataBuf<VECT3> LINEAR_ACCELERATION;
    // SensorDataBuf<VECT3> ACCELEROMETER;
    // SensorDataBuf<VECT3> MAGNETOMETER;
    // SensorDataBuf<VECT4> GEOMAG_ROTATION_VECTOR;
    // SensorDataBuf<VECT4> ROTATION_VECTOR;
    // SensorDataBuf<VECT4> GAME_ROTATION_VECTOR;

 public:
    ICM20948(SPIN_enum spi_n, SPI_PIN_enum sck, SPI_PIN_enum mosi, SPI_PIN_enum miso, SPI_PIN_enum cs, PIN_enum Int);
    void setMagnetometerBias(float biasX, float biasY, float biasZ);
    int setup();
    int selftest();
    void init();

    int enableSensor(inv_icm20948_sensor sensor);
    int disableSensor(inv_icm20948_sensor sensor);
    int setSensorPeriod(inv_icm20948_sensor sensor, uint32_t period);

    int readSensor();
};

#endif  // _ICM20948_hpp