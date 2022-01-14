#include "ICM20948.hpp"

#include <rthw.h>
#include <rtthread.h>

#include "ICM20948/dmp_img.hpp"
#include "ICM20948/macros.h"
#include "ICM20948/system.h"
#include "Invn/Devices/Drivers/ICM20948/Icm20948MPUFifoControl.h"
#include "Invn/Devices/SensorTypes.h"

extern "C" {
#include "fsl_debug_console.h"
#include "zf_gpio.h"
#include "zf_systick.h"
}

void ICM20948::init() {
    setMagnetometerBias(-143, -23, 180);
    setup();
    while (selftest()) setup();
    PRINTF("ICM20948: Init complete.\r\n");

    // clang-format off
    enableSetSensor(INV_ICM20948_SENSOR_GYROSCOPE,                   20 ); // 陀螺仪 1~225Hz
    enableSetSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION,         20 ); // 线加速度 50~255Hz (基于6DOF位姿和加速度计)
    enableSetSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR,             20 ); // 9DOF位姿 50~255Hz
    // enableSetSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, 100); // 地磁位姿 1~255Hz
    // enableSetSensor(INV_ICM20948_SENSOR_GRAVITY,                     20 ); // 重力 50~255Hz (基于6DOF位姿)
    // enableSetSensor(INV_ICM20948_SENSOR_ACCELEROMETER,               100); // 加速度计 1~225Hz
    // enableSetSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD,           100); // 磁场 1~70Hz
    // enableSetSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR,        20 ); // 6DOF位姿 50~255Hz
    // enableSetSensor(INV_ICM20948_SENSOR_ORIENTATION,                 20 ); // 9DOF RPY 50~255Hz (基于9DOF位姿)
    // enableSetSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 100); // 未校准磁场 1~255Hz
    // enableSetSensor(INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED,      100); // 未校准陀螺仪 1~255Hz
    //clang-format on

    gpio_interrupt_init(INT, RISING, GPIO_INT_CONFIG);
}

#define CHECK_RC(s) \
    if (rc) {       \
        PRINTF(s);  \
        return rc;  \
    }  // clang-format on

#define GRAVITY_CONST 9.8f

/* FSR configurations */
static int32_t cfg_acc_fsr = AccelFSR4g;      // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
static int32_t cfg_gyr_fsr = GyroFSR2000dps;  // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static float cfg_mounting_matrix[9]{
    1.f, 0,   0,    //
    0,   1.f, 0,    //
    0,   0,   1.f,  //
};

static int unscaled_bias[THREE_AXES * 2];

static int biasq16[3] = {0};

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX]  //
    {INV_SENSOR_TYPE_ACCELEROMETER,
     INV_SENSOR_TYPE_GYROSCOPE,
     INV_SENSOR_TYPE_RAW_ACCELEROMETER,
     INV_SENSOR_TYPE_RAW_GYROSCOPE,
     INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
     INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
     INV_SENSOR_TYPE_BAC,
     INV_SENSOR_TYPE_STEP_DETECTOR,
     INV_SENSOR_TYPE_STEP_COUNTER,
     INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
     INV_SENSOR_TYPE_ROTATION_VECTOR,
     INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
     INV_SENSOR_TYPE_MAGNETOMETER,
     INV_SENSOR_TYPE_SMD,
     INV_SENSOR_TYPE_PICK_UP_GESTURE,
     INV_SENSOR_TYPE_TILT_DETECTOR,
     INV_SENSOR_TYPE_GRAVITY,
     INV_SENSOR_TYPE_LINEAR_ACCELERATION,
     INV_SENSOR_TYPE_ORIENTATION,
     INV_SENSOR_TYPE_B2S};

static uint8_t icm20948_get_grv_accuracy(void) {
    uint8_t accel_accuracy;
    uint8_t gyro_accuracy;

    accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
    gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
    return (min(accel_accuracy, gyro_accuracy));
}

int ICM20948::spi_read(void* context, uint8_t reg, uint8_t* buf, uint32_t len) {
    ICM20948& self = *(ICM20948*)context;
    reg |= READ_BIT_MASK;
    self._buf[0] = reg;
    rt_base_t level = rt_hw_interrupt_disable();
    spi_mosi(self.SPI_N, self.CS, self._buf, self._buf, len + 1, 1);
    rt_hw_interrupt_enable(level);
    memcpy(buf, self._buf + 1, len);
    return 0;
}

int ICM20948::spi_write(void* context, uint8_t reg, const uint8_t* buf, uint32_t len) {
    ICM20948& self = *(ICM20948*)context;
    reg &= WRITE_BIT_MASK;
    self._buf[0] = reg;
    memcpy(self._buf + 1, buf, len);
    rt_base_t level = rt_hw_interrupt_disable();
    spi_mosi(self.SPI_N, self.CS, self._buf, self._buf, len + 1, 1);
    rt_hw_interrupt_enable(level);
    return 0;
}

ICM20948::ICM20948(SPIN_enum spi_n, SPI_PIN_enum sck, SPI_PIN_enum mosi, SPI_PIN_enum miso, SPI_PIN_enum cs, PIN_enum Int)
    : SPI_N(spi_n), SCK(sck), MOSI(mosi), MISO(miso), CS(cs), INT(Int) {
    context = this;
    read_reg = (decltype(read_reg))&ICM20948::spi_read;
    write_reg = (decltype(write_reg))&ICM20948::spi_write;
    max_read = max_write = 1024 * 16;
    is_spi = true;
    inv_icm20948_reset_states(this, this);
    inv_icm20948_register_aux_compass(this, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
}

void ICM20948::setMagnetometerBias(float biasX, float biasY, float biasZ) {
    biasq16[0] = (int)(biasX * (float)(1L << 16));
    biasq16[1] = (int)(biasY * (float)(1L << 16));
    biasq16[2] = (int)(biasZ * (float)(1L << 16));
}

int ICM20948::setup() {
    PRINTF("ICM20948: Running setup\r\n");
    systick_delay_ms(10);
    spi_init(SPI_N, SCK, MOSI, MISO, CS, 3, 7 * 1000 * 1000);
    int rc;

    // Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
    uint8_t whoami = 0xff;
    do {
        rc = inv_icm20948_get_whoami(this, &whoami);
        if (rc) rt_kprintf("ICM20948: WHOAMI error\r\n");
    } while (whoami != WHO_AM_I);

    // Setup accel and gyro mounting matrix and associated angle for current board
    inv_icm20948_init_matrix(this);

    // set default power mode
    rc = inv_icm20948_initialize(this, ICM20948_dmp_img, sizeof(ICM20948_dmp_img));
    CHECK_RC("ICM20948: DMP init error\r\n");

    inv_icm20948_register_aux_compass(this, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

    // Initialize auxiliary sensors
    rc = inv_icm20948_initialize_auxiliary(this);
    CHECK_RC("ICM20948: Compass not detected\r\n");

    // Set full-scale range for sensors
    for (int ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
        inv_icm20948_set_matrix(this, cfg_mounting_matrix, (inv_icm20948_sensor)ii);

    //  Smooth accel output
    // base_state.accel_averaging = 5;

    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void*)&cfg_acc_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_ACCELEROMETER, (const void*)&cfg_acc_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void*)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_GYROSCOPE, (const void*)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void*)&cfg_gyr_fsr);

    // Set GEOMAGNETIC bias
    inv_icm20948_set_bias(this, INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, biasq16);

    // re-initialize base state structure
    inv_icm20948_init_structure(this);

    // we should be good to go !

    // Now that Icm20948 device was initialized, we can proceed with DMP image loading
    // This step is mandatory as DMP image are not store in non volatile memory
    rc = inv_icm20948_load(this, ICM20948_dmp_img, sizeof(ICM20948_dmp_img));
    CHECK_RC("ICM20948: Error loading DMP3\r\n");

    return 0;
}

void inv_icm20948_get_st_bias(struct inv_icm20948* s, int* gyro_bias, int* accel_bias, int* st_bias, int* unscaled) {
    int axis, axis_sign;
    int gravity, gravity_scaled;
    int i, t;
    int check;
    int scale;

    /* check bias there ? */
    check = 0;
    for (i = 0; i < 3; i++) {
        if (gyro_bias[i] != 0) check = 1;
        if (accel_bias[i] != 0) check = 1;
    }

    /* if no bias, return all 0 */
    if (check == 0) {
        for (i = 0; i < 12; i++) st_bias[i] = 0;
        return;
    }

    /* dps scaled by 2^16 */
    scale = 65536 / DEF_SELFTEST_GYRO_SENS;

    /* Gyro normal mode */
    t = 0;
    for (i = 0; i < 3; i++) {
        st_bias[i + t] = gyro_bias[i] * scale;
        unscaled[i + t] = gyro_bias[i];
    }
    axis = 0;
    axis_sign = 1;
    if (INV20948_ABS(accel_bias[1]) > INV20948_ABS(accel_bias[0])) axis = 1;
    if (INV20948_ABS(accel_bias[2]) > INV20948_ABS(accel_bias[axis])) axis = 2;
    if (accel_bias[axis] < 0) axis_sign = -1;

    /* gee scaled by 2^16 */
    scale = 65536 / (DEF_ST_SCALE / (DEF_ST_ACCEL_FS_MG / 1000));

    gravity = 32768 / (DEF_ST_ACCEL_FS_MG / 1000) * axis_sign;
    gravity_scaled = gravity * scale;

    /* Accel normal mode */
    t += 3;
    for (i = 0; i < 3; i++) {
        st_bias[i + t] = accel_bias[i] * scale;
        unscaled[i + t] = accel_bias[i];
        if (axis == i) {
            st_bias[i + t] -= gravity_scaled;
            unscaled[i + t] -= gravity;
        }
    }
}

int ICM20948::selftest() {
    int gyro_bias_regular[THREE_AXES];
    int accel_bias_regular[THREE_AXES];
    static int raw_bias[THREE_AXES * 2];

    if (selftest_done == 1) {
        PRINTF("ICM20948: Self-test has already run. Skipping.\r\n");
        return 0;
    }
    /*
     * Perform self-test
     * For ICM20948 self-test is performed for both RAW_ACC/RAW_GYR
     */
    PRINTF("ICM20948: Running self-test...\r\n");

    /* Run the self-test */
    int rc = inv_icm20948_run_selftest(this, gyro_bias_regular, accel_bias_regular);
    if (!(rc & INV_ICM20948_GYR_SELF_TEST_OK)) PRINTF("ICM20948: Gyro self-test failed\r\n");
    if (!(rc & INV_ICM20948_ACC_SELF_TEST_OK)) PRINTF("ICM20948: Accel self-test failed\r\n");
    if (!(rc & INV_ICM20948_MAG_SELF_TEST_OK)) PRINTF("ICM20948: Mag self-test failed\r\n");
    if ((rc & INV_ICM20948_SELF_TEST_OK) != INV_ICM20948_SELF_TEST_OK) {
        PRINTF("ICM20948: Self-test failure !\r\n");
        return -1;
    }
    PRINTF("ICM20948: Self-test passed!\r\n");
    selftest_done = 1;
    offset_done = 0;

    /* It's advised to re-init the icm20948 device after self-test for normal use */
    setup();

    inv_icm20948_get_st_bias(this, gyro_bias_regular, accel_bias_regular, raw_bias, unscaled_bias);
    PRINTF("ICM20948: GYR bias (FS=250dps) (dps): x=%f, y=%f, z=%f\r\n", (float)(raw_bias[0] / (float)(1 << 16)),
           (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
    PRINTF("ICM20948: ACC bias (FS=2g) (g): x=%f, y=%f, z=%f\r\n", (float)(raw_bias[0 + 3] / (float)(1 << 16)),
           (float)(raw_bias[1 + 3] / (float)(1 << 16)), (float)(raw_bias[2 + 3] / (float)(1 << 16)));

    return 0;
}

int ICM20948::enableSensor(inv_icm20948_sensor sensor) {
    if (selftest_done && !offset_done) {  // If we've run selftes and not already set the offset.
        inv_icm20948_set_offset(this, unscaled_bias);
        offset_done = 1;
    }
    int rc = inv_icm20948_enable_sensor(this, sensor, 1);
    // rc |= inv_icm20948_set_sensor_period(this, sensor, period);
    return rc;
}

int ICM20948::disableSensor(inv_icm20948_sensor sensor) { return inv_icm20948_enable_sensor(this, sensor, 0); }

int ICM20948::setSensorPeriod(inv_icm20948_sensor sensor, uint32_t period) {
    return inv_icm20948_set_sensor_period(this, sensor, period);
}

int ICM20948::enableSetSensor(inv_icm20948_sensor sensor, uint32_t period) {
    return setSensorPeriod(sensor, period) | enableSensor(sensor);
}

int ICM20948::readSensor() {
    return inv_icm20948_poll_sensor(this, this, &ICM20948::build_sensor_event_data);  //
}
void ICM20948::build_sensor_event_data(void* context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void* data,
                                       const void* arg) {
    ((ICM20948*)context)->build_sensor_event_data(sensortype, timestamp, data, arg);
}
void ICM20948::build_sensor_event_data(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void* data,
                                       const void* arg) {
    uint8_t sensor_id = convert_to_generic_ids[sensortype];
    uint32_t stamp = timestamp / 1000;
    float* dat = (float*)data;
    using namespace rosRT::msgs;
    float64* buf;
    switch (sensor_id) {
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:  // 未校准陀螺仪
        buf = (float64*)&_msg_buf.vectbias;
        for (int i = 0; i < 6; ++i) buf[i] = dat[i];
        _pub_uncal_gyro.publish(&_msg_buf.vectbias);
        break;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:  // 未校准磁场
        buf = (float64*)&_msg_buf.vectbias;
        for (int i = 0; i < 6; ++i) buf[i] = dat[i];
        _pub_uncal_mag.publish(&_msg_buf.vectbias);
        break;
    case INV_SENSOR_TYPE_GYROSCOPE:  // 陀螺仪
        _msg_buf.vector3_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.vector3_stamped.vector;
        for (int i = 0; i < 3; ++i) buf[i] = dat[i];
        _pub_gyro.publish(&_msg_buf.vector3_stamped);
        break;
    case INV_SENSOR_TYPE_GRAVITY:  // 重力
        _msg_buf.vector3_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.vector3_stamped.vector;
        for (int i = 0; i < 3; ++i) buf[i] = dat[i];
        _pub_gravity.publish(&_msg_buf.vector3_stamped);
        break;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:  // 线加速度
        _msg_buf.vector3_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.vector3_stamped.vector;
        for (int i = 0; i < 3; ++i) buf[i] = dat[i];
        _pub_linear_accel.publish(&_msg_buf.vector3_stamped);
        break;
    case INV_SENSOR_TYPE_ACCELEROMETER:  // 加速度计
        _msg_buf.vector3_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.vector3_stamped.vector;
        for (int i = 0; i < 3; ++i) buf[i] = dat[i];
        _pub_accel.publish(&_msg_buf.vector3_stamped);
        break;
    case INV_SENSOR_TYPE_MAGNETOMETER:  // 磁场
        _msg_buf.vector3_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.vector3_stamped.vector;
        for (int i = 0; i < 3; ++i) buf[i] = dat[i];
        _pub_mag.publish(&_msg_buf.vector3_stamped);
        break;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:  // 地磁位姿
        _msg_buf.quaternion_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.quaternion_stamped.quaternion;
        for (int i = 0; i < 4; ++i) buf[i] = dat[i];
        _pub_mag_orientation.publish(&_msg_buf.quaternion_stamped);
        break;
    case INV_SENSOR_TYPE_ROTATION_VECTOR:  // 6DOF位姿
        _msg_buf.quaternion_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.quaternion_stamped.quaternion;
        for (int i = 0; i < 4; ++i) buf[i] = dat[i];
        _pub_9DOF_orientation.publish(&_msg_buf.quaternion_stamped);
        break;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:  // 9DOF位姿
        _msg_buf.quaternion_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.quaternion_stamped.quaternion;
        for (int i = 0; i < 4; ++i) buf[i] = dat[i];
        _pub_9DOF_orientation.publish(&_msg_buf.quaternion_stamped);
        break;
    case INV_SENSOR_TYPE_ORIENTATION:  // 9DOF RPY
        _msg_buf.vector3_stamped.header.stamp = stamp;
        buf = (float64*)&_msg_buf.vector3_stamped.vector;
        for (int i = 0; i < 3; ++i) buf[i] = dat[i];
        _pub_rpy_orientation.publish(&_msg_buf.vector3_stamped);
        break;
    }
}