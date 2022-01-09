#include "ICM20948.hpp"

#include <rtthread.h>

#include "ICM20948/dmp_img.hpp"
#include "ICM20948/macros.h"
#include "ICM20948/system.h"

extern "C" {
#include "zf_systick.h"
}
// clang-format off
#define DEBUG_LOG(s) rt_kputs(s)
#define CHECK_RC(s) if (rc) { DEBUG_LOG(s); return rc; }  // clang-format on

/* FSR configurations */
static int32_t cfg_acc_fsr = 4;    // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
static int32_t cfg_gyr_fsr = 250;  // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static float cfg_mounting_matrix[9]{
    1.f, 0,   0,    //
    0,   1.f, 0,    //
    0,   0,   1.f,  //
};

//  Magnetometer bias
static int biasq16[3] = {0};

int ICM20948::spi_read(void* context, uint8_t reg, uint8_t* buf, uint32_t len) {
    ICM20948& self = *(ICM20948*)context;
    reg |= READ_BIT_MASK;
    self._buf[0] = reg;
    spi_mosi(self.SPI_N, self.CS, self._buf, self._buf, len + 1, 1);
    memcpy(buf, self._buf + 1, len);
    return 0;
}

int ICM20948::spi_write(void* context, uint8_t reg, const uint8_t* buf, uint32_t len) {
    ICM20948& self = *(ICM20948*)context;
    reg &= WRITE_BIT_MASK;
    self._buf[0] = reg;
    memcpy(self._buf + 1, buf, len);
    spi_mosi(self.SPI_N, self.CS, self._buf, NULL, len + 1, 1);
    return 0;
}

ICM20948::ICM20948(SPIN_enum spi_n, SPI_PIN_enum sck, SPI_PIN_enum mosi, SPI_PIN_enum miso, SPI_PIN_enum cs)
    : SPI_N(spi_n), SCK(sck), MOSI(mosi), MISO(miso), CS(cs) {
    context = this;
    read_reg = (decltype(read_reg))&ICM20948::spi_read;
    write_reg = (decltype(write_reg))&ICM20948::spi_write;
    max_read = max_write = 1024 * 16;
    is_spi = true;
    inv_icm20948_reset_states(this, this);
    inv_icm20948_register_aux_compass(this, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
}

int ICM20948::init() {
    systick_delay_ms(10);
    spi_init(SPI_N, SCK, MOSI, MISO, CS, 3, 10 * 1000 * 1000);
    int rc;

    // Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
    uint8_t whoami = 0xff;
    do {
        rc = inv_icm20948_get_whoami(this, &whoami);
        if (rc) rt_kprintf("ICM20948: WHOAMI error");
    } while (whoami != WHO_AM_I);

    // Setup accel and gyro mounting matrix and associated angle for current board
    inv_icm20948_init_matrix(this);

    // set default power mode
    rc = inv_icm20948_initialize(this, ICM20948_dmp_img, sizeof(ICM20948_dmp_img));
    CHECK_RC("ICM20948: DMP init error");

    inv_icm20948_register_aux_compass(this, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

    // Initialize auxiliary sensors
    rc = inv_icm20948_initialize_auxiliary(this);
    CHECK_RC("ICM20948: Compass not detected");

    // Set full-scale range for sensors
    for (int ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
        inv_icm20948_set_matrix(this, cfg_mounting_matrix, (inv_icm20948_sensor)ii);

    //  Smooth accel output
    base_state.accel_averaging = 5;
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void*)&cfg_acc_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_ACCELEROMETER, (const void*)&cfg_acc_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void*)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_GYROSCOPE, (const void*)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(this, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void*)&cfg_gyr_fsr);

    inv_icm20948_set_bias(this, INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, biasq16);

    // re-initialize base state structure
    inv_icm20948_init_structure(this);

    // we should be good to go !

    // Now that Icm20948 device was initialized, we can proceed with DMP image loading
    // This step is mandatory as DMP image are not store in non volatile memory
    rc = inv_icm20948_load(this, ICM20948_dmp_img, sizeof(ICM20948_dmp_img));
    CHECK_RC("Error loading DMP3\n");

    DEBUG_LOG("ICM20948 Init complete.\n");

    return 0;
}

int ICM20948::enableSensor(inv_icm20948_sensor sensor, uint32_t period) {
    return inv_icm20948_enable_sensor(this, sensor, 1) | inv_icm20948_set_sensor_period(this, sensor, period);
}

int ICM20948::disableSensor(inv_icm20948_sensor sensor) { return inv_icm20948_enable_sensor(this, sensor, 0); }

int ICM20948::enableAllSensors(uint32_t period) {
    int rc;

    rc = enableSensor(INV_ICM20948_SENSOR_GYROSCOPE, period);
    CHECK_RC("Gyroscope enable failed\n");

    rc = enableSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, period);
    CHECK_RC("Game Rotation Vector enable failed\n");

    rc = enableSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION, period);
    CHECK_RC("Linear Acceleration enable failed\n");

    rc = enableSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR, period);
    CHECK_RC("Rotation Vector enable failed\n");

    rc = enableSensor(INV_ICM20948_SENSOR_GRAVITY, period);
    CHECK_RC("Gravity enable failed\n");

    DEBUG_LOG("All sensors enabled\n");
    return 0;
}