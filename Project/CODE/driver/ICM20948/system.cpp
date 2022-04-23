#include "devices.hpp"

extern "C" {
void inv_icm20948_sleep_us(int us) { systick.delay_us(us); }
uint64_t inv_icm20948_get_time_us(void) { return systick.get_us(); }
uint64_t inv_ak0991x_get_time_us(void) { return systick.get_us(); }
}