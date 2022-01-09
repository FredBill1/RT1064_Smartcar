#include "zf_spi.h"
#include "zf_systick.h"

void inv_icm20948_sleep_us(int us) { systick_delay_us(us); }
uint64_t inv_icm20948_get_time_us(void) { return systick_getval_us(); }
uint64_t inv_ak0991x_get_time_us(void) { return systick_getval_us(); }
