#ifndef _ICM20948_system_h
#define _ICM20948_system_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int idd_io_hal_read_reg(void* context, uint8_t reg, uint8_t* rbuffer, uint32_t rlen);
int idd_io_hal_write_reg(void* context, uint8_t reg, const uint8_t* wbuffer, uint32_t wlen);

#ifdef __cplusplus
}
#endif

#endif  // _ICM20948_system_h