#ifndef _MCU_ID_hpp
#define _MCU_ID_hpp

#define MCU_ID 0

#define MCU_MASTER if constexpr (MCU_ID == 0)
#define MCU_SLAVE if constexpr (MCU_ID == 1)
#define MCU_BOTH if constexpr (MCU_ID == 0 || MCU_ID == 1)

#endif  // _MCU_ID_hpp