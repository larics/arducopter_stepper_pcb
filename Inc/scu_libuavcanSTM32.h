#ifndef SCU_LIBUAVCANSTM32_H
#define SCU_LIBUAVCANSTM32_H

#include <uavcan_stm32/uavcan_stm32.hpp>

uavcan::ICanDriver& getCanDriver();

uavcan::ISystemClock& getSystemClock();

#endif
