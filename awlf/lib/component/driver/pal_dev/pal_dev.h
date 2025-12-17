#ifndef __PAL_DEV_H__
#define __PAL_DEV_H__

#include "core/awlf_config.h"
#include "component/driver/core/device.h"

#ifdef __AWLF_USE_HAL_SERIALS
#include "pal_serial_dev.h"
#endif

#ifdef __AWLF_USE_HAL_CAN
#include "pal_can_dev.h"
#endif

#endif // __PAL_DEV_H__
