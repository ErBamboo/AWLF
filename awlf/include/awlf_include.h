#ifndef __AWLF_INCLUDE_H__
#define __AWLF_INCLUDE_H__

/**** Frame files ****/
#include "f_port.h"     /* 端口文件，用于移植 */
#include "f_def.h"      /* 定义文件，定义了框架中常用的宏、枚举等 */
#include "f_api.h"

/***** Frame component *****/
#include "../component/include/f_device.h"   /* 设备基类 */
#include "../component/include/f_mem.h"     /* 内存管理 */
#include "../middlewire/f_middlewire.h"      /* 中间件 */

#endif

