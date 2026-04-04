#ifndef __SENSORLESS_LUENBERGER_H__
#define __SENSORLESS_LUENBERGER_H__

#include <stdio.h>
#include "luenberger.h"
#include "foc.h"
#include "ramp.h"
#include "print.h"

// 状态定义
typedef enum
{
    LUENBERGER_STATE_IF_STARTUP, // IF启动阶段
    LUENBERGER_STATE_RUNNING     // Luenberger闭环运行阶段
} luenberger_state_t;

void sensorless_luenberger_init(float speed_rpm);
void print_sensorless_luenberger_info(void);

#endif /* __SENSORLESS_LUENBERGER_H__ */
