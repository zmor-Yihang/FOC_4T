#ifndef __SENSORLESS_SMO__H__
#define __SENSORLESS_SMO__H__

#include <stdio.h>
#include "smo.h"
#include "foc.h"
#include "print.h"
#include "ramp.h"

// 状态定义
typedef enum
{
    STATE_IF_STARTUP, // IF启动阶段
    STATE_SMO_RUNNING // SMO闭环运行阶段
} sensorless_state_t;

void sensorless_smo_init(float speed_rpm);
void print_sensorless_smo_info(void);

#endif /* __SENSORLESS_SMO__H__ */
