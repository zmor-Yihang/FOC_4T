#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "../foc/foc.h"
#include <math.h>
#include "../adv_alg/flux_observer.h"
#include "../bsp/adc.h"
#include "../bsp/i2c.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/utils.h"
#include "../utils/print.h"

typedef enum
{
    MOTOR_CTRL_MODE_CURRENT = 0,
    MOTOR_CTRL_MODE_SPEED,
    MOTOR_CTRL_MODE_POSITION,
    MOTOR_CTRL_MODE_SPEED_WEAK,
    MOTOR_CTRL_MODE_FLUX_OBSERVER,
} motor_ctrl_mode_t;

typedef struct
{
    float id;
    float iq;
    float speed_rpm;
    float position_rad;
} motor_ctrl_cmd_t;

void motorControl_init(motor_ctrl_mode_t mode, const motor_ctrl_cmd_t *cmd);
void motorControl_debugPrint(void);
void motorControl_setPosition(float position_rad);
void motorControl_setPositionRev(float position_rev);
void motorControl_resetPosition(float position_rad);
foc_t *motorControl_getFocHandle(void);
motor_ctrl_mode_t motorControl_getMode(void);

#endif /* __MOTOR_CONTROL_H__ */
