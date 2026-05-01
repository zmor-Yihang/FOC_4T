#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "../foc/foc.h"
#include <math.h>
#include "../bsp/adc.h"
#include "../bsp/i2c.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/utils.h"
#include "../utils/print.h"

void motorControl_init(foc_mode_t mode, const foc_cmd_t *cmd);
void motorControl_debugPrint(void);
void motorControl_setPosition(float position_rad);
void motorControl_setPositionRev(float position_rev);
void motorControl_resetPosition(float position_rad);
foc_t *motorControl_getFocHandle(void);
foc_mode_t motorControl_getMode(void);

#endif /* __MOTOR_CONTROL_H__ */
