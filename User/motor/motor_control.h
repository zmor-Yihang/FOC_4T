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

void motorControl_init(foc_mode_t mode);
void motorControl_debugPrint(void);
void motorControl_setMode(foc_mode_t mode);
void motorControl_setSpeed(float rpm);
void motorControl_setCurrent(float target_id, float target_iq);
void motorControl_setPositionRev(float position_rev);
void motorControl_resetPosition(float position_rad);

#endif /* __MOTOR_CONTROL_H__ */
