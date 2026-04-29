#ifndef __POSITION_CLOSED_H__
#define __POSITION_CLOSED_H__

#include "../foc/foc.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"
#include "../sensor/current_sense.h"
#include "../app/user_config.h"

void positionClosed_init(float position_rad);
void positionClosed_setTarget(float position_rad);
void positionClosed_setTargetRev(float position_rev);
void positionClosed_resetPosition(float position_rad);
void positionClosedDebug_print_info(void);

#endif /* __POSITION_CLOSED_H__ */
