#ifndef __SPEED_CLOSED_H__
#define __SPEED_CLOSED_H__

#include "foc.h"
#include "encoder.h"
#include "print.h"
#include "current_sense.h"
#include "motor_config.h"

void speedClosed_init(float speed_rpm);
void speedClosedDebug_print_info(void);


#endif /* __SPEED_CLOSED_H__ */
