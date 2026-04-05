#ifndef __SPEED_CLOSED_H__
#define __SPEED_CLOSED_H__

#include "foc.h"
#include "encoder.h"
#include "print.h"
#include "motor_config.h"

void speed_closed_init(float speed_rpm);
void print_speed_info(void);

#endif /* __SPEED_CLOSED_H__ */
