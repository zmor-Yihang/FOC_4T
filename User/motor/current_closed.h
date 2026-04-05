#ifndef __CURRENT_CLOSED_H__
#define __CURRENT_CLOSED_H__

#include "gpio.h"
#include "foc.h"
#include "encoder.h"
#include "print.h"

void current_closed_init(float id, float iq);
void print_current_info(void);

#endif /*__CURRENT_CLOSED_H__*/
