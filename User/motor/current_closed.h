#ifndef __CURRENT_CLOSED_H__
#define __CURRENT_CLOSED_H__

#include "gpio.h"
#include "foc.h"
#include "encoder.h"
#include "current_sense.h"
#include "print.h"

void currentClosed_init(float id, float iq);
void currentClosedDebug_print_info(void);


#endif /*__CURRENT_CLOSED_H__*/
