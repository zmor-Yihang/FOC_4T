#ifndef __IF_OPEN_H__
#define __IF_OPEN_H__

#include "foc.h"
#include "ramp.h"
#include "print.h"

void if_open_init(float speed_rpm, float iq);
void print_if_current_info(void);

#endif /*__IF_OPEN_H__*/
