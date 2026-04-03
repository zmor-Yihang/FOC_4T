#ifndef __SENSORLESS_FLUX_OBSERVER_H__
#define __SENSORLESS_FLUX_OBSERVER_H__

#include <stdio.h>
#include "foc/flux_observer.h"
#include "foc/foc.h"
#include "utils/ramp.h"
#include "utils/print.h"

void sensorless_flux_observer_init(float target_speed_rpm);
void print_sensorless_flux_observer_info(void);

#endif /* __SENSORLESS_FLUX_OBSERVER_H__ */
