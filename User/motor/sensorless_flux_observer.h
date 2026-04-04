#ifndef __SENSORLESS_FLUX_OBSERVER_H__
#define __SENSORLESS_FLUX_OBSERVER_H__

#include <stdio.h>
#include "flux_observer.h"
#include "foc.h"
#include "ramp.h"
#include "print.h"

void sensorless_flux_observer_init(float target_speed_rpm);
void print_sensorless_flux_observer_info(void);

#endif /* __SENSORLESS_FLUX_OBSERVER_H__ */
