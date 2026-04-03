#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32g4xx_hal.h"
#include <stdio.h>

#include "bsp/usart.h"
#include "bsp/gpio.h"
#include "bsp/clock.h"

#include "motor/sensorless_luenberger.h"
#include "motor/if_open.h"
#include "motor/current_closed.h"
#include "motor/speed_closed.h"
#include "motor/sensorless_smo.h"
#include "motor/flux_weak_speed_closed.h"
#include "motor/speed_closed_with_smo.h"
#include "motor/speed_closed_with_luenberger.h"
#include "motor/speed_closed_with_flux_observer.h"
#include "motor/sensorless_flux_observer.h"
#include "motor/sensorless_flux_observer.h"


#endif /* __MAIN_H__ */
