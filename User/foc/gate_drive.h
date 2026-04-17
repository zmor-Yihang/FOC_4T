#ifndef __GATE_DRIVE_H__
#define __GATE_DRIVE_H__

#include "../alg/clark_park.h"
#include "../alg/svpwm.h"
#include "../bsp/tim.h"

abc_t gateDrive_set_voltage(alphabeta_t voltage_alphabeta);
abc_t gateDrive_stop(void);

#endif /* __GATE_DRIVE_H__ */
