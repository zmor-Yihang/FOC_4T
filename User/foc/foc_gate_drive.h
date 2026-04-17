#ifndef __FOC_GATE_DRIVE_H__
#define __FOC_GATE_DRIVE_H__

#include "../alg/clark_park.h"
#include "../alg/svpwm.h"
#include "../bsp/tim.h"

abc_t focGateDrive_set_voltage(alphabeta_t voltage_alphabeta);
abc_t focGateDrive_stop(void);

#endif /* __FOC_GATE_DRIVE_H__ */
