#ifndef TMC5160_H_
#define TMC5160_H_

#include <stdint.h>
#include "gpio.h"
#include "spi.h"
#include "STRUCTURES.h"
#include "DEFINES.h"



void TMC5160_Init(Stepper_t* motor);
void TMC5160_SetRMSCurrent(Stepper_t *motor, uint16_t current_mA);
void TMC5160_GetMaxSpeed(Stepper_t* motor);

void MonitorCoolStep(Stepper_t* motor, uint32_t* stallguard, uint32_t* current_scale);
void TMC5160_TuneCoolStep(Stepper_t* motor);
void TMC5160_AutoTuneStealthChop(Stepper_t* motor);
void TMC5160_MoveMotorAtRPM(Stepper_t* motor, uint32_t rpm);
void TMC5160_SlowMoveMotor(Stepper_t* motor, uint32_t steps);
void TMC5160_StopMotor(Stepper_t* motor);
void TMC5160_WaitForPosition(Stepper_t* motor, int32_t targetPosition);
uint32_t TMC5160_ConvertRPMToVelocity(uint32_t rpm);
#endif /* TMC5160_H_ */
