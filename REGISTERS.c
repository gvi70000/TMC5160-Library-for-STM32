#include "spi.h"
#include "gpio.h"
#include "TMC5160.h"
#include "DEFINES.h"
#include "STRUCTURES.h"
#include "STRUCTURES.h"
#include "REGISTERS.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// https://community.st.com/t5/stm32-mcus/dma-is-not-working-on-stm32h7-devices/ta-p/49498

Stepper_t myMotors[MOTOR_COUNT] = {
    {.ControlPins = {GPIOC, GPIO_PIN_14, GPIO_PIN_15}, .Reg = {0}},
    {.ControlPins = {GPIOC, GPIO_PIN_1, GPIO_PIN_2},  .Reg = {0}},
    {.ControlPins = {GPIOA, GPIO_PIN_5, GPIO_PIN_6},  .Reg = {0}},
    {.ControlPins = {GPIOC, GPIO_PIN_5, GPIO_PIN_4},  .Reg = {0}},
};

void TMC5160_EnableMotor(Stepper_t* motor, uint8_t state) {
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.enablePin, state);
}

void TMC5160_WriteRegister(Stepper_t* motor, uint8_t address, uint32_t value) {
    uint8_t txBuffer[5]; // 1 byte for address, 4 bytes for data

    // Set the MSB of the address to indicate a write operation (address + 0x80)
    txBuffer[0] = address | 0x80;

    // Transmit the 32-bit data (MSB first)
    txBuffer[1] = (value >> 24) & 0xFF;
    txBuffer[2] = (value >> 16) & 0xFF;
    txBuffer[3] = (value >> 8) & 0xFF;
    txBuffer[4] = value & 0xFF;

    // Select the motor (NCS low)
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.selectPin, GPIO_PIN_RESET);

    // Transmit 5 bytes (address + data)
    HAL_SPI_Transmit(&hspi4, txBuffer, 5, HAL_MAX_DELAY);

    // Deselect the motor (NCS high)
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.selectPin, GPIO_PIN_SET);
}

uint32_t TMC5160_ReadRegister(Stepper_t* motor, uint8_t address) {
    uint8_t txBuffer[5] = {0};  // 1 byte for address, 4 dummy bytes
    uint8_t rxBuffer[5] = {0};  // Buffer to store received data

    // Prepare the address byte for the read (clear the MSB for read operation)
    txBuffer[0] = address & 0x7F;

    // First transaction: Send the address, receive status bits and unused data
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.selectPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi4, txBuffer, rxBuffer, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.selectPin, GPIO_PIN_SET);

    // Store the status bits 
    motor->Reg.DRV_STATUS.Val.Value = (rxBuffer[1] << 24) | (rxBuffer[2] << 16) | (rxBuffer[3] << 8) | rxBuffer[4];

    // Second transaction: Send the address again, receive the actual data
    memset(rxBuffer, 0, sizeof(rxBuffer));  // Clear the receive buffer
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.selectPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi4, txBuffer, rxBuffer, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(motor->ControlPins.ctrlPort, motor->ControlPins.selectPin, GPIO_PIN_SET);

    // Combine the received bytes into a 32-bit value (data starts from rxBuffer[1])
    uint32_t receivedValue = (rxBuffer[1] << 24) | (rxBuffer[2] << 16) | (rxBuffer[3] << 8) | rxBuffer[4];

    return receivedValue;
}

// R/W: General configuration register
void TMC5160_GetGCONF(Stepper_t* motor) {
    motor->Reg.GCONF.Val.Value = TMC5160_ReadRegister(motor, TMC5160_GCONF);
}

void TMC5160_SetGCONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_GCONF, motor->Reg.GCONF.Val.Value);
}

// R+WC: Global status register
void TMC5160_GetGSTAT(Stepper_t* motor) {
    motor->Reg.GSTAT.Val.Value = TMC5160_ReadRegister(motor, TMC5160_GSTAT);
}

void TMC5160_ClearGSTAT(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_GSTAT, 0xFFFFFFFF);  // Clear all bits by writing 1
}

#ifndef USE_SPI
// R: Interface transmission counter
void TMC5160_GetIFCNT(Stepper_t* motor) {
    motor->Reg.IFCNT.Val.Value = TMC5160_ReadRegister(motor, TMC5160_IFCNT);
}

// W: Node configuration register
void TMC5160_SetNODECONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_NODECONF, motor->Reg.NODECONF.Val.Value);
}
#endif

// R: Input pins status
void TMC5160_GetIOIN(Stepper_t* motor) {
    motor->Reg.IOIN.Val.Value = TMC5160_ReadRegister(motor, TMC5160_IOIN);
}

// W: Position comparison register
void TMC5160_SetX_COMPARE(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_X_COMPARE, motor->Reg.X_COMPARE.Val.Value);
}

// W: OTP programming register
void TMC5160_SetOTP_PROG(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_OTP_PROG, motor->Reg.OTP_PROG.Val.Value);
}

// R: OTP read register
void TMC5160_GetOTP_READ(Stepper_t* motor) {
    motor->Reg.OTP_READ.Val.Value = TMC5160_ReadRegister(motor, TMC5160_OTP_READ);
}

// R/W: Factory configuration register
void TMC5160_GetFACTORY_CONF(Stepper_t* motor) {
    motor->Reg.FACTORY_CONF.Val.Value = TMC5160_ReadRegister(motor, TMC5160_FACTORY_CONF);
}

void TMC5160_SetFACTORY_CONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_FACTORY_CONF, motor->Reg.FACTORY_CONF.Val.Value);
}

// W: Short configuration register
void TMC5160_SetSHORT_CONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_SHORT_CONF, motor->Reg.SHORT_CONF.Val.Value);
}

// W: Driver configuration register
void TMC5160_SetDRV_CONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_DRV_CONF, motor->Reg.DRV_CONF.Val.Value);
}

// W: Global scaler register
void TMC5160_SetGLOBAL_SCALER(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_GLOBAL_SCALER, motor->Reg.GLOBAL_SCALER.Val.Value);
}

// R: Offset read register
void TMC5160_GetOFFSET_READ(Stepper_t* motor) {
    motor->Reg.OFFSET_READ.Val.Value = TMC5160_ReadRegister(motor, TMC5160_OFFSET_READ);
}

// R/W: IHOLD and IRUN register
void TMC5160_GetIHOLD_IRUN(Stepper_t* motor) {
    motor->Reg.IHOLD_IRUN.Val.Value = TMC5160_ReadRegister(motor, TMC5160_IHOLD_IRUN);
}

void TMC5160_SetIHOLD_IRUN(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_IHOLD_IRUN, motor->Reg.IHOLD_IRUN.Val.Value);
}

// R/W: TPOWERDOWN register
void TMC5160_GetTPOWERDOWN(Stepper_t* motor) {
    motor->Reg.TPOWERDOWN.Val.Value = TMC5160_ReadRegister(motor, TMC5160_TPOWERDOWN);
}

void TMC5160_SetTPOWERDOWN(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_TPOWERDOWN, motor->Reg.TPOWERDOWN.Val.Value);
}

// R: TSTEP register
void TMC5160_GetTSTEP(Stepper_t* motor) {
    motor->Reg.TSTEP.Val.Value = TMC5160_ReadRegister(motor, TMC5160_TSTEP);
}

// W: TPWMTHRS register
void TMC5160_SetTPWMTHRS(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_TPWMTHRS, motor->Reg.TPWMTHRS.Val.Value);
}

// W: TCOOLTHRS register
void TMC5160_SetTCOOLTHRS(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_TCOOLTHRS, motor->Reg.TCOOLTHRS.Val.Value);
}

// W: THIGH register
void TMC5160_SetTHIGH(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_THIGH, motor->Reg.THIGH.Val.Value);
}

// R/W: RAMPMODE register
void TMC5160_GetRAMPMODE(Stepper_t* motor) {
    motor->Reg.RAMPMODE.Val.Value = TMC5160_ReadRegister(motor, TMC5160_RAMPMODE);
}

void TMC5160_SetRAMPMODE(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_RAMPMODE, motor->Reg.RAMPMODE.Val.Value);
}

// R/W: XACTUAL register
void TMC5160_GetXACTUAL(Stepper_t* motor) {
    motor->Reg.XACTUAL.Val.Value = TMC5160_ReadRegister(motor, TMC5160_XACTUAL);
}

void TMC5160_SetXACTUAL(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_XACTUAL, motor->Reg.XACTUAL.Val.Value);
}

// R: VACTUAL register
void TMC5160_GetVACTUAL(Stepper_t* motor) {
    motor->Reg.VACTUAL.Val.Value = TMC5160_ReadRegister(motor, TMC5160_VACTUAL);
}

// W: VMAX register
void TMC5160_SetVMAX(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_VMAX, motor->Reg.VMAX.Val.Value);
}

// W: VSTART register
void TMC5160_SetVSTART(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_VSTART, motor->Reg.VSTART.Val.Value);
}

// W: A1 register
void TMC5160_SetA1(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_A1, motor->Reg.A1.Val.Value);
}

// W: V1 register
void TMC5160_SetV1(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_V1, motor->Reg.V1.Val.Value);
}

// W: AMAX register
void TMC5160_SetAMAX(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_AMAX, motor->Reg.AMAX.Val.Value);
}

// W: DMAX register
void TMC5160_SetDMAX(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_DMAX, motor->Reg.DMAX.Val.Value);
}

// W: D1 register
void TMC5160_SetD1(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_D1, motor->Reg.D1.Val.Value);
}

// W: VSTOP register
void TMC5160_SetVSTOP(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_VSTOP, motor->Reg.VSTOP.Val.Value);
}

// W: TZEROWAIT register
void TMC5160_SetTZEROWAIT(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_TZEROWAIT, motor->Reg.TZEROWAIT.Val.Value);
}

// R/W: XTARGET register
void TMC5160_GetXTARGET(Stepper_t* motor) {
    motor->Reg.XTARGET.Val.Value = TMC5160_ReadRegister(motor, TMC5160_XTARGET);
}

void TMC5160_SetXTARGET(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_XTARGET, motor->Reg.XTARGET.Val.Value);
}

// W: VDCMIN register
void TMC5160_SetVDCMIN(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_VDCMIN, motor->Reg.VDCMIN.Val.Value);
}

// R/W: SW_MODE register
void TMC5160_GetSW_MODE(Stepper_t* motor) {
    motor->Reg.SW_MODE.Val.Value = TMC5160_ReadRegister(motor, TMC5160_SW_MODE);
}

void TMC5160_SetSW_MODE(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_SW_MODE, motor->Reg.SW_MODE.Val.Value);
}

// R+WC: RAMP_STAT register
void TMC5160_GetRAMP_STAT(Stepper_t* motor) {
    motor->Reg.RAMP_STAT.Val.Value = TMC5160_ReadRegister(motor, TMC5160_RAMP_STAT);
}

void TMC5160_ClearRAMP_STAT(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_RAMP_STAT, 0xFFFFFFFF);  // Clear all bits by writing 1
}

// R: XLATCH register
void TMC5160_GetXLATCH(Stepper_t* motor) {
    motor->Reg.XLATCH.Val.Value = TMC5160_ReadRegister(motor, TMC5160_XLATCH);
}

// R/W: ENCMODE register
void TMC5160_GetENCMODE(Stepper_t* motor) {
    motor->Reg.ENCMODE.Val.Value = TMC5160_ReadRegister(motor, TMC5160_ENCMODE);
}

void TMC5160_SetENCMODE(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_ENCMODE, motor->Reg.ENCMODE.Val.Value);
}

// R/W: X_ENC register
void TMC5160_GetX_ENC(Stepper_t* motor) {
    motor->Reg.X_ENC.Val.Value = TMC5160_ReadRegister(motor, TMC5160_X_ENC);
}

void TMC5160_SetX_ENC(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_X_ENC, motor->Reg.X_ENC.Val.Value);
}

// W: ENC_CONST register
void TMC5160_SetENC_CONST(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_ENC_CONST, motor->Reg.ENC_CONST.Val.Value);
}

// R: ENC_STATUS register
void TMC5160_GetENC_STATUS(Stepper_t* motor) {
    motor->Reg.ENC_STATUS.Val.Value = TMC5160_ReadRegister(motor, TMC5160_ENC_STATUS);
}

// R: ENC_LATCH register
void TMC5160_GetENC_LATCH(Stepper_t* motor) {
    motor->Reg.ENC_LATCH.Val.Value = TMC5160_ReadRegister(motor, TMC5160_ENC_LATCH);
}

// W: ENC_DEVIATION register
void TMC5160_SetENC_DEVIATION(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_ENC_DEVIATION, motor->Reg.ENC_DEVIATION.Val.Value);
}

// W: MSLUT0 register
void TMC5160_SetMSLUT0(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT0, motor->Reg.MSLUT0.Val.Value);
}

// W: MSLUT1 register
void TMC5160_SetMSLUT1(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT1, motor->Reg.MSLUT1.Val.Value);
}

// W: MSLUT2 register
void TMC5160_SetMSLUT2(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT2, motor->Reg.MSLUT2.Val.Value);
}

// W: MSLUT3 register
void TMC5160_SetMSLUT3(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT3, motor->Reg.MSLUT3.Val.Value);
}

// W: MSLUT4 register
void TMC5160_SetMSLUT4(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT4, motor->Reg.MSLUT4.Val.Value);
}

// W: MSLUT5 register
void TMC5160_SetMSLUT5(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT5, motor->Reg.MSLUT5.Val.Value);
}

// W: MSLUT6 register
void TMC5160_SetMSLUT6(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT6, motor->Reg.MSLUT6.Val.Value);
}

// W: MSLUT7 register
void TMC5160_SetMSLUT7(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUT7, motor->Reg.MSLUT7.Val.Value);
}

// W: MSLUTSEL register
void TMC5160_SetMSLUTSEL(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUTSEL, motor->Reg.MSLUTSEL.Val.Value);
}

// W: MSLUTSTART register
void TMC5160_SetMSLUTSTART(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_MSLUTSTART, motor->Reg.MSLUTSTART.Val.Value);
}

// R: MSCNT register
void TMC5160_GetMSCNT(Stepper_t* motor) {
    motor->Reg.MSCNT.Val.Value = TMC5160_ReadRegister(motor, TMC5160_MSCNT);
}

// R: MSCURACT register
void TMC5160_GetMSCURACT(Stepper_t* motor) {
    motor->Reg.MSCURACT.Val.Value = TMC5160_ReadRegister(motor, TMC5160_MSCURACT);
}

// R/W: CHOPCONF register
void TMC5160_GetCHOPCONF(Stepper_t* motor) {
    motor->Reg.CHOPCONF.Val.Value = TMC5160_ReadRegister(motor, TMC5160_CHOPCONF);
}

void TMC5160_SetCHOPCONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_CHOPCONF, motor->Reg.CHOPCONF.Val.Value);
}

// R/W: COOLCONF register
void TMC5160_GetCOOLCONF(Stepper_t* motor) {
    motor->Reg.COOLCONF.Val.Value = TMC5160_ReadRegister(motor, TMC5160_COOLCONF);
}

void TMC5160_SetCOOLCONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_COOLCONF, motor->Reg.COOLCONF.Val.Value);
}

// W: DCCTRL register
void TMC5160_SetDCCTRL(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_DCCTRL, motor->Reg.DCCTRL.Val.Value);
}

// R: DRV_STATUS register
void TMC5160_GetDRV_STATUS(Stepper_t* motor) {
    motor->Reg.DRV_STATUS.Val.Value = TMC5160_ReadRegister(motor, TMC5160_DRV_STATUS);
}

// R/W: PWMCONF register
void TMC5160_GetPWMCONF(Stepper_t* motor) {
    motor->Reg.PWMCONF.Val.Value = TMC5160_ReadRegister(motor, TMC5160_PWMCONF);
}

void TMC5160_SetPWMCONF(Stepper_t* motor) {
    TMC5160_WriteRegister(motor, TMC5160_PWMCONF, motor->Reg.PWMCONF.Val.Value);
}

// R: PWM_SCALE register
void TMC5160_GetPWM_SCALE(Stepper_t* motor) {
    motor->Reg.PWM_SCALE.Val.Value = TMC5160_ReadRegister(motor, TMC5160_PWM_SCALE);
}

// R: PWM_AUTO register
void TMC5160_GetPWM_AUTO(Stepper_t* motor) {
    motor->Reg.PWM_AUTO.Val.Value = TMC5160_ReadRegister(motor, TMC5160_PWM_AUTO);
}

#ifndef USE_SPI
// R: LOST_STEPS register
void TMC5160_GetLOST_STEPS(Stepper_t* motor) {
    motor->Reg.LOST_STEPS.Val.Value = TMC5160_ReadRegister(motor, TMC5160_LOST_STEPS);
}
#endif