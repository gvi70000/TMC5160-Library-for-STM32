#ifndef REGISTERS_H_
#define REGISTERS_H_

	// Function declarations
	extern Stepper_t myMotors[MOTOR_COUNT];
	void TMC5160_EnableMotor(Stepper_t* motor, uint8_t state);
	void TMC5160_WriteRegister(Stepper_t* motor, uint8_t regAddress, uint32_t regValue);
	uint32_t TMC5160_ReadRegister(Stepper_t* motor, uint8_t regAddress);

	// R/W: General configuration register
	void TMC5160_GetGCONF(Stepper_t* motor);
	void TMC5160_SetGCONF(Stepper_t* motor);

	// R+WC: Global status register
	void TMC5160_GetGSTAT(Stepper_t* motor);
	void TMC5160_ClearGSTAT(Stepper_t* motor);

	#ifndef USE_SPI
	// R: Interface transmission counter
	void TMC5160_GetIFCNT(Stepper_t* motor);

	// W: Node configuration register
	void TMC5160_SetNODECONF(Stepper_t* motor);
	#endif

	// R: Input pins status
	void TMC5160_GetIOIN(Stepper_t* motor);

	// W: Position comparison register
	void TMC5160_SetX_COMPARE(Stepper_t* motor);

	// W: OTP programming register
	void TMC5160_SetOTP_PROG(Stepper_t* motor);

	// R: OTP read register
	void TMC5160_GetOTP_READ(Stepper_t* motor);

	// R/W: Factory configuration register
	void TMC5160_GetFACTORY_CONF(Stepper_t* motor);
	void TMC5160_SetFACTORY_CONF(Stepper_t* motor);

	// W: Short configuration register
	void TMC5160_SetSHORT_CONF(Stepper_t* motor);

	// W: Driver configuration register
	void TMC5160_SetDRV_CONF(Stepper_t* motor);

	// W: Global scaler register
	void TMC5160_SetGLOBAL_SCALER(Stepper_t* motor);

	// R: Offset read register
	void TMC5160_GetOFFSET_READ(Stepper_t* motor);

	// R/W: IHOLD and IRUN register
	void TMC5160_GetIHOLD_IRUN(Stepper_t* motor);
	void TMC5160_SetIHOLD_IRUN(Stepper_t* motor);

	// R/W: TPOWERDOWN register
	void TMC5160_GetTPOWERDOWN(Stepper_t* motor);
	void TMC5160_SetTPOWERDOWN(Stepper_t* motor);

	// R: TSTEP register
	void TMC5160_GetTSTEP(Stepper_t* motor);

	// W: TPWMTHRS register
	void TMC5160_SetTPWMTHRS(Stepper_t* motor);

	// W: TCOOLTHRS register
	void TMC5160_SetTCOOLTHRS(Stepper_t* motor);

	// W: THIGH register
	void TMC5160_SetTHIGH(Stepper_t* motor);

	// R/W: RAMPMODE register
	void TMC5160_GetRAMPMODE(Stepper_t* motor);
	void TMC5160_SetRAMPMODE(Stepper_t* motor);

	// R/W: XACTUAL register
	void TMC5160_GetXACTUAL(Stepper_t* motor);
	void TMC5160_SetXACTUAL(Stepper_t* motor);

	// R: VACTUAL register
	void TMC5160_GetVACTUAL(Stepper_t* motor);

	// W: VMAX register
	void TMC5160_SetVMAX(Stepper_t* motor);

	// W: VSTART register
	void TMC5160_SetVSTART(Stepper_t* motor);

	// W: A1 register
	void TMC5160_SetA1(Stepper_t* motor);

	// W: V1 register
	void TMC5160_SetV1(Stepper_t* motor);

	// W: AMAX register
	void TMC5160_SetAMAX(Stepper_t* motor);

	// W: DMAX register
	void TMC5160_SetDMAX(Stepper_t* motor);

	// W: D1 register
	void TMC5160_SetD1(Stepper_t* motor);

	// W: VSTOP register
	void TMC5160_SetVSTOP(Stepper_t* motor);

	// W: TZEROWAIT register
	void TMC5160_SetTZEROWAIT(Stepper_t* motor);

	// R/W: XTARGET register
	void TMC5160_GetXTARGET(Stepper_t* motor);
	void TMC5160_SetXTARGET(Stepper_t* motor);

	// W: VDCMIN register
	void TMC5160_SetVDCMIN(Stepper_t* motor);

	// R/W: SW_MODE register
	void TMC5160_GetSW_MODE(Stepper_t* motor);
	void TMC5160_SetSW_MODE(Stepper_t* motor);

	// R+WC: RAMP_STAT register
	void TMC5160_GetRAMP_STAT(Stepper_t* motor);
	void TMC5160_ClearRAMP_STAT(Stepper_t* motor);

	// R: XLATCH register
	void TMC5160_GetXLATCH(Stepper_t* motor);

	// R/W: ENCMODE register
	void TMC5160_GetENCMODE(Stepper_t* motor);
	void TMC5160_SetENCMODE(Stepper_t* motor);

	// R/W: X_ENC register
	void TMC5160_GetX_ENC(Stepper_t* motor);
	void TMC5160_SetX_ENC(Stepper_t* motor);

	// W: ENC_CONST register
	void TMC5160_SetENC_CONST(Stepper_t* motor);

	// R: ENC_STATUS register
	void TMC5160_GetENC_STATUS(Stepper_t* motor);

	// R: ENC_LATCH register
	void TMC5160_GetENC_LATCH(Stepper_t* motor);

	// W: ENC_DEVIATION register
	void TMC5160_SetENC_DEVIATION(Stepper_t* motor);

	// W: MSLUT0 register
	void TMC5160_SetMSLUT0(Stepper_t* motor);

	// W: MSLUT1 register
	void TMC5160_SetMSLUT1(Stepper_t* motor);

	// W: MSLUT2 register
	void TMC5160_SetMSLUT2(Stepper_t* motor);

	// W: MSLUT3 register
	void TMC5160_SetMSLUT3(Stepper_t* motor);

	// W: MSLUT4 register
	void TMC5160_SetMSLUT4(Stepper_t* motor);

	// W: MSLUT5 register
	void TMC5160_SetMSLUT5(Stepper_t* motor);

	// W: MSLUT6 register
	void TMC5160_SetMSLUT6(Stepper_t* motor);

	// W: MSLUT7 register
	void TMC5160_SetMSLUT7(Stepper_t* motor);

	// W: MSLUTSEL register
	void TMC5160_SetMSLUTSEL(Stepper_t* motor);

	// W: MSLUTSTART register
	void TMC5160_SetMSLUTSTART(Stepper_t* motor);

	// R: MSCNT register
	void TMC5160_GetMSCNT(Stepper_t* motor);

	// R: MSCURACT register
	void TMC5160_GetMSCURACT(Stepper_t* motor);

	// R/W: CHOPCONF register
	void TMC5160_GetCHOPCONF(Stepper_t* motor);
	void TMC5160_SetCHOPCONF(Stepper_t* motor);

	// R/W: COOLCONF register
	void TMC5160_GetCOOLCONF(Stepper_t* motor);
	void TMC5160_SetCOOLCONF(Stepper_t* motor);

	// W: DCCTRL register
	void TMC5160_SetDCCTRL(Stepper_t* motor);

	// R: DRV_STATUS register
	void TMC5160_GetDRV_STATUS(Stepper_t* motor);

	// R/W: PWMCONF register
	void TMC5160_GetPWMCONF(Stepper_t* motor);
	void TMC5160_SetPWMCONF(Stepper_t* motor);

	// R: PWM_SCALE register
	void TMC5160_GetPWM_SCALE(Stepper_t* motor);

	// R: PWM_AUTO register
	void TMC5160_GetPWM_AUTO(Stepper_t* motor);

	#ifndef USE_SPI
	// R: LOST_STEPS register
	void TMC5160_GetLOST_STEPS(Stepper_t* motor);
	#endif
#endif /* REGISTERS_H_ */