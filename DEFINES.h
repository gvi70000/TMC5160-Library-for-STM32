/*******************************************************************************
 * Copyright © 2024 Ion Grozea
 * www.grozeaion.com
 *******************************************************************************/

#ifndef DEFINES_H_
#define DEFINES_H_

#define USE_SPI								// Use only SPI + SD_MODE = 0
#define USE_SELF_LOCK		// If a gearbox with break or worm gear is used
#define VFS					0.325	// 0.325V Reference voltage for controller
#define RSENSE				0.075	// 75 mOhm Sens resistor on Silent StepStick v1
#define VREF				2.4		// 2.4V Rated voltage of the motor
#define VSUPPLY				10.5	// 10.5V Supply voltage of the motor
#define IRATED				2.4		// 2.4A rated current of the motor
#define R_COIL				1.5		// 1.2 Ohm Phase resistance
#define L_COIL				0.0016	// 1.6 mH Phase inductance

// Rev/sec = V/(2*L*I)/(steps/rev)
#define I_COIL_MIN 218 // Min Coil current A Max RPM 2400Rot/min
#define I_COIL_MAX 1800// Max Coil current A Max RPM 291.667Rot/min

#define	SPS_MIN 12 // Min uSteps/sec
#define	SPS_MAX 8388096 // Max uSteps/sec
#define RPM_TO_T 2386.0929422222222222222222222222 //(T_CLK*400FullSteps*256uSteps)/60seconds
#define VMAX_TO_AMAX_2S 0.00546133333333333333333333333333 (2^17/2[Acceleration time in sec])/12*10^6 [fCLK]

#define	F_CLK				12000000 // TMC5160 Internal clock
#define	T_CLK				1.3981013333333333333333333333333//(16777216/F_CLK) // TMC5160 Internal period
#define	T_CLK_TO_SEC		0.7152557373046875//1/T_CLK // To use steps per second
#define	VMAX_CLK			8388096 // TMC5160 Max speed uS/T_CLK
#define	MAX_SPS				6000000 // TMC5160 Max steps per second
#define SPR					400	// 400 Full steps/revolution
#define SPS_256				102400	// 400*256 uSteps/revolution
#define MIN_TO SEC			60.0f// Seconds in a minute
#define SPS_256_MINUTES		1706.6666666666666666666666666667 // 400*256/60
#define GS_FACTOR			0.01196925541070978882449345925438//(1/256)*((31+1)/32)*(VFS/RSENSE)*(1/1.414213562)
#define VMAX_TO_RPM_CONVERSION 4.1909515857696533203125000000001e-4 // Conversion factor for VMAX to RPM

#define MOTOR_COUNT		4				// Number of motors


#define SPI_DATA_LEN 5
#define SPI_TIMEOUT 100

#define RPM_TO_VMAX(x) (uint32_t)((RPM_TO_T * (x)) + 0.5)
#define VMAX_TO_AMAX(x) (uint32_t)((RPM_TO_T * x)+0.5)

#ifndef USE_SPI
	#define TMC5160_REG_CNT 63 // All
#else
	#define TMC5160_REG_CNT 59 // Only the ones used in SPI mode + SD_MODE = 0
#endif
#define TMC5160_WRITE_BIT 0x80

//R Read only 
//W Write only 
//R/W Read- and writable register 
//R+WC Clear by writing "1" bit 


//  Registers in TMC5160
//1. General Configuration Registers 
#define TMC5160_GCONF          0x00
#define TMC5160_GSTAT          0x01
#ifndef USE_SPI
#define TMC5160_IFCNT          0x02
#define TMC5160_NODECONF      0x03
#endif
#define TMC5160_IOIN		   0x04 // IOIN and OUTPUT
#define TMC5160_X_COMPARE      0x05
#define TMC5160_OTP_PROG       0x06
#define TMC5160_OTP_READ       0x07
#define TMC5160_FACTORY_CONF   0x08
#define TMC5160_SHORT_CONF     0x09
#define TMC5160_DRV_CONF       0x0A
#define TMC5160_GLOBAL_SCALER  0x0B
#define TMC5160_OFFSET_READ    0x0C

//2. Velocity Dependent Driver Feature Control Register Set
#define TMC5160_IHOLD_IRUN     0x10
#define TMC5160_TPOWERDOWN     0x11
#define TMC5160_TSTEP          0x12
#define TMC5160_TPWMTHRS       0x13
#define TMC5160_TCOOLTHRS      0x14
#define TMC5160_THIGH          0x15

//3. Ramp Generator Registers
#define TMC5160_RAMPMODE       0x20
#define TMC5160_XACTUAL        0x21
#define TMC5160_VACTUAL        0x22
#define TMC5160_VSTART         0x23
#define TMC5160_A1             0x24
#define TMC5160_V1             0x25
#define TMC5160_AMAX           0x26
#define TMC5160_VMAX           0x27
#define TMC5160_DMAX           0x28
#define TMC5160_D1             0x2A
#define TMC5160_VSTOP          0x2B
#define TMC5160_TZEROWAIT      0x2C
#define TMC5160_XTARGET        0x2D
#define TMC5160_VDCMIN         0x33
#define TMC5160_SW_MODE         0x34
#define TMC5160_RAMP_STAT       0x35
#define TMC5160_XLATCH         0x36

//4. Encoder Registers
#define TMC5160_ENCMODE        0x38
#define TMC5160_X_ENC           0x39
#define TMC5160_ENC_CONST      0x3A
#define TMC5160_ENC_STATUS     0x3B
#define TMC5160_ENC_LATCH      0x3C
#define TMC5160_ENC_DEVIATION  0x3D

//5. Motor Driver Registers 
#define TMC5160_MSLUT0         0x60
#define TMC5160_MSLUT1         0x61
#define TMC5160_MSLUT2         0x62
#define TMC5160_MSLUT3         0x63
#define TMC5160_MSLUT4         0x64
#define TMC5160_MSLUT5         0x65
#define TMC5160_MSLUT6         0x66
#define TMC5160_MSLUT7         0x67
#define TMC5160_MSLUTSEL       0x68
#define TMC5160_MSLUTSTART     0x69
#define TMC5160_MSCNT          0x6A
#define TMC5160_MSCURACT       0x6B
#define TMC5160_CHOPCONF       0x6C
#define TMC5160_COOLCONF       0x6D
#define TMC5160_DCCTRL         0x6E
#define TMC5160_DRV_STATUS     0x6F
#define TMC5160_PWMCONF        0x70
#define TMC5160_PWM_SCALE       0x71
#define TMC5160_PWM_AUTO       0x72

#ifndef USE_SPI
#define TMC5160_LOST_STEPS     0x73
#endif

#endif /* DEFINES_H_ */
