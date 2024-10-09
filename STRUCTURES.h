/******************************************************************************* 
 * Copyright © 2024 Ion Grozea 
 * www.grozeaion.com 
 *******************************************************************************/

#ifndef STRUCTURES_H_
#define STRUCTURES_H_

#include "gpio.h"
#include "DEFINES.h"
#include <stdint.h>

/* 0x00 RW GCONF General configuration register 18 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t RECALIBRATE            : 1;
            uint32_t FASTSTANDSTILL         : 1;
            uint32_t EN_PWM_MODE            : 1;
            uint32_t MULTISTEP_FILT         : 1;
            uint32_t SHAFT                  : 1;
            uint32_t DIAG0_ERROR            : 1;
            uint32_t DIAG0_OTPW             : 1;
            uint32_t DIAG0_STALL_STEP       : 1;
            uint32_t DIAG1_STALL_DIR        : 1;
            uint32_t DIAG1_INDEX            : 1;
            uint32_t DIAG1_ONSTATE          : 1;
            uint32_t DIAG1_STEPS_SKIPPED    : 1;
            uint32_t DIAG0_INT_PUSHPULL     : 1;
            uint32_t DIAG1_POSCOMP_PUSHPULL : 1;
            uint32_t SMALL_HYSTERESIS       : 1;
            uint32_t STOP_ENABLE            : 1;
            uint32_t DIRECT_MODE            : 1;
            uint32_t TEST_MODE              : 1;
            uint32_t RESERVED               : 14;
        } BitField;
    } Val;
} GCONF_Register;

/* 0x01 R+WC GSTAT Global status flags 3 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t RESET                  : 1;
            uint32_t DRV_ERR                : 1;
            uint32_t UV_CP                  : 1;
            uint32_t RESERVED               : 29;
        } BitField;
    } Val;
} GSTAT_Register;

/* 0x02 R IFCNT Interface transmission counter 8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t COUNT                  : 8;
            uint32_t RESERVED               : 24;
        } BitField;
    } Val;
} IFCNT_Register;

/* 0x03 W NODECONF Read input pins 8+4 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t NODEADDR               : 8;
            uint32_t SENDDELAY              : 4;
            uint32_t RESERVED               : 20;
        } BitField;
    } Val;
} NODECONF_Register;

/* 0x04 R IOIN Read input pins 8+8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t REFL_STEP              : 1;
            uint32_t REFR_DIR               : 1;
            uint32_t ENCB_DCEN_CFG4         : 1;
            uint32_t ENCA_DCIN_CFG5         : 1;
            uint32_t DRV_ENN                : 1;
            uint32_t ENC_N_DCO_CFG6         : 1;
            uint32_t SD_MODE                : 1;
            uint32_t SWCOMP_IN              : 1;
            uint32_t RESERVED               : 16;
            uint32_t VERSION                : 8;
        } BitField;
    } Val;
} IOIN_Register;

/* 0x04 W OUTPUT Sets the IO output pin polarity in UART mode 1 Bit*/
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t POLARITY               : 1; //00 Sets the IO output pin polarity in UART mode
            uint32_t RESERVED               : 31;
        } BitField;
    } Val;
} OUTPUT_Register;

/* 0x05 W X_COMPARE Position comparison register for motion controller position strobe 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t X_COMPARE               : 32;
        } BitField;
    } Val;
} X_COMPARE_Register;

/* 0x06 W OTP_PROG OTP programming 3+2+8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t OTPBIT                 : 3;
            uint32_t RESERVED0              : 1;
            uint32_t OTPBYTE                : 2;
            uint32_t RESERVED1              : 2;
            uint32_t OTPMAGIC               : 8;
            uint32_t RESERVED2              : 16;
        } BitField;
    } Val;
} OTP_PROG_Register;

/* 0x07 R OTP_READ (Access to OTP memory result and update) 8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t OTP_FCLKTRIM           : 5;
            uint32_t OTP_S2_LEVEL           : 1;
            uint32_t OTP_BBM                : 1;
            uint32_t OTP_TBL                : 1;
            uint32_t RESERVED               : 24;
        } BitField;
    } Val;
} OTP_READ_Register;

/* 0x08 RW FACTORY_CONF FCLKTRIM (Reset default: OTP)0…31: Lowest to highest clock frequency 5 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t FCLKTRIM               : 5;
            uint32_t RESERVED               : 27;
        } BitField;
    } Val;
} FACTORY_CONF_Register;

/* 0x09 W SHORT_CONF 19 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t S2VS_LEVEL             : 4;
            uint32_t RESERVED0              : 4;
            uint32_t S2G_LEVEL              : 4;
            uint32_t RESERVED1              : 4;
            uint32_t SHORTFILTER            : 2;
            uint32_t SHORTDELAY             : 1;
            uint32_t RESERVED2              : 13;
        } BitField;
    } Val;
} SHORT_CONF_Register;

/* 0x0A W DRV_CONF Register Structure 22 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t BBMTIME                : 5;
            uint32_t RESERVED0              : 3;
            uint32_t BBMCLKS                : 4;
            uint32_t RESERVED1              : 4;
            uint32_t OTSELECT               : 2;
            uint32_t DRVSTRENGTH            : 2;
            uint32_t FILT_ISENSE            : 2;
            uint32_t RESERVED2              : 10;
        } BitField;
    } Val;
} DRV_CONF_Register;

/* 0x0B W GLOBAL_SCALER Register Structure 8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t GLOBAL_SCALER          : 8;
            uint32_t RESERVED               : 24;
        } BitField;
    } Val;
} GLOBAL_SCALER_Register;

/* 0x0C R OFFSET_READ Register Structure 16 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t OFFSET_B               : 8;
            uint32_t OFFSET_A               : 8;
            uint32_t RESERVED               : 16;
        } BitField;
    } Val;
} OFFSET_READ_Register;

/* 0x10 W IHOLD_IRUN Register Structure 5+5+4 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t IHOLD                  : 5;
            uint32_t RESERVED0              : 3;
            uint32_t IRUN                   : 5;
            uint32_t RESERVED1              : 3;
            uint32_t IHOLDDELAY             : 4;
            uint32_t RESERVED2              : 12;
        } BitField;
    } Val;
} IHOLD_IRUN_Register;

/* 0x11 W TPOWERDOWN Register Structure 8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t TPOWERDOWN             : 8;
            uint32_t RESERVED               : 24;
        } BitField;
    } Val;
} TPOWERDOWN_Register;

/* 0x12 R TSTEP Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t TSTEP                  : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} TSTEP_Register;

/* 0x13 W TPWMTHRS Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t TPWMTHRS               : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} TPWMTHRS_Register;

/* 0x14 W TCOOLTHRS Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t TCOOLTHRS              : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} TCOOLTHRS_Register;

/* 0x15 W THIGH Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t THIGH                  : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} THIGH_Register;

/* 0x20 RW RAMPMODE Register Structure 2 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t RAMPMODE               : 2;
            uint32_t RESERVED               : 30;
        } BitField;
    } Val;
} RAMPMODE_Register;

/* 0x21 RW XACTUAL Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t XACTUAL                : 32;
        } BitField;
    } Val;
} XACTUAL_Register;

/* 0x22 R VACTUAL Register Structure 24 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t VACTUAL                : 24;
            uint32_t RESERVED               : 8;
        } BitField;
    } Val;
} VACTUAL_Register;

/* 0x23 W VSTART Register Structure 18 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t VSTART                 : 18;
            uint32_t RESERVED               : 14;
        } BitField;
    } Val;
} VSTART_Register;

/* 0x24 W A1 Register Structure 16 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t A1                     : 16;
            uint32_t RESERVED               : 16;
        } BitField;
    } Val;
} A1_Register;

/* 0x25 W V1 Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t V1                     : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} V1_Register;

/* 0x26 W AMAX Register Structure 16 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t AMAX                   : 16;
            uint32_t RESERVED               : 16;
        } BitField;
    } Val;
} AMAX_Register;

/* 0x27 W VMAX Register Structure 23 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t VMAX                   : 23;
            uint32_t RESERVED               : 9;
        } BitField;
    } Val;
} VMAX_Register;

/* 0x28 W DMAX Register Structure 16 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t DMAX                   : 16;
            uint32_t RESERVED               : 16;
        } BitField;
    } Val;
} DMAX_Register;

/* 0x2A W D1 Register Structure 16 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t D1                     : 16;
            uint32_t RESERVED               : 16;
        } BitField;
    } Val;
} D1_Register;

/* 0x2B W VSTOP Register Structure 18 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t VSTOP                  : 18;
            uint32_t RESERVED               : 14;
        } BitField;
    } Val;
} VSTOP_Register;

/* 0x2C W TZEROWAIT Register Structure 16 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t TZEROWAIT              : 16;
            uint32_t RESERVED               : 16;
        } BitField;
    } Val;
} TZEROWAIT_Register;

/* 0x2D RW XTARGET Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t XTARGET                : 32;
        } BitField;
    } Val;
} XTARGET_Register;

/* 0x33 W VDCMIN Register Structure 23 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t VDCMIN                 : 23;
            uint32_t RESERVED               : 9;
        } BitField;
    } Val;
} VDCMIN_Register;

/* 0x34 RW SW_MODE Register Structure 12 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t STOP_L_ENABLE          : 1;
            uint32_t STOP_R_ENABLE          : 1;
            uint32_t POL_STOP_L             : 1;
            uint32_t POL_STOP_R             : 1;
            uint32_t SWAP_LR                : 1;
            uint32_t LATCH_L_ACTIVE         : 1;
            uint32_t LATCH_L_INACTIVE       : 1;
            uint32_t LATCH_R_ACTIVE         : 1;
            uint32_t LATCH_R_INACTIVE       : 1;
            uint32_t EN_LATCH_ENCODER       : 1;
            uint32_t SG_STOP                : 1;
            uint32_t EN_SOFTSTOP            : 1;
            uint32_t RESERVED               : 20;
        } BitField;
    } Val;
} SW_MODE_Register;

/* 0x35 R+WC RAMP_STAT Register Structure 14 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t STATUS_STOP_L          : 1;
            uint32_t STATUS_STOP_R          : 1;
            uint32_t STATUS_LATCH_L         : 1;
            uint32_t STATUS_LATCH_R         : 1;
            uint32_t EVENT_STOP_L           : 1;
            uint32_t EVENT_STOP_R           : 1;
            uint32_t EVENT_STOP_SG          : 1;
            uint32_t EVENT_POS_REACHED      : 1;
            uint32_t VELOCITY_REACHED       : 1;
            uint32_t POSITION_REACHED       : 1;
            uint32_t VZERO                  : 1;
            uint32_t T_ZEROWAIT_ACTIVE      : 1;
            uint32_t SECOND_MOVE            : 1;
            uint32_t STATUS_SG              : 1;
            uint32_t RESERVED               : 18;
        } BitField;
    } Val;
} RAMP_STAT_Register;

/* 0x36 R XLATCH Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t XLATCH                 : 32;
        } BitField;
    } Val;
} XLATCH_Register;

/* 0x38 RW ENCMODE Register Structure 11 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t POL_A                  : 1;
            uint32_t POL_B                  : 1;
            uint32_t POL_N                  : 1;
            uint32_t IGNORE_AB              : 1;
            uint32_t CLR_CONT               : 1;
            uint32_t CLR_ONCE               : 1;
            uint32_t POS_EDGE               : 1;
            uint32_t NEG_EDGE               : 1;
            uint32_t CLR_ENC_X              : 1;
            uint32_t LATCH_X_ACT            : 1;
            uint32_t ENC_SEL_DEC            : 1;
            uint32_t RESERVED               : 21;
        } BitField;
    } Val;
} ENCMODE_Register;

/* 0x39 RW X_ENC Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t X_ENC                  : 32;
        } BitField;
    } Val;
} X_ENC_Register;

/* 0x3A W ENC_CONST Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t ENC_CONST              : 32;
        } BitField;
    } Val;
} ENC_CONST_Register;

/* 0x3B R ENC_STATUS Register Structure 2 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t ENC_STATUS             : 2;
            uint32_t RESERVED               : 30;
        } BitField;
    } Val;
} ENC_STATUS_Register;

/* 0x3C R ENC_LATCH Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t ENC_LATCH              : 32;
        } BitField;
    } Val;
} ENC_LATCH_Register;

/* 0x3D W ENC_DEVIATION Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t ENC_DEVIATION          : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} ENC_DEVIATION_Register;

/* 0x60 W MSLUT[0] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT0                 : 32;
        } BitField;
    } Val;
} MSLUT0_Register;

/* 0x61 W MSLUT[1] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT1                 : 32;
        } BitField;
    } Val;
} MSLUT1_Register;

/* 0x62 W MSLUT[2] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT2                 : 32;
        } BitField;
    } Val;
} MSLUT2_Register;

/* 0x63 W MSLUT[3] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT3                 : 32;
        } BitField;
    } Val;
} MSLUT3_Register;

/* 0x64 W MSLUT[4] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT4                 : 32;
        } BitField;
    } Val;
} MSLUT4_Register;

/* 0x65 W MSLUT[5] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT5                 : 32;
        } BitField;
    } Val;
} MSLUT5_Register;

/* 0x66 W MSLUT[6] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT6                 : 32;
        } BitField;
    } Val;
} MSLUT6_Register;

/* 0x67 W MSLUT[7] Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSLUT7                 : 32;
        } BitField;
    } Val;
} MSLUT7_Register;

/* 0x68 W MSLUTSEL Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t W0                     : 2;
            uint32_t W1                     : 2;
            uint32_t W2                     : 2;
            uint32_t W3                     : 2;
            uint32_t X1                     : 8;
            uint32_t X2                     : 8;
            uint32_t X3                     : 8;
        } BitField;
    } Val;
} MSLUTSEL_Register;

/* 0x69 W MSLUTSTART Register Structure 8+8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t START_SIN              : 8;
            uint32_t RESERVED               : 8;
            uint32_t START_SIN90            : 8;
            uint32_t RESERVED1              : 8;
        } BitField;
    } Val;
} MSLUTSTART_Register;

/* 0x6A RW MSCNT Register Structure 10 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t MSCNT                  : 10;
            uint32_t RESERVED               : 22;
        } BitField;
    } Val;
} MSCNT_Register;

/* 0x6B R MSCURACT Register Structure 9+9 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t CUR_A                  : 9;
            uint32_t RESERVED               : 7;
            uint32_t CUR_B                  : 9;
            uint32_t RESERVED1              : 7;
        } BitField;
    } Val;
} MSCURACT_Register;

/* 0x6C RW CHOPCONF Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t TOFF                   : 4;
            uint32_t HSTRT                  : 3;
            uint32_t HEND                   : 4;
            uint32_t TFD                    : 1;
            uint32_t DISFDCC                : 1;
            uint32_t RESERVED0              : 1;
            uint32_t CHM                    : 1;
            uint32_t TBL                    : 2;
            uint32_t RESERVED1              : 1;
            uint32_t VHIGHFS                : 1;
            uint32_t VHIGHCHM               : 1;
            uint32_t TPFD                   : 4;
            uint32_t MRES                   : 4;
            uint32_t INTPOL                 : 1;
            uint32_t DEDGE                  : 1;
            uint32_t DISS2G                 : 1;
            uint32_t DISS2VS                : 1;
        } BitField;
    } Val;
} CHOPCONF_Register;

/* 0x6D R COOLCONF Register Structure 25 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t SEMIN                  : 4;
            uint32_t RESERVED0              : 1;
            uint32_t SEUP                   : 2;
            uint32_t RESERVED1              : 1;
            uint32_t SEMAX                  : 4;
            uint32_t RESERVED2              : 1;
            uint32_t SEDN                   : 2;
            uint32_t SEIMIN                 : 1;
            uint32_t SGT                    : 7;
            uint32_t RESERVED3              : 1;
            uint32_t SFILT                  : 1;
            uint32_t RESERVED4              : 7;
        } BitField;
    } Val;
} COOLCONF_Register;

/* 0x6E W DCCTRL Register Structure 24 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t SEMDC_TIME             : 10;
            uint32_t RESERVED0              : 6;
            uint32_t DC_SG                  : 8;
            uint32_t RESERVED1              : 8;
        } BitField;
    } Val;
} DCCTRL_Register;

/* 0x6F R DRV_STATUS Register Structure 32 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t SG_RESULT              : 10;
            uint32_t RESERVED0              : 2;
            uint32_t S2VSA                  : 1;
            uint32_t S2VSB                  : 1;
            uint32_t STEALTH                : 1;
            uint32_t FSACTIVE               : 1;
            uint32_t CS_ACTUAL              : 5;
            uint32_t RESERVED1              : 3;
            uint32_t STALLGUARD             : 1;
            uint32_t OT                     : 1;
            uint32_t OTPW                   : 1;
            uint32_t S2GA                   : 1;
            uint32_t S2GB                   : 1;
            uint32_t OLA                    : 1;
            uint32_t OLB                    : 1;
            uint32_t STST                   : 1;
        } BitField;
    } Val;
} DRV_STATUS_Register;

/* 0x70 W PWMCONF Register Structure 22 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t PWM_OFS                : 8;
            uint32_t PWM_GRAD               : 8;
            uint32_t PWM_FREQ               : 2;
            uint32_t PWM_AUTOSCALE          : 1;
            uint32_t PWM_AUTOGRAD           : 1;
            uint32_t FREEWHEEL              : 2;
            uint32_t RESERVED               : 2;
            uint32_t PWM_REG                : 4;
            uint32_t PWM_LIM                : 4;
        } BitField;
    } Val;
} PWMCONF_Register;

/* 0x71 R PWM_SCALE Register Structure 9+8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t PWM_SCALE_SUM          : 8;
            uint32_t RESERVED0              : 8;
            uint32_t PWM_SCALE_AUTO         : 9;
            uint32_t RESERVED1              : 7;
        } BitField;
    } Val;
} PWM_SCALE_Register;

/* 0x72 R PWM_AUTO Register Structure 8+8 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t PWM_OFS_AUTO           : 8;
            uint32_t RESERVED0              : 8;
            uint32_t PWM_GRAD_AUTO          : 8;
            uint32_t RESERVED1              : 8;
        } BitField;
    } Val;
} PWM_AUTO_Register;

/* 0x73 R LOST_STEPS Register Structure 20 Bits */
typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t Value;
        struct __attribute__((packed, aligned(4))) {
            uint32_t LOST_STEPS             : 20;
            uint32_t RESERVED               : 12;
        } BitField;
    } Val;
} LOST_STEPS_Register;

typedef struct __attribute__((packed, aligned(4))) {
    GCONF_Register GCONF;
    GSTAT_Register GSTAT;
#ifndef USE_SPI
    IFCNT_Register IFCNT;
    NODECONF_Register NODECONF;
#endif
    IOIN_Register IOIN;
    X_COMPARE_Register X_COMPARE;
    OTP_PROG_Register OTP_PROG;
    OTP_READ_Register OTP_READ;
    FACTORY_CONF_Register FACTORY_CONF;
    SHORT_CONF_Register SHORT_CONF;
    DRV_CONF_Register DRV_CONF;
    GLOBAL_SCALER_Register GLOBAL_SCALER;
    OFFSET_READ_Register OFFSET_READ;
    IHOLD_IRUN_Register IHOLD_IRUN;
    TPOWERDOWN_Register TPOWERDOWN;
    TSTEP_Register TSTEP;
    TPWMTHRS_Register TPWMTHRS;
    TCOOLTHRS_Register TCOOLTHRS;
    THIGH_Register THIGH;
    RAMPMODE_Register RAMPMODE;
    XACTUAL_Register XACTUAL;
    VACTUAL_Register VACTUAL;
    VSTART_Register VSTART;
    A1_Register A1;
    V1_Register V1;
    AMAX_Register AMAX;
    VMAX_Register VMAX;
    DMAX_Register DMAX;
    D1_Register D1;
    VSTOP_Register VSTOP;
    TZEROWAIT_Register TZEROWAIT;
    XTARGET_Register XTARGET;
    VDCMIN_Register VDCMIN;
    SW_MODE_Register SW_MODE;
    RAMP_STAT_Register RAMP_STAT;
    XLATCH_Register XLATCH;
    ENCMODE_Register ENCMODE;
    X_ENC_Register X_ENC;
    ENC_CONST_Register ENC_CONST;
    ENC_STATUS_Register ENC_STATUS;
    ENC_LATCH_Register ENC_LATCH;
    ENC_DEVIATION_Register ENC_DEVIATION;
    MSLUT0_Register MSLUT0;
    MSLUT1_Register MSLUT1;
    MSLUT2_Register MSLUT2;
    MSLUT3_Register MSLUT3;
    MSLUT4_Register MSLUT4;
    MSLUT5_Register MSLUT5;
    MSLUT6_Register MSLUT6;
    MSLUT7_Register MSLUT7;
    MSLUTSEL_Register MSLUTSEL;
    MSLUTSTART_Register MSLUTSTART;
    MSCNT_Register MSCNT;
    MSCURACT_Register MSCURACT;
    CHOPCONF_Register CHOPCONF;
    COOLCONF_Register COOLCONF;
    DCCTRL_Register DCCTRL;
    DRV_STATUS_Register DRV_STATUS;
    PWMCONF_Register PWMCONF;
    PWM_SCALE_Register PWM_SCALE;
    PWM_AUTO_Register PWM_AUTO;
#ifndef USE_SPI
    LOST_STEPS_Register LOST_STEPS;
#endif
} TMC_Registers;

/* Motor control pins structure */
typedef struct __attribute__((packed, aligned(4))) {
    GPIO_TypeDef* const ctrlPort;
    const uint16_t selectPin;
    const uint16_t enablePin;
} Pins_t;

/* Main motor structure with control pins and registers */
typedef struct __attribute__((packed, aligned(4))) {
    Pins_t ControlPins;
    TMC_Registers Reg;
} Stepper_t;

#endif /* STRUCTURES_H_ */
