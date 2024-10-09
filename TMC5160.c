#include "spi.h"
#include "gpio.h"
#include "INA260.h"
#include "TMC5160.h"
#include "DEFINES.h"
#include "STRUCTURES.h"
#include "REGISTERS.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Function to initialize a single motor
void TMC5160_Init(Stepper_t* motor) {
		// Set TBL to 1 {16, 24, 36, 54}
		// Set PWM_FREQ to 1 {2/1024 fCLK, 2/683 fCLK, 2/512 fCLK, 2/410 fCLK} {23.4kHz, 35.1kHz, 46.9kHz, 58.5kHz}
		// Ilow_lim = 0.21875A = 16*(2/1024)*VM/Rcoil for TBL=0, PWM_FREQ=0
//		motor->Reg.CHOPCONF.Val.BitField.TBL = 0;
//		motor->Reg.PWMCONF.Val.BitField.PWM_FREQ = 0;
//		motor->Reg.IHOLD_IRUN.Val.BitField.IRUN = 31;
//		TMC5160_Calculate_GlobalScaler(motor, I_COIL_MIN);

   // Disable the motor before configuration
    TMC5160_EnableMotor(motor, 1);

    // Set CHOPCONF (chopper configuration) to default working values
    motor->Reg.CHOPCONF.Val.BitField.TOFF = 3;  // Enable the driver (min value to switch on)
    motor->Reg.CHOPCONF.Val.BitField.HSTRT = 4; // High side short-to-ground time
    motor->Reg.CHOPCONF.Val.BitField.HEND = 1;  // Low side short-to-ground time
    motor->Reg.CHOPCONF.Val.BitField.TBL = 2;   // Comparator blank time (typical value)
    motor->Reg.CHOPCONF.Val.BitField.CHM = 0;   // SpreadCycle mode
    TMC5160_WriteRegister(motor, TMC5160_CHOPCONF, motor->Reg.CHOPCONF.Val.Value);

    // Set IHOLD_IRUN (current control settings)
    motor->Reg.IHOLD_IRUN.Val.BitField.IHOLD = 2;  // Low holding current
    motor->Reg.IHOLD_IRUN.Val.BitField.IRUN = 31;  // Maximum current for run
    motor->Reg.IHOLD_IRUN.Val.BitField.IHOLDDELAY = 10;  // Delay before reducing current in standstill
    TMC5160_WriteRegister(motor, TMC5160_IHOLD_IRUN, motor->Reg.IHOLD_IRUN.Val.Value);

    // Set TPOWERDOWN (power down delay)
    motor->Reg.TPOWERDOWN.Val.BitField.TPOWERDOWN = 10;  // Power down delay
    TMC5160_WriteRegister(motor, TMC5160_TPOWERDOWN, motor->Reg.TPOWERDOWN.Val.Value);

    // Enable StealthChop for smooth motion (PWM mode)
    motor->Reg.GCONF.Val.BitField.EN_PWM_MODE = 1;  // Enable PWM mode (StealthChop)
    TMC5160_WriteRegister(motor, TMC5160_GCONF, motor->Reg.GCONF.Val.Value);

    // Set RAMPMODE to position mode (target position moves)
    motor->Reg.RAMPMODE.Val.BitField.RAMPMODE = 0;  // Position mode
    TMC5160_WriteRegister(motor, TMC5160_RAMPMODE, motor->Reg.RAMPMODE.Val.Value);

    // Set initial velocity and acceleration settings
    motor->Reg.VMAX.Val.BitField.VMAX = 25000;  // Initial velocity
    TMC5160_WriteRegister(motor, TMC5160_VMAX, motor->Reg.VMAX.Val.Value);

    motor->Reg.AMAX.Val.BitField.AMAX = 5000;  // Acceleration value
    TMC5160_WriteRegister(motor, TMC5160_AMAX, motor->Reg.AMAX.Val.Value);

    motor->Reg.DMAX.Val.BitField.DMAX = 5000;  // Deceleration value
    TMC5160_WriteRegister(motor, TMC5160_DMAX, motor->Reg.DMAX.Val.Value);

    // Set stop velocity to ensure proper stopping
    motor->Reg.VSTOP.Val.BitField.VSTOP = 10;  // Near zero stop velocity
    TMC5160_WriteRegister(motor, TMC5160_VSTOP, motor->Reg.VSTOP.Val.Value);

    // Set TPWMTHRS (switching velocity for StealthChop to SpreadCycle) to a reasonable value
    motor->Reg.TPWMTHRS.Val.BitField.TPWMTHRS = 200;  // Example threshold
    TMC5160_WriteRegister(motor, TMC5160_TPWMTHRS, motor->Reg.TPWMTHRS.Val.Value);

    // Enable the motor after configuration
    TMC5160_EnableMotor(motor, 0);
}

void TMC5160_SetRMSCurrent(Stepper_t *motor, uint16_t current_mA) {
    const uint32_t V_fs = 325;  // 0.325 * 1000
    const uint32_t denom_base = 83198;  // Precomputed denominator base
    const uint32_t CURRENT_SCALING_FACTOR = 11585;  // 32 * 256 * sqrt(2)
    uint_fast8_t CS = 31;  // Initial value for current scaling
    uint32_t scaler = 0;

    // Use integer multiplication and division for RS_scaled calculation
    uint16_t RS_scaled = (motor->Reg.SHORT_CONF.Val.BitField.S2VS_LEVEL * 0xFFFF) / 1000;

    // Precompute numerator using the named constant
    uint32_t numerator = CURRENT_SCALING_FACTOR * RS_scaled;
    numerator = (numerator * current_mA) >> 8;  // Use the passed current in mA

    // Use a loop to adjust CS and compute scaler
    while (CS >= 16) {
        uint32_t denominator = denom_base * (CS + 1);
        scaler = numerator / denominator;

        if (scaler <= 255 && scaler >= 32) {
            break;  // Scaler is within valid range, exit loop
        }

        // If scaler is too small, reduce CS
        if (scaler < 32) {
            CS--;
        } else {
            scaler = 0;  // Scaler exceeds the maximum, set to full scale
            break;
        }
    }

    // Update GLOBAL_SCALER register
    motor->Reg.GLOBAL_SCALER.Val.BitField.GLOBAL_SCALER = scaler;

    // Update IHOLD_IRUN register
    motor->Reg.IHOLD_IRUN.Val.BitField.IRUN = (CS > 31) ? 31 : CS;

    // Use IHOLD only if USE_SELF_LOCK is not defined
    #ifndef USE_SELF_LOCK
    motor->Reg.IHOLD_IRUN.Val.BitField.IHOLD = (motor->Reg.IHOLD_IRUN.Val.BitField.IRUN * motor->Reg.IHOLD_IRUN.Val.BitField.IHOLDDELAY) / 100;
    #else
    motor->Reg.IHOLD_IRUN.Val.BitField.IHOLD = 0;  // No hold current for self-locking systems
    #endif

    // Write updated values to the respective registers
    TMC5160_SetGLOBAL_SCALER(motor);
    TMC5160_SetIHOLD_IRUN(motor);
}

void TMC5160_GetMaxSpeed(Stepper_t* motor) {
    uint32_t vmax = 20000;  // Start with a base speed (can be adjusted)
    uint32_t maxSpeedReached = vmax;
    uint32_t prevPosition = 0;
    uint32_t currentPosition = 0;
    uint32_t speed = 0;
    uint32_t prevSpeed = 0;
    int stableCount = 0;  // Counter to track stable speed to exit the loop
    const int stableThreshold = 5;  // Number of stable readings to consider speed limit reached
    uint32_t startTime = HAL_GetTick();  // Get the start time (in milliseconds)
    float current_A = 0;  // Variable to store current from INA260

    // Get current microstepping resolution from CHOPCONF register
    TMC5160_GetCHOPCONF(motor);
    uint16_t microsteps = 256;  // Default to 256 microsteps

    // Set initial velocity mode
    motor->Reg.RAMPMODE.Val.BitField.RAMPMODE = 1;  // Velocity mode to positive direction
    TMC5160_SetRAMPMODE(motor);

    // Enable motor
    TMC5160_EnableMotor(motor, 0);

    while (stableCount < stableThreshold) {
        // Check if the motor has not reached velocity in 5 seconds (5000 ms)
        if ((HAL_GetTick() - startTime) > 5000) {
            printf("Motor did not reach the target velocity in 5 seconds.\n");
            break;  // Exit the loop and ramp down
        }

        // Apply the speed limitations based on VMAX_CLK and MAX_SPS
        if (vmax > VMAX_CLK) {
            vmax = VMAX_CLK;  // Limit to maximum allowable velocity (microsteps per second)
        }

        // Set velocity
        motor->Reg.VMAX.Val.BitField.VMAX = vmax;  // Set velocity in VMAX units
        TMC5160_SetVMAX(motor);

        // Wait for the motor to reach its new speed
        do {
            // Delay to give time for the motor to move
            HAL_Delay(50);  // Short delay, adjust if necessary

            // Get current motor position (XACTUAL register)
            TMC5160_GetXACTUAL(motor);
            currentPosition = motor->Reg.XACTUAL.Val.Value;

            // Calculate speed (difference in position over time)
            speed = currentPosition - prevPosition;

            // Read current from INA260
            if (INA260_GetCurrent(&current_A) == HAL_OK) {
                printf("Current Draw: %.3f A\n", current_A);
            } else {
                printf("Error reading current from INA260\n");
            }

        } while (speed == 0);  // Wait until the motor starts moving

        // If the speed is stable or decreasing, exit the loop
        if (speed <= prevSpeed) {
            stableCount++;
        } else {
            stableCount = 0;  // Reset the counter if speed is increasing
            maxSpeedReached = vmax;  // Track the maximum speed achieved
        }

        // Calculate speed in RPM based on VMAX
        float rpm = (vmax * VMAX_TO_RPM_CONVERSION);

        // Print current speed in VMAX units, RPM, microstepping, and VMAX on one line
        printf("VMAX: %d usteps, RPM: %.2f, VMAX Register: %d\n", vmax, rpm, vmax);

        // Increment the speed for the next iteration
        vmax += 20000;  // Adjust increment value as needed
        prevPosition = currentPosition;
        prevSpeed = speed;
        startTime = HAL_GetTick();
    }

    // Start ramping down speed over 10 seconds
    uint32_t rampDownStart = HAL_GetTick();  // Track the start of the ramp-down period
    uint32_t rampDownTime = 10000;  // Ramp down over 10 seconds
    uint32_t rampStepTime = 100;  // Time between each reduction step (100 ms)
    uint32_t numSteps = rampDownTime / rampStepTime;  // Total number of steps to ramp down
    uint32_t decrement = maxSpeedReached / numSteps;  // Amount to decrease on each step

    while (vmax > 0) {
        // Gradually reduce vmax
        if ((HAL_GetTick() - rampDownStart) >= rampStepTime) {
            vmax = (vmax > decrement) ? vmax - decrement : 0;  // Reduce speed or set to 0
            motor->Reg.VMAX.Val.BitField.VMAX = vmax;  // Update VMAX register
            TMC5160_SetVMAX(motor);  // Apply new velocity
            rampDownStart = HAL_GetTick();  // Reset start time for next step
        }

        // Print current speed during ramp down
        float rpm = (vmax * VMAX_TO_RPM_CONVERSION);
        printf("Ramping down. VMAX: %d usteps, RPM: %.2f\n", vmax, rpm);

        HAL_Delay(50);  // Short delay between each loop
    }

    // Disable motor
    TMC5160_EnableMotor(motor, 1);  // 1 disables the motor

    // Print final result on one line
    float finalRPM = (float)(maxSpeedReached * 4.1909515857696533203125000000001e-4);
    printf("Maximum speed reached: VMAX: %d usteps, RPM: %.2f\n", maxSpeedReached, finalRPM);
}


// Function to monitor CoolStep parameters
void MonitorCoolStep(Stepper_t* motor, uint32_t* stallguard, uint32_t* current_scale) {
    *stallguard = motor->Reg.DRV_STATUS.Val.BitField.SG_RESULT;
    *current_scale = motor->Reg.PWM_SCALE.Val.BitField.PWM_SCALE_AUTO;

    printf("StallGuard2 result: %d\n", *stallguard);
    printf("Current scaling (PWM_SCALE_AUTO): %d\n", *current_scale);
}

void TMC5160_TuneCoolStep(Stepper_t* motor) {
    const uint32_t maxIterations = 100;
    const uint32_t velocity = RPM_TO_VMAX(100);
    
    uint32_t best_sg_result = 0;
    uint32_t best_sg_threshold = 0;
    uint32_t best_semin = 0;
    uint32_t best_semax = 0;
    uint32_t best_current_scale = 0;
    uint32_t stallguard = 0;
    uint32_t current_scale = 0;

    motor->Reg.IHOLD_IRUN.Val.BitField.IRUN = 31;
    motor->Reg.IHOLD_IRUN.Val.BitField.IHOLD = 0;
    TMC5160_SetIHOLD_IRUN(motor);

    for (uint32_t iteration = 0; iteration < maxIterations; iteration++) {
        motor->Reg.COOLCONF.Val.BitField.SGT = (iteration % 64) - 32;
        motor->Reg.COOLCONF.Val.BitField.SEMIN = 5 + (iteration % 10);
        motor->Reg.COOLCONF.Val.BitField.SEMAX = 10 + (iteration % 5);
        TMC5160_SetCOOLCONF(motor);

        motor->Reg.VMAX.Val.BitField.VMAX = velocity;
        TMC5160_SetVMAX(motor);
        
        motor->Reg.XTARGET.Val.Value += SPS_256;
        TMC5160_SetXTARGET(motor);

        MonitorCoolStep(motor, &stallguard, &current_scale);
        TMC5160_WaitForPosition(motor, motor->Reg.XTARGET.Val.Value);

        if (stallguard > best_sg_result || current_scale < best_current_scale) {
            best_sg_result = stallguard;
            best_sg_threshold = motor->Reg.COOLCONF.Val.BitField.SGT;
            best_semin = motor->Reg.COOLCONF.Val.BitField.SEMIN;
            best_semax = motor->Reg.COOLCONF.Val.BitField.SEMAX;
            best_current_scale = current_scale;

            printf("New optimal settings found: SGT=%d, SEMIN=%d, SEMAX=%d, Current Scale=%d\n",
                best_sg_threshold, best_semin, best_semax, best_current_scale);
        }

        motor->Reg.VMAX.Val.BitField.VMAX = 0;
        TMC5160_SetVMAX(motor);
    }

    motor->Reg.COOLCONF.Val.BitField.SGT = best_sg_threshold;
    motor->Reg.COOLCONF.Val.BitField.SEMIN = best_semin;
    motor->Reg.COOLCONF.Val.BitField.SEMAX = best_semax;
    TMC5160_SetCOOLCONF(motor);

    printf("Final CoolStep tuning: SGT=%d, SEMIN=%d, SEMAX=%d, Current Scale=%d\n",
           best_sg_threshold, best_semin, best_semax, best_current_scale);
}

void TMC5160_AutoTuneStealthChop(Stepper_t* motor) {
    const uint32_t AT2_FullSteps = 400;
    const uint32_t fPWM = 23400;

    uint32_t best_pwm_ofs = 0, best_pwm_grad = 0, best_pwm_scale_auto = 0;
    uint32_t current_pwm_scale_auto = 0;
    
    TMC5160_GetDRV_STATUS(motor);
    if (motor->Reg.DRV_STATUS.Val.BitField.STST) {
        //TMC5160_SetIRunToIHold(motor);
        TMC5160_GetPWMCONF(motor);
        motor->Reg.PWMCONF.Val.BitField.PWM_AUTOSCALE = 1;
        TMC5160_SetPWMCONF(motor);

        for (int i = 0; i < 10; i++) {
            TMC5160_SlowMoveMotor(motor, 8);
            TMC5160_WaitForPosition(motor, motor->Reg.XTARGET.Val.Value);
            TMC5160_StopMotor(motor);
        }

        TMC5160_GetPWM_AUTO(motor);
        if (motor->Reg.PWM_AUTO.Val.BitField.PWM_OFS_AUTO == 0) {
            return;
        }
    }

    for (uint32_t pwm_ofs = 0; pwm_ofs <= 255; pwm_ofs += 10) {
        for (uint32_t pwm_grad = 0; pwm_grad <= 255; pwm_grad += 10) {
            motor->Reg.PWM_AUTO.Val.BitField.PWM_OFS_AUTO = pwm_ofs;
            motor->Reg.PWM_AUTO.Val.BitField.PWM_GRAD_AUTO = pwm_grad;
            TMC5160_SetPWMCONF(motor);

            TMC5160_MoveMotorAtRPM(motor, 100);
            TMC5160_WaitForPosition(motor, motor->Reg.XTARGET.Val.Value);

            TMC5160_GetPWM_SCALE(motor);
            current_pwm_scale_auto = motor->Reg.PWM_SCALE.Val.BitField.PWM_SCALE_AUTO;

            if (current_pwm_scale_auto == 0) {
                best_pwm_ofs = pwm_ofs;
                best_pwm_grad = pwm_grad;
                best_pwm_scale_auto = current_pwm_scale_auto;
                TMC5160_StopMotor(motor);
                break;
            }
        }
        if (best_pwm_scale_auto == 0) {
            break;
        }
    }

    motor->Reg.PWM_AUTO.Val.BitField.PWM_OFS_AUTO = best_pwm_ofs;
    motor->Reg.PWM_AUTO.Val.BitField.PWM_GRAD_AUTO = best_pwm_grad;
    TMC5160_SetPWMCONF(motor);

    printf("StealthChop tuning completed with PWM_OFS_AUTO=%d, PWM_GRAD_AUTO=%d\n", best_pwm_ofs, best_pwm_grad);
}

void TMC5160_MoveMotorAtRPM(Stepper_t* motor, uint32_t rpm) {
    uint32_t velocity = RPM_TO_VMAX(rpm);
    motor->Reg.VMAX.Val.BitField.VMAX = velocity;
    TMC5160_SetVMAX(motor);

    motor->Reg.XTARGET.Val.Value += SPS_256;
    TMC5160_SetXTARGET(motor);
    TMC5160_WaitForPosition(motor, motor->Reg.XTARGET.Val.Value);
}

void TMC5160_SlowMoveMotor(Stepper_t* motor, uint32_t steps) {
    TMC5160_MoveMotorAtRPM(motor, 8);
    motor->Reg.XTARGET.Val.Value += steps;
    TMC5160_SetXTARGET(motor);

    TMC5160_WaitForPosition(motor, motor->Reg.XTARGET.Val.Value);
    motor->Reg.VMAX.Val.BitField.VMAX = 0;
    TMC5160_SetVMAX(motor);
}

void TMC5160_StopMotor(Stepper_t* motor) {
    motor->Reg.VMAX.Val.BitField.VMAX = 0;
    TMC5160_SetVMAX(motor);
}

void TMC5160_WaitForPosition(Stepper_t* motor, int32_t targetPosition) {
    uint32_t startTime = HAL_GetTick(); // Get current time
    uint32_t timeout = 5000; // Set timeout to 5000ms (5 seconds)
    
    while (motor->Reg.XACTUAL.Val.Value != targetPosition) {
        TMC5160_GetXACTUAL(motor);
        
        // Break the loop if it exceeds the timeout period
        if ((HAL_GetTick() - startTime) > timeout) {
            printf("Timeout: Motor did not reach target position\n");
            break;
        }
    }
}



