/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"




// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */
extern P    rtP_Left;

extern DW   rtDW_Right;                 /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
// ###############################################################################



static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
int16_t I_BusR=0; // Right driver total current in x10 resolution

int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

volatile int pwml = 0;
volatile int pwmr = 0;
extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;
static uint8_t  brkResActiveL = 0;
static uint8_t  brkResActiveExt = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

static uint16_t pwm_res;              /* Timer auto-reload value; set during initialization */
 static volatile int ur, vr, wr;

void BLDC_SetPwmResolution(uint16_t periodCounts) {
  pwm_res = periodCounts ? periodCounts : 1U;
}

static uint16_t offsetcount = 0;
static int16_t offsetrlA    = 2000;
static int16_t offsetrlB    = 2000;
static int16_t offsetrrB    = 2000;
static int16_t offsetrrC    = 2000;
static int16_t offsetdcl    = 2000;
static int16_t offsetdcr    = 2000;

extern int16_t batVoltageCalib;
int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point
int32_t emulated_mech_angle_deg = 0; // For encoder simulation during alignment
// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
void DMA1_Channel1_IRQHandler(void) {

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if(offsetcount < 2000) {  // calibrate ADC offsets
    offsetcount++;
  offsetrlA = (adc_buffer.adc12.value.rlA + offsetrlA) / 2;
  offsetrlB = (adc_buffer.adc12.value.rlB + offsetrlB) / 2;
  offsetrrB = (adc_buffer.adc12.value.rrB + offsetrrB) / 2;
  offsetrrC = (adc_buffer.adc12.value.rrC + offsetrrC) / 2;
  offsetdcl = (adc_buffer.adc12.value.dcl + offsetdcl) / 2;
  offsetdcr = (adc_buffer.adc12.value.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
  filtLowPass32(adc_buffer.adc3.value.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  curL_phaA = (int16_t)(offsetrlA - adc_buffer.adc12.value.rlA);
  curL_phaB = (int16_t)(offsetrlB - adc_buffer.adc12.value.rlB);
  curL_DC   = (int16_t)(offsetdcl - adc_buffer.adc12.value.dcl);
  
  // Get Right motor currents
  curR_phaB = (int16_t)(offsetrrB - adc_buffer.adc12.value.rrB);
  curR_phaC = (int16_t)(offsetrrC - adc_buffer.adc12.value.rrC);
  curR_DC   = (int16_t)(offsetdcr - adc_buffer.adc12.value.dcr);

  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
#if defined(HOCP) //Disable motors as precaution if overcurrent or short detected through bkin input
  if (LEFT_TIM->SR & TIM_SR_BIF) {
    enable = 0;
  }
#endif

  if(ABS(curL_DC) > curDC_max || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

#if defined(HOCP)
  if (RIGHT_TIM->SR & TIM_SR_BIF) {
    enable = 0;
  }
#endif

  if(ABS(curR_DC)  > curDC_max || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  // Create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerPrev == 0) {
      buzzerPrev = 1;
      if (++buzzerIdx > (buzzerCount + 2)) {    // pause 2 periods
        buzzerIdx = 1;
      }
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) {
      #ifdef BEEPER_OFF
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);  // add line for LED
      #else
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);   // comment line for BEEPER off
      #endif
    }

  } else if (buzzerPrev) {
      #ifdef BEEPER_OFF
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);  // add line for LED
      #else
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); 
      #endif
      buzzerPrev = 0;
  }

  // Adjust pwm_margin depending on the selected Control Type
  if (rtP_Left.z_ctrlTypSel == FOC_CTRL) {
    pwm_margin = 110;
  } else {
    pwm_margin = 0;
  }

  // ############################### MOTOR CONTROL ###############################
  #ifndef INTBRK_L_EN
  int ul, vl, wl;
  #endif
  //int ur, vr, wr;
  static boolean_T OverrunFlag = false;

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  enableFin = enable && !rtY_Left.z_errCode && !rtY_Right.z_errCode &&
              !encoder_alignment_faulted() && !overcurrent_fault() && !DLVPA() &&
              !estop_active();
 
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    #ifndef ENCODER_Y
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);
    #else //Emulate static hall sensor position if using ENCODER to prevent driver from triggering hall error
    uint8_t hall_ul = 0;
    uint8_t hall_vl = 1;
    uint8_t hall_wl = 0;
    #endif
    /* Set motor inputs here */
    rtU_Left.b_motEna     = enableFin;
    rtU_Left.z_ctrlModReq = ctrlModReq;  
    rtU_Left.b_hallA      = hall_ul;
    rtU_Left.b_hallB      = hall_vl;
    rtU_Left.b_hallC      = hall_wl; 
    rtU_Left.i_phaAB      = curL_phaA;
    rtU_Left.i_phaBC      = curL_phaB;
    rtU_Left.i_DCLink     = curL_DC;
    
    #ifdef ENCODER_Y
    if (!encoder_y.align_state) {
      rtU_Left.r_inpTgt = pwml;
    } else {
      rtU_Left.r_inpTgt = encoder_y.align_inpTgt;
       rtU_Left.a_mechAngle = (encoder_y.emulated_mech_count * 23040) / (uint32_t)ENCODER_Y_CPR;
       
    }
    if (encoder_y.ali){
    encoder_y.aligned_count = encoder_y_handle.Instance->CNT;
    rtU_Left.a_mechAngle = (encoder_y.aligned_count * 23040) / (uint32_t)ENCODER_Y_CPR;
    // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`
    } 
    #else
    rtU_Left.r_inpTgt = pwml;
    #endif
    
    /* Step the controller */
    #ifdef MOTOR_LEFT_ENA    
    BLDC_controller_step(rtM_Left);
    #endif


    #ifndef INTBRK_L_EN
    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
  // errCodeLeft  = rtY_Left.z_errCode;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;

    /* Apply commands */
    
    LEFT_TIM->LEFT_TIM_U    = (uint16_t)CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_V    = (uint16_t)CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_W    = (uint16_t)CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    #else
    int16_t regenCur = curR_DC - MAX_REGEN_CURRENT;
    const int16_t brkOnThresh = BRKRESACT_SENS;
    const int16_t brkOffThresh = (BRKRESACT_SENS > 1) ? (BRKRESACT_SENS / 2) : 0;

    if (regenCur < 0) {
      regenCur = 0;
    }

    if (!brkResActiveL) {
      if (regenCur > brkOnThresh) {
        brkResActiveL = 1;
      }
    } else {
      if (regenCur < brkOffThresh) {
        brkResActiveL = 0;
      }
    }

    if (brkResActiveL) {
      I_BusR = CLAMP(((((int32_t)regenCur * BRAKE_RESISTANCE * pwm_res) /(50*batVoltageCalib))) ,0, (((uint32_t)pwm_res*90)/100));
      LEFT_TIM->LEFT_TIM_U = I_BusR;
    } else {
      LEFT_TIM->LEFT_TIM_U = 0;
    }
     
    #endif
  // =================================================================
  

  // ========================= RIGHT MOTOR ===========================  
    // Get hall sensors values
    #ifndef ENCODER_X 
    uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);
    #else  //Emulate static hall sensor position if using ENCODER to prevent driver from triggering hall error
    uint8_t hall_ur = 0;
    uint8_t hall_vr = 1;
    uint8_t hall_wr = 0;
    #endif

    /* Set motor inputs here */
    rtU_Right.b_motEna      = enableFin;
    rtU_Right.z_ctrlModReq  = ctrlModReq;
    rtU_Right.b_hallA       = hall_ur;
    rtU_Right.b_hallB       = hall_vr;
    rtU_Right.b_hallC       = hall_wr;
    rtU_Right.i_phaAB       = curR_phaB;
    rtU_Right.i_phaBC       = curR_phaC;
    rtU_Right.i_DCLink      = curR_DC;
  

     #ifdef ENCODER_X
    if (!encoder_x.align_state) {
      #ifdef HSPWM
      rtU_Right.r_inpTgt = HS_PWM;  //~-1ms
      #else
      rtU_Right.r_inpTgt = pwmr;
      #endif

    } else {
      rtU_Right.r_inpTgt = encoder_x.align_inpTgt;
       emulated_mech_angle_deg = (encoder_x.emulated_mech_count * 23040) / (uint32_t)ENCODER_X_CPR;
      rtU_Right.a_mechAngle = emulated_mech_angle_deg;
    }
    if (encoder_x.ali){
    encoder_x.aligned_count = encoder_x_handle.Instance->CNT;
    rtU_Right.a_mechAngle = (encoder_x.aligned_count * 23040) / (uint32_t)ENCODER_X_CPR;
    } 
    #else
  rtU_Right.r_inpTgt = pwmr;
    #endif
    
    /* Step the controller */
    #ifdef MOTOR_RIGHT_ENA
    BLDC_controller_step(rtM_Right);
    #endif

    /* Get motor outputs here */
    ur            = (uint16_t)CLAMP(rtY_Right.DC_phaA + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    vr            = (uint16_t)CLAMP(rtY_Right.DC_phaB + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    wr            = (uint16_t)CLAMP(rtY_Right.DC_phaC + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
 // errCodeRight  = rtY_Right.z_errCode;
 // motSpeedRight = rtY_Right.n_mot;
 // motAngleRight = rtY_Right.a_elecAngle;

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = ur;
    RIGHT_TIM->RIGHT_TIM_V  = vr;
    RIGHT_TIM->RIGHT_TIM_W  = wr;
    
  // =================================================================
   

  //External Break Resistor Output Control
  #ifdef EXTBRK_EN
  int16_t busRegenCur = (curR_DC + curL_DC) - MAX_REGEN_CURRENT; // Total bus regen current in x10 resolution
  const int16_t brkOnThreshExt = BRKRESACT_SENS;
  const int16_t brkOffThreshExt = (BRKRESACT_SENS > 1) ? (BRKRESACT_SENS / 2) : 0;

  if (busRegenCur < 0) {
    busRegenCur = 0;
  }

  if (!brkResActiveExt) {
    if (busRegenCur > brkOnThreshExt) {
      brkResActiveExt = 1;
    }
  } else {
    if (busRegenCur < brkOffThreshExt) {
      brkResActiveExt = 0;
    }
  }

  if (brkResActiveExt) {
    I_BusR = CLAMP(((((int32_t)busRegenCur * BRAKE_RESISTANCE * pwm_res) /(50*batVoltageCalib))) ,0, (((uint32_t)pwm_res*90)/100));
    EXT_PWM_BRK = I_BusR;
  } else {
    EXT_PWM_BRK = 0;
    I_BusR = 0;
  }
  #endif
  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}
