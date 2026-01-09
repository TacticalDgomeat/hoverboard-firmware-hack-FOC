/*
 * File: BLDC_controller.h
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 16.18
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Wed Jan  7 21:34:53 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef BLDC_controller_h_
#define BLDC_controller_h_
#ifndef BLDC_controller_COMMON_INCLUDES_
#define BLDC_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* BLDC_controller_COMMON_INCLUDES_ */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<S13>/Counter' */
typedef struct {
  int16_T UnitDelay_DSTATE;            /* '<S18>/UnitDelay' */
} DW_Counter;

/* Block signals and states (default storage) for system '<S50>/Low_Pass_Filter' */
typedef struct {
  int32_T UnitDelay1_DSTATE[2];        /* '<S56>/UnitDelay1' */
} DW_Low_Pass_Filter;

/* Block signals and states (default storage) for system '<S25>/Counter' */
typedef struct {
  uint16_T UnitDelay_DSTATE;           /* '<S30>/UnitDelay' */
} DW_Counter_d;

/* Block signals and states (default storage) for system '<S21>/either_edge' */
typedef struct {
  boolean_T UnitDelay_DSTATE;          /* '<S26>/UnitDelay' */
} DW_either_edge;

/* Block signals and states (default storage) for system '<S20>/Debounce_Filter' */
typedef struct {
  DW_either_edge either_edge_n;        /* '<S21>/either_edge' */
  DW_Counter_d Counter_n;              /* '<S24>/Counter' */
  DW_Counter_d Counter_e2;             /* '<S25>/Counter' */
  boolean_T UnitDelay_DSTATE;          /* '<S21>/UnitDelay' */
} DW_Debounce_Filter;

/* Block signals and states (default storage) for system '<S136>/I_backCalc_fixdt' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S141>/UnitDelay' */
  int32_T UnitDelay_DSTATE_a;          /* '<S143>/UnitDelay' */
} DW_I_backCalc_fixdt;

/* Block signals and states (default storage) for system '<S63>/PI_clamp_fixdt' */
typedef struct {
  int32_T ResettableDelay_DSTATE;      /* '<S130>/Resettable Delay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S127>/UnitDelay1' */
  boolean_T icLoad;                    /* '<S130>/Resettable Delay' */
} DW_PI_clamp_fixdt;

/* Block signals and states (default storage) for system '<S61>/PI_clamp_fixdt' */
typedef struct {
  int32_T ResettableDelay_DSTATE;      /* '<S67>/Resettable Delay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S65>/UnitDelay1' */
  boolean_T icLoad;                    /* '<S67>/Resettable Delay' */
} DW_PI_clamp_fixdt_e;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_PI_clamp_fixdt_e PI_clamp_fixdt_jc;/* '<S61>/PI_clamp_fixdt' */
  DW_PI_clamp_fixdt PI_clamp_fixdt_e;  /* '<S63>/PI_clamp_fixdt' */
  DW_I_backCalc_fixdt I_backCalc_fixdt_h;/* '<S135>/I_backCalc_fixdt' */
  DW_I_backCalc_fixdt I_backCalc_fixdt1;/* '<S136>/I_backCalc_fixdt1' */
  DW_I_backCalc_fixdt I_backCalc_fixdt_c;/* '<S136>/I_backCalc_fixdt' */
  DW_either_edge either_edge_j;        /* '<S20>/either_edge' */
  DW_Debounce_Filter Debounce_Filter_e;/* '<S20>/Debounce_Filter' */
  DW_Low_Pass_Filter Low_Pass_Filter_e;/* '<S50>/Low_Pass_Filter' */
  DW_Counter Counter_c;                /* '<S13>/Counter' */
  int32_T Divide1;                     /* '<S134>/Divide1' */
  int32_T UnitDelay_DSTATE;            /* '<S40>/UnitDelay' */
  int16_T Gain4_o[3];                  /* '<S57>/Gain4' */
  int16_T DataTypeConversion[2];       /* '<S56>/Data Type Conversion' */
  int16_T Merge;                       /* '<S59>/Merge' */
  int16_T Switch1;                     /* '<S131>/Switch1' */
  int16_T Gain3;                       /* '<S133>/Gain3' */
  int16_T OutportBufferForVd_max;
  int16_T Vq_max_M1;                   /* '<S133>/Vq_max_M1' */
  int16_T Gain5;                       /* '<S133>/Gain5' */
  int16_T Divide1_d;                   /* '<S133>/Divide1' */
  int16_T Gain1;                       /* '<S133>/Gain1' */
  int16_T Gain4;                       /* '<S133>/Gain4' */
  int16_T OutportBufferForid_max;
  int16_T Switch2_g;                   /* '<S140>/Switch2' */
  int16_T Switch2_p;                   /* '<S146>/Switch2' */
  int16_T Switch2_b;                   /* '<S144>/Switch2' */
  int16_T OutportBufferForr_fieldWeak;
  int16_T Divide3;                     /* '<S42>/Divide3' */
  int16_T Merge1;                      /* '<S33>/Merge1' */
  int16_T Abs1;                        /* '<S5>/Abs1' */
  int16_T OutportBufferForiqAbs;
  int16_T Abs5_d;                      /* '<S50>/Abs5' */
  int16_T OutportBufferForid_f;
  int16_T Divide11;                    /* '<S17>/Divide11' */
  int16_T r_sin_M1_1;                  /* '<S52>/r_sin_M1' */
  int16_T r_cos_M1_1;                  /* '<S52>/r_cos_M1' */
  int16_T z_counterRawPrev;            /* '<S17>/z_counterRawPrev' */
  int16_T UnitDelay3_DSTATE;           /* '<S13>/UnitDelay3' */
  int16_T UnitDelay4_DSTATE;           /* '<S17>/UnitDelay4' */
  int16_T UnitDelay2_DSTATE;           /* '<S17>/UnitDelay2' */
  int16_T UnitDelay3_DSTATE_k;         /* '<S17>/UnitDelay3' */
  int16_T UnitDelay5_DSTATE;           /* '<S17>/UnitDelay5' */
  int16_T UnitDelay4_DSTATE_f;         /* '<S13>/UnitDelay4' */
  int16_T UnitDelay4_DSTATE_a;         /* '<S8>/UnitDelay4' */
  int16_T Integrator_DSTATE;           /* '<S108>/Integrator' */
  int8_T Switch2_a;                    /* '<S12>/Switch2' */
  int8_T UnitDelay2_DSTATE_o;          /* '<S12>/UnitDelay2' */
  int8_T If1_ActiveSubsystem;          /* '<S7>/If1' */
  int8_T If2_ActiveSubsystem;          /* '<S7>/If2' */
  int8_T If1_ActiveSubsystem_f;        /* '<S47>/If1' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S59>/Switch Case' */
  int8_T If1_ActiveSubsystem_a;        /* '<S59>/If1' */
  int8_T If1_ActiveSubsystem_b;        /* '<S48>/If1' */
  int8_T SwitchCase_ActiveSubsystem_o; /* '<S133>/Switch Case' */
  int8_T If2_ActiveSubsystem_k;        /* '<S33>/If2' */
  int8_T If2_ActiveSubsystem_a;        /* '<S45>/If2' */
  uint8_T z_ctrlMod;                   /* '<S5>/F03_02_Control_Mode_Manager' */
  uint8_T Switch1_n;                   /* '<S20>/Switch1' */
  uint8_T UnitDelay3_DSTATE_f;         /* '<S10>/UnitDelay3' */
  uint8_T UnitDelay1_DSTATE;           /* '<S10>/UnitDelay1' */
  uint8_T UnitDelay2_DSTATE_k;         /* '<S10>/UnitDelay2' */
  uint8_T Integrator_IC_LOADING;       /* '<S108>/Integrator' */
  uint8_T is_active_c2_BLDC_controller;/* '<S5>/F03_02_Control_Mode_Manager' */
  uint8_T is_c2_BLDC_controller;       /* '<S5>/F03_02_Control_Mode_Manager' */
  uint8_T is_ACTIVE;                   /* '<S5>/F03_02_Control_Mode_Manager' */
  boolean_T Merge_a;                   /* '<S21>/Merge' */
  boolean_T dz_cntTrnsDet;             /* '<S17>/dz_cntTrnsDet' */
  boolean_T UnitDelay2_DSTATE_g;       /* '<S2>/UnitDelay2' */
  boolean_T UnitDelay5_DSTATE_l;       /* '<S2>/UnitDelay5' */
  boolean_T UnitDelay6_DSTATE;         /* '<S2>/UnitDelay6' */
  boolean_T UnitDelay_DSTATE_c;        /* '<S39>/UnitDelay' */
  boolean_T UnitDelay1_DSTATE_g;       /* '<S17>/UnitDelay1' */
  boolean_T n_commDeacv_Mode;          /* '<S13>/n_commDeacv' */
  boolean_T dz_cntTrnsDet_Mode;        /* '<S17>/dz_cntTrnsDet' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: r_sin_M1_1_Table
   * Referenced by: '<S52>/r_sin_M1'
   */
  int16_T r_sin_M1_1_Table[181];

  /* Computed Parameter: r_cos_M1_1_Table
   * Referenced by: '<S52>/r_cos_M1'
   */
  int16_T r_cos_M1_1_Table[181];

  /* Computed Parameter: r_sin3PhaA_M1_Table
   * Referenced by: '<S149>/r_sin3PhaA_M1'
   */
  int16_T r_sin3PhaA_M1_Table[181];

  /* Computed Parameter: r_sin3PhaB_M1_Table
   * Referenced by: '<S149>/r_sin3PhaB_M1'
   */
  int16_T r_sin3PhaB_M1_Table[181];

  /* Computed Parameter: r_sin3PhaC_M1_Table
   * Referenced by: '<S149>/r_sin3PhaC_M1'
   */
  int16_T r_sin3PhaC_M1_Table[181];

  /* Computed Parameter: iq_maxSca_M1_Table
   * Referenced by: '<S133>/iq_maxSca_M1'
   */
  uint16_T iq_maxSca_M1_Table[50];

  /* Computed Parameter: z_commutMap_M1_table
   * Referenced by: '<S147>/z_commutMap_M1'
   */
  int8_T z_commutMap_M1_table[18];

  /* Computed Parameter: vec_hallToPos_Value
   * Referenced by: '<S11>/vec_hallToPos'
   */
  int8_T vec_hallToPos_Value[8];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  boolean_T b_motEna;                  /* '<Root>/b_motEna' */
  uint8_T z_ctrlModReq;                /* '<Root>/z_ctrlModReq' */
  int16_T r_inpTgt;                    /* '<Root>/r_inpTgt' */
  uint8_T b_hallA;                     /* '<Root>/b_hallA ' */
  uint8_T b_hallB;                     /* '<Root>/b_hallB' */
  uint8_T b_hallC;                     /* '<Root>/b_hallC' */
  int16_T i_phaAB;                     /* '<Root>/i_phaAB' */
  int16_T i_phaBC;                     /* '<Root>/i_phaBC' */
  int16_T i_DCLink;                    /* '<Root>/i_DCLink' */
  int16_T a_mechAngle;                 /* '<Root>/a_mechAngle' */
  real32_T a_mechAnglef;               /* '<Root>/a_mechAnglef' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  int16_T DC_phaA;                     /* '<Root>/DC_phaA' */
  int16_T DC_phaB;                     /* '<Root>/DC_phaB' */
  int16_T DC_phaC;                     /* '<Root>/DC_phaC' */
  uint8_T z_errCode;                   /* '<Root>/z_errCode' */
  int16_T n_mot;                       /* '<Root>/n_mot' */
  int16_T a_elecAngle;                 /* '<Root>/a_elecAngle' */
  int16_T iq;                          /* '<Root>/iq' */
  int16_T id;                          /* '<Root>/id' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  int32_T dV_openRate;                 /* Variable: dV_openRate
                                        * Referenced by: '<S37>/dV_openRate'
                                        */
  real32_T cf_idKi;                    /* Variable: cf_idKi
                                        * Referenced by: '<S63>/cf_idKi1'
                                        */
  real32_T cf_idKp;                    /* Variable: cf_idKp
                                        * Referenced by: '<S63>/cf_idKp1'
                                        */
  real32_T cf_iqKi;                    /* Variable: cf_iqKi
                                        * Referenced by: '<S62>/cf_iqKi'
                                        */
  real32_T cf_iqKp;                    /* Variable: cf_iqKp
                                        * Referenced by: '<S62>/cf_iqKp'
                                        */
  int16_T dz_cntTrnsDetHi;             /* Variable: dz_cntTrnsDetHi
                                        * Referenced by: '<S17>/dz_cntTrnsDet'
                                        */
  int16_T dz_cntTrnsDetLo;             /* Variable: dz_cntTrnsDetLo
                                        * Referenced by: '<S17>/dz_cntTrnsDet'
                                        */
  int16_T n_cruiseMotTgt;              /* Variable: n_cruiseMotTgt
                                        * Referenced by: '<S61>/n_cruiseMotTgt'
                                        */
  int16_T z_maxCntRst;                 /* Variable: z_maxCntRst
                                        * Referenced by:
                                        *   '<S13>/Counter'
                                        *   '<S13>/z_maxCntRst'
                                        *   '<S13>/z_maxCntRst2'
                                        *   '<S13>/UnitDelay3'
                                        *   '<S17>/z_counter'
                                        */
  int16_T Vq_max_M1[79];               /* Variable: Vq_max_M1
                                        * Referenced by: '<S133>/Vq_max_M1'
                                        */
  int16_T Vq_max_XA[79];               /* Variable: Vq_max_XA
                                        * Referenced by: '<S133>/Vq_max_XA'
                                        */
  int16_T a_phaAdvMax;                 /* Variable: a_phaAdvMax
                                        * Referenced by: '<S42>/a_phaAdvMax'
                                        */
  int16_T i_max;                       /* Variable: i_max
                                        * Referenced by:
                                        *   '<S36>/i_max'
                                        *   '<S133>/i_max'
                                        */
  int16_T id_fieldWeakMax;             /* Variable: id_fieldWeakMax
                                        * Referenced by: '<S42>/id_fieldWeakMax'
                                        */
  int16_T n_commAcvLo;                 /* Variable: n_commAcvLo
                                        * Referenced by: '<S13>/n_commDeacv'
                                        */
  int16_T n_commDeacvHi;               /* Variable: n_commDeacvHi
                                        * Referenced by: '<S13>/n_commDeacv'
                                        */
  int16_T n_fieldWeakAuthHi;           /* Variable: n_fieldWeakAuthHi
                                        * Referenced by: '<S42>/n_fieldWeakAuthHi'
                                        */
  int16_T n_fieldWeakAuthLo;           /* Variable: n_fieldWeakAuthLo
                                        * Referenced by: '<S42>/n_fieldWeakAuthLo'
                                        */
  int16_T n_max;                       /* Variable: n_max
                                        * Referenced by:
                                        *   '<S36>/n_max'
                                        *   '<S133>/n_max1'
                                        */
  int16_T n_stdStillDet;               /* Variable: n_stdStillDet
                                        * Referenced by: '<S13>/n_stdStillDet'
                                        */
  int16_T r_errInpTgtThres;            /* Variable: r_errInpTgtThres
                                        * Referenced by: '<S20>/r_errInpTgtThres'
                                        */
  int16_T r_fieldWeakHi;               /* Variable: r_fieldWeakHi
                                        * Referenced by: '<S42>/r_fieldWeakHi'
                                        */
  int16_T r_fieldWeakLo;               /* Variable: r_fieldWeakLo
                                        * Referenced by: '<S42>/r_fieldWeakLo'
                                        */
  uint16_T cf_KbLimProt;               /* Variable: cf_KbLimProt
                                        * Referenced by:
                                        *   '<S135>/cf_KbLimProt'
                                        *   '<S136>/cf_KbLimProt'
                                        */
  uint16_T cf_nKp;                     /* Variable: cf_nKp
                                        * Referenced by: '<S61>/cf_nKp'
                                        */
  uint16_T cf_currFilt;                /* Variable: cf_currFilt
                                        * Referenced by: '<S50>/cf_currFilt'
                                        */
  uint16_T cf_iqKiLimProt;             /* Variable: cf_iqKiLimProt
                                        * Referenced by:
                                        *   '<S134>/cf_iqKiLimProt'
                                        *   '<S136>/cf_iqKiLimProt'
                                        */
  uint16_T cf_nKi;                     /* Variable: cf_nKi
                                        * Referenced by: '<S61>/cf_nKi'
                                        */
  uint16_T cf_nKiLimProt;              /* Variable: cf_nKiLimProt
                                        * Referenced by:
                                        *   '<S135>/cf_nKiLimProt'
                                        *   '<S136>/cf_nKiLimProt'
                                        */
  uint16_T cf_speedCoef;               /* Variable: cf_speedCoef
                                        * Referenced by: '<S17>/cf_speedCoef'
                                        */
  uint16_T t_errDequal;                /* Variable: t_errDequal
                                        * Referenced by: '<S20>/t_errDequal'
                                        */
  uint16_T t_errQual;                  /* Variable: t_errQual
                                        * Referenced by: '<S20>/t_errQual'
                                        */
  boolean_T b_angleMeasEna;            /* Variable: b_angleMeasEna
                                        * Referenced by:
                                        *   '<S3>/b_angleMeasEna'
                                        *   '<S13>/b_angleMeasEna'
                                        */
  boolean_T b_cruiseCtrlEna;           /* Variable: b_cruiseCtrlEna
                                        * Referenced by: '<S1>/b_cruiseCtrlEna'
                                        */
  boolean_T b_diagEna;                 /* Variable: b_diagEna
                                        * Referenced by: '<S4>/b_diagEna'
                                        */
  boolean_T b_fieldWeakEna;            /* Variable: b_fieldWeakEna
                                        * Referenced by:
                                        *   '<S6>/b_fieldWeakEna'
                                        *   '<S150>/b_fieldWeakEna'
                                        */
  uint8_T n_polePairs;                 /* Variable: n_polePairs
                                        * Referenced by: '<S15>/n_polePairs'
                                        */
  uint8_T z_ctrlTypSel;                /* Variable: z_ctrlTypSel
                                        * Referenced by: '<S1>/z_ctrlTypSel'
                                        */
  uint8_T z_selPhaCurMeasABC;          /* Variable: z_selPhaCurMeasABC
                                        * Referenced by: '<S49>/z_selPhaCurMeasABC'
                                        */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Real-time Model Data Structure */
struct tag_RTM {
  P *defaultParam;
  ExtU *inputs;
  ExtY *outputs;
  DW *dwork;
};

/* Block parameters (default storage) */
//extern P rtP;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void BLDC_controller_initialize(RT_MODEL *const rtM);
extern void BLDC_controller_step(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S13>/Scope2' : Unused code path elimination
 * Block '<S14>/Scope' : Unused code path elimination
 * Block '<S15>/Scope1' : Unused code path elimination
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Propagation' : Unused code path elimination
 * Block '<S43>/Data Type Duplicate' : Unused code path elimination
 * Block '<S43>/Data Type Propagation' : Unused code path elimination
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
 * Block '<S116>/Data Type Duplicate' : Unused code path elimination
 * Block '<S116>/Data Type Propagation' : Unused code path elimination
 * Block '<S70>/Data Type Duplicate' : Unused code path elimination
 * Block '<S70>/Data Type Propagation' : Unused code path elimination
 * Block '<S62>/Scope' : Unused code path elimination
 * Block '<S128>/Data Type Duplicate' : Unused code path elimination
 * Block '<S128>/Data Type Propagation' : Unused code path elimination
 * Block '<S132>/Data Type Duplicate' : Unused code path elimination
 * Block '<S132>/Data Type Propagation' : Unused code path elimination
 * Block '<S47>/Scope' : Unused code path elimination
 * Block '<S133>/Scope' : Unused code path elimination
 * Block '<S137>/Data Type Duplicate' : Unused code path elimination
 * Block '<S137>/Data Type Propagation' : Unused code path elimination
 * Block '<S140>/Data Type Duplicate' : Unused code path elimination
 * Block '<S140>/Data Type Propagation' : Unused code path elimination
 * Block '<S144>/Data Type Duplicate' : Unused code path elimination
 * Block '<S144>/Data Type Propagation' : Unused code path elimination
 * Block '<S146>/Data Type Duplicate' : Unused code path elimination
 * Block '<S146>/Data Type Propagation' : Unused code path elimination
 * Block '<S7>/Scope12' : Unused code path elimination
 * Block '<S7>/Scope8' : Unused code path elimination
 * Block '<S7>/toNegative' : Unused code path elimination
 * Block '<S150>/Scope' : Unused code path elimination
 * Block '<S2>/Data Type Conversion' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('BLDCmotor_FOC_R2017b_fixdt/BLDC_controller')    - opens subsystem BLDCmotor_FOC_R2017b_fixdt/BLDC_controller
 * hilite_system('BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotor_FOC_R2017b_fixdt'
 * '<S1>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller'
 * '<S2>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/Call_Scheduler'
 * '<S3>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations'
 * '<S4>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics'
 * '<S5>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager'
 * '<S6>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening'
 * '<S7>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control'
 * '<S8>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management'
 * '<S9>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/Task_Scheduler'
 * '<S10>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_01_Edge_Detector'
 * '<S11>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_02_Position_Calculation'
 * '<S12>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_03_Direction_Detection'
 * '<S13>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation'
 * '<S14>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Electrical_Angle_Estimation'
 * '<S15>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_06_Electrical_Angle_Measurement'
 * '<S16>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Counter'
 * '<S17>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Raw_Motor_Speed_Estimation'
 * '<S18>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Counter/rst_Delay'
 * '<S19>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_06_Electrical_Angle_Measurement/Modulo_fixdt'
 * '<S20>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled'
 * '<S21>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter'
 * '<S22>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/either_edge'
 * '<S23>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Default'
 * '<S24>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Dequalification'
 * '<S25>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Qualification'
 * '<S26>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/either_edge'
 * '<S27>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Dequalification/Counter'
 * '<S28>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Dequalification/Counter/rst_Delay'
 * '<S29>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Qualification/Counter'
 * '<S30>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Diagnostics_Enabled/Debounce_Filter/Qualification/Counter/rst_Delay'
 * '<S31>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_01_Mode_Transition_Calculation'
 * '<S32>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_02_Control_Mode_Manager'
 * '<S33>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis'
 * '<S34>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Default_Control_Type'
 * '<S35>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Default_Mode'
 * '<S36>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/FOC_Control_Type'
 * '<S37>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode'
 * '<S38>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter'
 * '<S39>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/rising_edge_init'
 * '<S40>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter/Delay_Init1'
 * '<S41>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter/Saturation Dynamic'
 * '<S42>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening/Field_Weakening_Enabled'
 * '<S43>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening/Field_Weakening_Enabled/Saturation Dynamic'
 * '<S44>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening/Field_Weakening_Enabled/Saturation Dynamic1'
 * '<S45>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward'
 * '<S46>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Inverse'
 * '<S47>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC'
 * '<S48>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations'
 * '<S49>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Clarke_Transform'
 * '<S50>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Current_Filtering'
 * '<S51>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Park_Transform'
 * '<S52>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Sine_Cosine_Approximation'
 * '<S53>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Clarke_Transform/Clarke_PhasesAB'
 * '<S54>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Clarke_Transform/Clarke_PhasesAC'
 * '<S55>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Clarke_Transform/Clarke_PhasesBC'
 * '<S56>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Forward/Current_Filtering/Low_Pass_Filter'
 * '<S57>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Inverse/Inv_Clarke_Transform'
 * '<S58>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Park_Transform_Inverse/Inv_Park_Transform'
 * '<S59>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled'
 * '<S60>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Open_Mode'
 * '<S61>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Speed_Mode'
 * '<S62>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode'
 * '<S63>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Vd_Calculation'
 * '<S64>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Voltage_Mode'
 * '<S65>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Speed_Mode/PI_clamp_fixdt'
 * '<S66>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Speed_Mode/PI_clamp_fixdt/Clamping_circuit'
 * '<S67>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Speed_Mode/PI_clamp_fixdt/Integrator'
 * '<S68>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Speed_Mode/PI_clamp_fixdt/Saturation_hit'
 * '<S69>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller'
 * '<S70>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/Saturation Dynamic1'
 * '<S71>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Anti-windup'
 * '<S72>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/D Gain'
 * '<S73>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/External Derivative'
 * '<S74>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Filter'
 * '<S75>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Filter ICs'
 * '<S76>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/I Gain'
 * '<S77>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Ideal P Gain'
 * '<S78>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Ideal P Gain Fdbk'
 * '<S79>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Integrator'
 * '<S80>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Integrator ICs'
 * '<S81>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/N Copy'
 * '<S82>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/N Gain'
 * '<S83>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/P Copy'
 * '<S84>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Parallel P Gain'
 * '<S85>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Reset Signal'
 * '<S86>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Saturation'
 * '<S87>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Saturation Fdbk'
 * '<S88>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Sum'
 * '<S89>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Sum Fdbk'
 * '<S90>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tracking Mode'
 * '<S91>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tracking Mode Sum'
 * '<S92>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tsamp - Integral'
 * '<S93>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tsamp - Ngain'
 * '<S94>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/postSat Signal'
 * '<S95>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/preInt Signal'
 * '<S96>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/preSat Signal'
 * '<S97>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S98>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S99>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S100>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S101>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/D Gain/Disabled'
 * '<S102>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/External Derivative/Disabled'
 * '<S103>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Filter/Disabled'
 * '<S104>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Filter ICs/Disabled'
 * '<S105>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/I Gain/External Parameters'
 * '<S106>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Ideal P Gain/Passthrough'
 * '<S107>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Ideal P Gain Fdbk/Disabled'
 * '<S108>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Integrator/Discrete'
 * '<S109>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Integrator ICs/External IC'
 * '<S110>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/N Copy/Disabled wSignal Specification'
 * '<S111>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/N Gain/Disabled'
 * '<S112>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/P Copy/Disabled'
 * '<S113>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Parallel P Gain/External Parameters'
 * '<S114>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Reset Signal/Disabled'
 * '<S115>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Saturation/External'
 * '<S116>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Saturation/External/Saturation Dynamic'
 * '<S117>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Saturation Fdbk/Disabled'
 * '<S118>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Sum/Sum_PI'
 * '<S119>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Sum Fdbk/Disabled'
 * '<S120>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tracking Mode/Disabled'
 * '<S121>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tracking Mode Sum/Passthrough'
 * '<S122>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tsamp - Integral/Passthrough'
 * '<S123>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/Tsamp - Ngain/Passthrough'
 * '<S124>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/postSat Signal/Forward_Path'
 * '<S125>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/preInt Signal/Internal PreInt'
 * '<S126>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Torque_Mode/PI Controller/preSat Signal/Forward_Path'
 * '<S127>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Vd_Calculation/PI_clamp_fixdt'
 * '<S128>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Vd_Calculation/Saturation Dynamic'
 * '<S129>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Vd_Calculation/PI_clamp_fixdt/Clamping_circuit'
 * '<S130>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Vd_Calculation/PI_clamp_fixdt/Integrator'
 * '<S131>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Vd_Calculation/PI_clamp_fixdt/Saturation_hit'
 * '<S132>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/FOC_Enabled/Voltage_Mode/Saturation Dynamic1'
 * '<S133>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled'
 * '<S134>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Speed_Mode_Protection'
 * '<S135>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Torque_Mode_Protection'
 * '<S136>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection'
 * '<S137>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Speed_Mode_Protection/Saturation Dynamic'
 * '<S138>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Torque_Mode_Protection/I_backCalc_fixdt'
 * '<S139>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Torque_Mode_Protection/I_backCalc_fixdt/Integrator'
 * '<S140>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Torque_Mode_Protection/I_backCalc_fixdt/Saturation Dynamic1'
 * '<S141>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection/I_backCalc_fixdt'
 * '<S142>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection/I_backCalc_fixdt1'
 * '<S143>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection/I_backCalc_fixdt/Integrator'
 * '<S144>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection/I_backCalc_fixdt/Saturation Dynamic1'
 * '<S145>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection/I_backCalc_fixdt1/Integrator'
 * '<S146>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Motor_Limitations_Enabled/Voltage_Mode_Protection/I_backCalc_fixdt1/Saturation Dynamic1'
 * '<S147>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/COM_Method'
 * '<S148>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/FOC_Method'
 * '<S149>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/SIN_Method'
 * '<S150>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/SIN_Method/Final_Phase_Advance_Calculation'
 * '<S151>' : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/SIN_Method/Final_Phase_Advance_Calculation/Modulo_fixdt'
 */
#endif                                 /* BLDC_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
