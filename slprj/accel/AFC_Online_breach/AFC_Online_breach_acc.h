#include "__cf_AFC_Online_breach.h"
#ifndef RTW_HEADER_AFC_Online_breach_acc_h_
#define RTW_HEADER_AFC_Online_breach_acc_h_
#include <stddef.h>
#include <float.h>
#ifndef AFC_Online_breach_acc_COMMON_INCLUDES_
#define AFC_Online_breach_acc_COMMON_INCLUDES_
#include <stdlib.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#endif
#include "AFC_Online_breach_acc_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rtGetInf.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
typedef struct { real_T B_16_4_0 [ 2 ] ; real_T B_16_9_0 [ 2 ] ; real_T
B_16_10_0 ; real_T B_16_13_0 ; real_T B_16_17_0 ; real_T B_16_20_0 ; real_T
B_15_0_0 ; real_T B_15_1_0 ; real_T B_15_2_0 ; real_T B_15_4_0 ; real_T
B_15_7_0 ; real_T B_15_8_0 ; real_T B_15_9_0 ; real_T B_15_10_0 ; real_T
B_15_12_0 ; real_T B_15_13_0 ; real_T B_15_14_0 ; real_T B_15_15_0 ; real_T
B_15_16_0 ; real_T B_15_18_0 ; real_T B_15_19_0 ; real_T B_15_20_0 ; real_T
B_15_21_0 ; real_T B_15_23_0 ; real_T B_15_24_0 ; real_T B_15_25_0 ; real_T
B_15_27_0 ; real_T B_15_32_0 ; real_T B_15_33_0 ; real_T B_15_34_0 ; real_T
B_15_35_0 ; real_T B_15_41_0 ; real_T B_15_42_0 ; real_T B_15_43_0 ; real_T
B_15_45_0 ; real_T B_15_46_0 ; real_T B_15_47_0 ; real_T B_15_49_0 ; real_T
B_15_50_0 ; real_T B_15_51_0 ; real_T B_15_52_0 ; real_T B_15_53_0 ; real_T
B_15_54_0 ; real_T B_15_55_0 ; real_T B_15_56_0 ; real_T B_15_57_0 ; real_T
B_15_58_0 ; real_T B_15_59_0 ; real_T B_15_60_0 ; real_T B_15_61_0 ; real_T
B_15_62_0 ; real_T B_15_63_0 ; real_T B_15_64_0 ; real_T B_15_65_0 ; real_T
B_15_66_0 ; real_T B_14_0_0 ; real_T B_13_1_0 ; real32_T B_9_0_0 ; real32_T
B_8_0_0 ; real32_T B_8_3_0 ; real32_T B_7_2_0 ; real32_T B_4_5_0 ; real32_T
B_4_7_0 ; real32_T B_2_6_0 ; real32_T B_2_7_0 ; real32_T B_1_13_0 ; boolean_T
B_9_3_0 ; boolean_T B_8_5_0 ; boolean_T B_7_6_0 ; boolean_T B_4_10_0 ; }
B_AFC_Online_breach_T ; typedef struct { real_T STLMonitor_DSTATE [ 2 ] ;
real_T STLMonitor_DSTATE_c [ 2 ] ; real_T nextTime ; int64_T
numCompleteCycles ; struct { real_T modelTStart ; }
fuelsystemtransportdelay_RWORK ; void * STLMonitor_PWORK ; struct { void *
AQHandles ; void * SlioLTF ; }
HiddenToAsyncQueue_InsertedFor_EngineSpeed_at_outport_0_PWORK ; struct { void
* AQHandles ; void * SlioLTF ; }
HiddenToAsyncQueue_InsertedFor_PedalAngle_at_outport_0_PWORK ; void *
STLMonitor_PWORK_p ; struct { void * AQHandles ; void * SlioLTF ; }
HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_0_PWORK ; struct {
void * AQHandles ; void * SlioLTF ; }
HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_1_PWORK ; struct {
void * AQHandles ; void * SlioLTF ; }
HiddenToAsyncQueue_InsertedFor_Throttle_at_outport_0_PWORK ; struct { void *
AQHandles ; void * SlioLTF ; }
HiddenToAsyncQueue_InsertedFor_Wallwetting_at_outport_0_PWORK ; struct { void
* TUbufferPtrs [ 3 ] ; } fuelsystemtransportdelay_PWORK ; void *
commanded_fuel_PWORK ; void * mode_fb_PWORK ; void * mode_fb1_PWORK ; void *
DataStoreMemory_PWORK ; void * DataStoreMemory1_PWORK ; void *
DataStoreMemory2_PWORK ; void * DataStoreMemory3_PWORK ; real32_T
UnitDelay2_DSTATE ; real32_T UnitDelay1_DSTATE ; real32_T UnitDelay1_DSTATE_d
; real32_T commanded_fuel ; real32_T airbyfuel_ref ; real32_T engine_speed ;
real32_T throttle_flow ; real32_T airbyfuel_meas ; real32_T throttle_angle ;
int32_T AirFuelControlModel_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun ; int32_T
AF_Controller_sysIdxToRun ; int32_T justEnabled ; int32_T currentValue ;
int32_T dsmIdx ; int32_T dsmIdx_l ; int32_T dsmIdx_e ; int32_T
fuel_controller_sysIdxToRun ; int32_T dsmIdx_f ; int32_T dsmIdx_h ; int32_T
dsmIdx_ha ; int32_T dsmIdx_m ; int32_T fuel_controller_pwon_sysIdxToRun ;
int32_T fuel_controller_mode_10ms_sysIdxToRun ; int32_T
sensor_failure_detection_sysIdxToRun ; int32_T
power_mode_detection_sysIdxToRun ; int32_T normal_mode_detection_sysIdxToRun
; int32_T TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_p ; int32_T
TmpAtomicSubsysAtSwitchInport3_sysIdxToRun ; int32_T
fuel_controller_10ms_sysIdxToRun ; int32_T feedforward_controller_sysIdxToRun
; int32_T feedback_PI_controller_sysIdxToRun ; int32_T
air_estimation_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitchInport3_sysIdxToRun_m ; struct { int_T Tail ; int_T
Head ; int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
fuelsystemtransportdelay_IWORK ; int_T EngineSpeed9001100_MODE ; int_T
theta090_MODE ; int_T MinMax_MODE ; int_T flowdirection_MODE ; int_T
AFSensorFaultInjection_MODE ; int_T Pwon_MODE ; boolean_T UnitDelay_DSTATE ;
boolean_T UnitDelay1_DSTATE_a ; boolean_T UnitDelay1_DSTATE_e ; int8_T
fuel_controller_pwon_SubsysRanBC ; int8_T
fuel_controller_mode_10ms_SubsysRanBC ; int8_T
fuel_controller_10ms_SubsysRanBC ; int8_T feedback_PI_controller_SubsysRanBC
; boolean_T Switch_Mode ; boolean_T controller_mode ; char_T
pad_controller_mode [ 3 ] ; } DW_AFC_Online_breach_T ; typedef struct {
real_T p00543bar_CSTATE ; real_T Throttledelay_CSTATE ; real_T
Integrator_CSTATE ; real_T Integrator_CSTATE_c ; real_T Integrator_CSTATE_h ;
real_T fuelsystemtransportdelay_CSTATE ; } X_AFC_Online_breach_T ; typedef
struct { real_T p00543bar_CSTATE ; real_T Throttledelay_CSTATE ; real_T
Integrator_CSTATE ; real_T Integrator_CSTATE_c ; real_T Integrator_CSTATE_h ;
real_T fuelsystemtransportdelay_CSTATE ; } XDot_AFC_Online_breach_T ; typedef
struct { boolean_T p00543bar_CSTATE ; boolean_T Throttledelay_CSTATE ;
boolean_T Integrator_CSTATE ; boolean_T Integrator_CSTATE_c ; boolean_T
Integrator_CSTATE_h ; boolean_T fuelsystemtransportdelay_CSTATE ; }
XDis_AFC_Online_breach_T ; typedef struct { real_T p00543bar_CSTATE ; real_T
Throttledelay_CSTATE ; real_T Integrator_CSTATE ; real_T Integrator_CSTATE_c
; real_T Integrator_CSTATE_h ; real_T fuelsystemtransportdelay_CSTATE ; }
CStateAbsTol_AFC_Online_breach_T ; typedef struct { real_T
EngineSpeed9001100_UprLim_ZC ; real_T EngineSpeed9001100_LwrLim_ZC ; real_T
theta090_UprLim_ZC ; real_T theta090_LwrLim_ZC ; real_T MinMax_MinmaxInput_ZC
; real_T Switch_SwitchCond_ZC ; real_T flowdirection_Input_ZC ; real_T
AFSensorFaultInjection_StepTime_ZC ; real_T Pwon_StepTime_ZC ; real_T
fuel_controller_pwon_Trig_ZC ; real_T fuel_controller_mode_10ms_Trig_ZC ;
real_T fuel_controller_10ms_Trig_ZC ; } ZCV_AFC_Online_breach_T ; typedef
struct { ZCSigState EngineSpeed9001100_UprLim_ZCE ; ZCSigState
EngineSpeed9001100_LwrLim_ZCE ; ZCSigState theta090_UprLim_ZCE ; ZCSigState
theta090_LwrLim_ZCE ; ZCSigState MinMax_MinmaxInput_ZCE ; ZCSigState
Switch_SwitchCond_ZCE ; ZCSigState flowdirection_Input_ZCE ; ZCSigState
AFSensorFaultInjection_StepTime_ZCE ; ZCSigState Pwon_StepTime_ZCE ;
ZCSigState fuel_controller_pwon_Trig_ZCE ; ZCSigState
fuel_controller_mode_10ms_Trig_ZCE ; ZCSigState fuel_controller_10ms_Trig_ZCE
; } PrevZCX_AFC_Online_breach_T ; typedef struct { real_T Pedal_Angle ;
real_T Engine_Speed ; } ExternalUPtrs_AFC_Online_breach_T ; typedef struct {
real_T * B_16_1 ; real_T * B_16_2 ; real_T * B_16_3 ; real_T * B_16_5 ;
real_T * B_16_6 ; } ExtY_AFC_Online_breach_T ; struct P_AFC_Online_breach_T_
{ real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T P_3 ; real_T P_4 ; real_T P_5
; real_T P_6 ; real_T P_7 ; real_T P_8 ; real_T P_9 ; real_T P_10 ; real_T
P_11 ; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ;
real_T P_17 ; real_T P_18 ; real_T P_19 ; real_T P_20 ; real_T P_21 [ 20 ] ;
real_T P_22 [ 5 ] ; real_T P_23 [ 4 ] ; real_T P_24 ; real_T P_25 ; real_T
P_26 ; real_T P_27 ; real_T P_28 ; real_T P_29 ; real_T P_30 ; real_T P_31 ;
real_T P_32 ; real_T P_33 ; real_T P_34 ; real_T P_35 ; real_T P_36 [ 20 ] ;
real_T P_37 [ 5 ] ; real_T P_38 [ 4 ] ; real_T P_39 ; real_T P_40 ; real_T
P_41 ; real_T P_42 ; real_T P_43 ; real_T P_44 ; real_T P_45 ; real_T P_46 [
20 ] ; real_T P_47 [ 5 ] ; real_T P_48 [ 4 ] ; real_T P_49 ; real_T P_50 ;
real_T P_51 ; real_T P_52 [ 2 ] ; real_T P_53 [ 8 ] ; real_T P_54 [ 2 ] ;
real_T P_55 [ 87 ] ; real_T P_56 [ 2 ] ; real_T P_57 ; real_T P_58 [ 2 ] ;
real_T P_59 ; real_T P_60 [ 2 ] ; real_T P_61 [ 8 ] ; real_T P_62 [ 2 ] ;
real_T P_63 [ 87 ] ; real_T P_64 [ 2 ] ; real_T P_65 ; real_T P_66 [ 2 ] ;
real_T P_67 ; real_T P_68 ; real_T P_69 ; real_T P_70 ; real_T P_71 ;
real32_T P_72 ; real32_T P_73 ; real32_T P_74 ; real32_T P_75 ; real32_T P_76
; real32_T P_77 ; real32_T P_78 ; real32_T P_79 ; real32_T P_80 ; real32_T
P_81 ; real32_T P_82 ; real32_T P_83 ; real32_T P_84 ; real32_T P_85 ;
real32_T P_86 ; real32_T P_87 ; real32_T P_88 ; real32_T P_89 ; real32_T P_90
; real32_T P_91 ; real32_T P_92 ; real32_T P_93 ; real32_T P_94 ; real32_T
P_95 ; real32_T P_96 ; real32_T P_97 ; real32_T P_98 ; real32_T P_99 ;
real32_T P_100 ; real32_T P_101 ; real32_T P_102 ; real32_T P_103 ; real32_T
P_104 ; real32_T P_105 ; real32_T P_106 ; uint32_T P_107 [ 2 ] ; uint32_T
P_108 [ 2 ] ; uint32_T P_109 [ 2 ] ; boolean_T P_110 ; boolean_T P_111 ;
boolean_T P_112 ; boolean_T P_113 ; } ; extern P_AFC_Online_breach_T
AFC_Online_breach_rtDefaultP ;
#endif
