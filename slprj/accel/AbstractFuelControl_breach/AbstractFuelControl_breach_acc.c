#include "__cf_AbstractFuelControl_breach.h"
#if 0
This file is not available for use in any application other than as a MATLAB
( R ) MEX file for use with the Simulink ( R ) product . If you do not have
the Simulink Coder licensed , this file contains encrypted block names , etc
. If you purchase the Simulink Coder , this file will contain full block
descriptions and improved variable names .
#endif
#include <math.h>
#include "AbstractFuelControl_breach_acc.h"
#include "AbstractFuelControl_breach_acc_private.h"
#include <stdio.h>
#include "slexec_vm_simstruct_bridge.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_lookup_functions.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
#include "simtarget/slAccSfcnBridge.h"
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T AbstractFuelControl_breach_acc_rt_TDelayUpdateTailOrGrowBuf ( int_T
* bufSzPtr , int_T * tailPtr , int_T * headPtr , int_T * lastPtr , real_T
tMinusDelay , real_T * * tBufPtr , real_T * * uBufPtr , real_T * * xBufPtr ,
boolean_T isfixedbuf , boolean_T istransportdelay , int_T * maxNewBufSzPtr )
{ int_T testIdx ; int_T tail = * tailPtr ; int_T bufSz = * bufSzPtr ; real_T
* tBuf = * tBufPtr ; real_T * xBuf = ( NULL ) ; int_T numBuffer = 2 ; if (
istransportdelay ) { numBuffer = 3 ; xBuf = * xBufPtr ; } testIdx = ( tail <
( bufSz - 1 ) ) ? ( tail + 1 ) : 0 ; if ( ( tMinusDelay <= tBuf [ testIdx ] )
&& ! isfixedbuf ) { int_T j ; real_T * tempT ; real_T * tempU ; real_T *
tempX = ( NULL ) ; real_T * uBuf = * uBufPtr ; int_T newBufSz = bufSz + 1024
; if ( newBufSz > * maxNewBufSzPtr ) { * maxNewBufSzPtr = newBufSz ; } tempU
= ( real_T * ) utMalloc ( numBuffer * newBufSz * sizeof ( real_T ) ) ; if (
tempU == ( NULL ) ) { return ( false ) ; } tempT = tempU + newBufSz ; if (
istransportdelay ) tempX = tempT + newBufSz ; for ( j = tail ; j < bufSz ; j
++ ) { tempT [ j - tail ] = tBuf [ j ] ; tempU [ j - tail ] = uBuf [ j ] ; if
( istransportdelay ) tempX [ j - tail ] = xBuf [ j ] ; } for ( j = 0 ; j <
tail ; j ++ ) { tempT [ j + bufSz - tail ] = tBuf [ j ] ; tempU [ j + bufSz -
tail ] = uBuf [ j ] ; if ( istransportdelay ) tempX [ j + bufSz - tail ] =
xBuf [ j ] ; } if ( * lastPtr > tail ) { * lastPtr -= tail ; } else { *
lastPtr += ( bufSz - tail ) ; } * tailPtr = 0 ; * headPtr = bufSz ; utFree (
uBuf ) ; * bufSzPtr = newBufSz ; * tBufPtr = tempT ; * uBufPtr = tempU ; if (
istransportdelay ) * xBufPtr = tempX ; } else { * tailPtr = testIdx ; }
return ( true ) ; } real_T
AbstractFuelControl_breach_acc_rt_VTDelayfindtDInterpolate ( real_T x ,
real_T * tBuf , real_T * uBuf , real_T * xBuf , int_T bufSz , int_T head ,
int_T tail , int_T * pLast , real_T t , real_T tStart , boolean_T discrete ,
boolean_T minorStepAndTAtLastMajorOutput , real_T initOutput , real_T *
appliedDelay ) { int_T n , k ; real_T f ; int_T kp1 ; real_T tminustD , tL ,
tR , uD , uL , uR , fU ; if ( minorStepAndTAtLastMajorOutput ) { if ( * pLast
== head ) { * pLast = ( * pLast == 0 ) ? bufSz - 1 : * pLast - 1 ; } head = (
head == 0 ) ? bufSz - 1 : head - 1 ; } if ( x <= 1 ) { return initOutput ; }
k = * pLast ; n = 0 ; for ( ; ; ) { n ++ ; if ( n > bufSz ) break ; if ( x -
xBuf [ k ] > 1.0 ) { if ( k == head ) { int_T km1 ; f = ( x - 1.0 - xBuf [ k
] ) / ( x - xBuf [ k ] ) ; tminustD = ( 1.0 - f ) * tBuf [ k ] + f * t ; km1
= k - 1 ; if ( km1 < 0 ) km1 = bufSz - 1 ; tL = tBuf [ km1 ] ; tR = tBuf [ k
] ; uL = uBuf [ km1 ] ; uR = uBuf [ k ] ; break ; } kp1 = k + 1 ; if ( kp1 ==
bufSz ) kp1 = 0 ; if ( x - xBuf [ kp1 ] <= 1.0 ) { f = ( x - 1.0 - xBuf [ k ]
) / ( xBuf [ kp1 ] - xBuf [ k ] ) ; tL = tBuf [ k ] ; tR = tBuf [ kp1 ] ; uL
= uBuf [ k ] ; uR = uBuf [ kp1 ] ; tminustD = ( 1.0 - f ) * tL + f * tR ;
break ; } k = kp1 ; } else { if ( k == tail ) { f = ( x - 1.0 ) / xBuf [ k ]
; if ( discrete ) { return ( uBuf [ tail ] ) ; } kp1 = k + 1 ; if ( kp1 ==
bufSz ) kp1 = 0 ; tminustD = ( 1 - f ) * tStart + f * tBuf [ k ] ; tL = tBuf
[ k ] ; tR = tBuf [ kp1 ] ; uL = uBuf [ k ] ; uR = uBuf [ kp1 ] ; break ; } k
= k - 1 ; if ( k < 0 ) k = bufSz - 1 ; } } * pLast = k ; if ( tR == tL ) { fU
= 1.0 ; } else { fU = ( tminustD - tL ) / ( tR - tL ) ; } if ( discrete ) {
uD = ( fU > ( 1.0 - fU ) ) ? uR : uL ; } else { uD = ( 1.0 - fU ) * uL + fU *
uR ; } * appliedDelay = t - tminustD ; return uD ; } real_T look2_binlxpw (
real_T u0 , real_T u1 , const real_T bp0 [ ] , const real_T bp1 [ ] , const
real_T table [ ] , const uint32_T maxIndex [ ] , uint32_T stride ) { real_T
frac ; uint32_T bpIndices [ 2 ] ; real_T fractions [ 2 ] ; real_T yL_1d ;
uint32_T iRght ; uint32_T bpIdx ; uint32_T iLeft ; if ( u0 <= bp0 [ 0U ] ) {
iLeft = 0U ; frac = ( u0 - bp0 [ 0U ] ) / ( bp0 [ 1U ] - bp0 [ 0U ] ) ; }
else if ( u0 < bp0 [ maxIndex [ 0U ] ] ) { bpIdx = maxIndex [ 0U ] >> 1U ;
iLeft = 0U ; iRght = maxIndex [ 0U ] ; while ( iRght - iLeft > 1U ) { if ( u0
< bp0 [ bpIdx ] ) { iRght = bpIdx ; } else { iLeft = bpIdx ; } bpIdx = (
iRght + iLeft ) >> 1U ; } frac = ( u0 - bp0 [ iLeft ] ) / ( bp0 [ iLeft + 1U
] - bp0 [ iLeft ] ) ; } else { iLeft = maxIndex [ 0U ] - 1U ; frac = ( u0 -
bp0 [ maxIndex [ 0U ] - 1U ] ) / ( bp0 [ maxIndex [ 0U ] ] - bp0 [ maxIndex [
0U ] - 1U ] ) ; } fractions [ 0U ] = frac ; bpIndices [ 0U ] = iLeft ; if (
u1 <= bp1 [ 0U ] ) { iLeft = 0U ; frac = ( u1 - bp1 [ 0U ] ) / ( bp1 [ 1U ] -
bp1 [ 0U ] ) ; } else if ( u1 < bp1 [ maxIndex [ 1U ] ] ) { bpIdx = maxIndex
[ 1U ] >> 1U ; iLeft = 0U ; iRght = maxIndex [ 1U ] ; while ( iRght - iLeft >
1U ) { if ( u1 < bp1 [ bpIdx ] ) { iRght = bpIdx ; } else { iLeft = bpIdx ; }
bpIdx = ( iRght + iLeft ) >> 1U ; } frac = ( u1 - bp1 [ iLeft ] ) / ( bp1 [
iLeft + 1U ] - bp1 [ iLeft ] ) ; } else { iLeft = maxIndex [ 1U ] - 1U ; frac
= ( u1 - bp1 [ maxIndex [ 1U ] - 1U ] ) / ( bp1 [ maxIndex [ 1U ] ] - bp1 [
maxIndex [ 1U ] - 1U ] ) ; } bpIdx = iLeft * stride + bpIndices [ 0U ] ;
yL_1d = ( table [ bpIdx + 1U ] - table [ bpIdx ] ) * fractions [ 0U ] + table
[ bpIdx ] ; bpIdx += stride ; return ( ( ( table [ bpIdx + 1U ] - table [
bpIdx ] ) * fractions [ 0U ] + table [ bpIdx ] ) - yL_1d ) * frac + yL_1d ; }
void rt_ssGetBlockPath ( SimStruct * S , int_T sysIdx , int_T blkIdx , char_T
* * path ) { _ssGetBlockPath ( S , sysIdx , blkIdx , path ) ; } void
rt_ssSet_slErrMsg ( SimStruct * S , void * diag ) { if ( !
_ssIsErrorStatusAslErrMsg ( S ) ) { _ssSet_slErrMsg ( S , diag ) ; } else {
_ssDiscardDiagnostic ( S , diag ) ; } } void rt_ssReportDiagnosticAsWarning (
SimStruct * S , void * diag ) { _ssReportDiagnosticAsWarning ( S , diag ) ; }
static void mdlOutputs ( SimStruct * S , int_T tid ) { real32_T rtb_B_13_5_0
; boolean_T rtb_B_13_4_0 ; real32_T rtb_B_15_28_0 ; real32_T rtb_B_15_29_0 ;
real_T rtb_B_15_30_0 ; real_T rtb_B_15_37_0 ; real_T taskTimeV ; real_T ratio
; uint32_T numCycles ; real_T rtb_B_13_0_0 ; ZCEventType zcEvent ; boolean_T
rtb_B_10_3_0 ; int32_T isHit ; B_AbstractFuelControl_breach_T * _rtB ;
P_AbstractFuelControl_breach_T * _rtP ; X_AbstractFuelControl_breach_T * _rtX
; PrevZCX_AbstractFuelControl_breach_T * _rtZCE ;
DW_AbstractFuelControl_breach_T * _rtDW ; _rtDW = ( (
DW_AbstractFuelControl_breach_T * ) ssGetRootDWork ( S ) ) ; _rtZCE = ( (
PrevZCX_AbstractFuelControl_breach_T * ) _ssGetPrevZCSigState ( S ) ) ; _rtX
= ( ( X_AbstractFuelControl_breach_T * ) ssGetContStates ( S ) ) ; _rtP = ( (
P_AbstractFuelControl_breach_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_AbstractFuelControl_breach_T * ) _ssGetModelBlockIO ( S ) ) ; _rtB ->
B_15_0_0 = _rtX -> p00543bar_CSTATE ; if ( ( (
ExternalUPtrs_AbstractFuelControl_breach_T * ) ssGetU ( S ) ) -> Engine_Speed
> _rtP -> P_8 ) { _rtB -> B_15_1_0 = _rtP -> P_8 ; } else if ( ( (
ExternalUPtrs_AbstractFuelControl_breach_T * ) ssGetU ( S ) ) -> Engine_Speed
< _rtP -> P_9 ) { _rtB -> B_15_1_0 = _rtP -> P_9 ; } else { _rtB -> B_15_1_0
= ( ( ExternalUPtrs_AbstractFuelControl_breach_T * ) ssGetU ( S ) ) ->
Engine_Speed ; } _rtB -> B_15_2_0 = _rtP -> P_10 * _rtB -> B_15_1_0 ; _rtB ->
B_15_4_0 = ( ( ( 0.08979 * _rtB -> B_15_0_0 * _rtB -> B_15_2_0 + - 0.366 ) -
0.0337 * _rtB -> B_15_2_0 * _rtB -> B_15_0_0 * _rtB -> B_15_0_0 ) + 0.0001 *
_rtB -> B_15_0_0 * _rtB -> B_15_2_0 * _rtB -> B_15_2_0 ) * _rtP -> P_86 ; {
if ( ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_0_PWORK . AQHandles
|| _rtDW -> HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_0_PWORK
. SlioLTF ) && ssGetLogOutput ( S ) ) { sdiSlioSdiWriteSignal ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_0_PWORK . AQHandles
, _rtDW -> HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_0_PWORK .
SlioLTF , 0 , ssGetTaskTime ( S , 0 ) , ( void * ) & _rtB -> B_15_4_0 ) ; } }
{ if ( ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_1_PWORK . AQHandles
|| _rtDW -> HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_1_PWORK
. SlioLTF ) && ssGetLogOutput ( S ) ) { sdiSlioSdiWriteSignal ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_1_PWORK . AQHandles
, _rtDW -> HiddenToAsyncQueue_InsertedFor_IntakeManifold_at_outport_1_PWORK .
SlioLTF , 0 , ssGetTaskTime ( S , 0 ) , ( void * ) & _rtB -> B_15_0_0 ) ; } }
_rtB -> B_15_7_0 = 0.0 ; _rtB -> B_15_7_0 += _rtP -> P_12 * _rtX ->
Throttledelay_CSTATE ; _rtB -> B_15_9_0 = _rtB -> B_15_7_0 + _rtB -> B_15_8_0
; if ( _rtB -> B_15_9_0 > _rtP -> P_14 ) { _rtB -> B_15_10_0 = _rtP -> P_14 ;
} else if ( _rtB -> B_15_9_0 < _rtP -> P_15 ) { _rtB -> B_15_10_0 = _rtP ->
P_15 ; } else { _rtB -> B_15_10_0 = _rtB -> B_15_9_0 ; } _rtB -> B_15_13_0 =
_rtB -> B_15_0_0 / _rtB -> B_15_12_0 ; _rtB -> B_15_14_0 = 1.0 / _rtB ->
B_15_0_0 * _rtB -> B_15_12_0 ; _rtB -> B_15_15_0 = muDoubleScalarMin ( _rtB
-> B_15_13_0 , _rtB -> B_15_14_0 ) ; if ( _rtB -> B_15_15_0 >= _rtP -> P_18 )
{ rtb_B_15_30_0 = _rtB -> B_15_15_0 - _rtB -> B_15_15_0 * _rtB -> B_15_15_0 ;
if ( rtb_B_15_30_0 < 0.0 ) { rtb_B_15_30_0 = - muDoubleScalarSqrt ( -
rtb_B_15_30_0 ) ; } else { rtb_B_15_30_0 = muDoubleScalarSqrt ( rtb_B_15_30_0
) ; } _rtB -> B_15_18_0 = 2.0 * rtb_B_15_30_0 ; } else { _rtB -> B_15_18_0 =
_rtB -> B_15_16_0 ; } _rtB -> B_15_19_0 = _rtB -> B_15_12_0 - _rtB ->
B_15_0_0 ; _rtB -> B_15_20_0 = muDoubleScalarSign ( _rtB -> B_15_19_0 ) ;
_rtB -> B_15_21_0 = ( ( ( 2.821 - 0.05231 * _rtB -> B_15_10_0 ) + 0.10299 *
_rtB -> B_15_10_0 * _rtB -> B_15_10_0 ) - 0.00063 * _rtB -> B_15_10_0 * _rtB
-> B_15_10_0 * _rtB -> B_15_10_0 ) * _rtB -> B_15_18_0 * _rtB -> B_15_20_0 ;
{ if ( ( _rtDW -> HiddenToAsyncQueue_InsertedFor_Throttle_at_outport_0_PWORK
. AQHandles || _rtDW ->
HiddenToAsyncQueue_InsertedFor_Throttle_at_outport_0_PWORK . SlioLTF ) &&
ssGetLogOutput ( S ) ) { sdiSlioSdiWriteSignal ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_Throttle_at_outport_0_PWORK . AQHandles ,
_rtDW -> HiddenToAsyncQueue_InsertedFor_Throttle_at_outport_0_PWORK . SlioLTF
, 0 , ssGetTaskTime ( S , 0 ) , ( void * ) & _rtB -> B_15_21_0 ) ; } } _rtB
-> B_15_23_0 = _rtP -> P_19 * _rtB -> B_15_2_0 ; _rtB -> B_15_24_0 = _rtB ->
B_15_4_0 / _rtB -> B_15_2_0 ; _rtB -> B_15_25_0 = _rtP -> P_20 * _rtB ->
B_15_24_0 ; _rtB -> B_15_27_0 = _rtP -> P_24 * look2_binlxpw ( _rtB ->
B_15_23_0 , _rtB -> B_15_25_0 , _rtP -> P_22 , _rtP -> P_23 , _rtP -> P_21 ,
_rtP -> P_87 , 5U ) ; rtb_B_15_28_0 = ( real32_T ) _rtB -> B_15_2_0 ;
rtb_B_15_29_0 = ( real32_T ) _rtB -> B_15_10_0 ; rtb_B_15_30_0 = _rtP -> P_25
* _rtB -> B_15_21_0 ; rtb_B_13_0_0 = ssGetTaskTime ( S , 0 ) ; _rtB ->
B_15_34_0 = _rtX -> Integrator_CSTATE ; _rtB -> B_15_35_0 = _rtP -> P_31 *
_rtB -> B_15_34_0 ; if ( rtb_B_13_0_0 < _rtP -> P_27 ) { rtb_B_13_0_0 = _rtP
-> P_28 ; } else { rtb_B_13_0_0 = _rtP -> P_29 ; } if ( rtb_B_13_0_0 >= _rtP
-> P_32 ) { rtb_B_13_0_0 = _rtB -> B_15_32_0 ; } else { rtb_B_13_0_0 = _rtB
-> B_15_35_0 ; } rtb_B_15_37_0 = _rtP -> P_33 * rtb_B_13_0_0 ; rtb_B_13_0_0 =
ssGetTaskTime ( S , 0 ) ; if ( rtb_B_13_0_0 < _rtP -> P_0 ) { rtb_B_13_0_0 =
_rtP -> P_1 ; } else { rtb_B_13_0_0 = _rtP -> P_2 ; } isHit = ssIsSampleHit (
S , 3 , 0 ) ; if ( isHit != 0 ) { taskTimeV = ssGetTaskTime ( S , 3 ) ; if (
ssGetTNextWasAdjusted ( S , 3 ) != 0 ) { _rtDW -> nextTime =
_ssGetVarNextHitTime ( S , 0 ) ; } if ( _rtDW -> justEnabled != 0 ) { _rtDW
-> justEnabled = 0 ; if ( taskTimeV >= _rtP -> P_6 ) { ratio = ( taskTimeV -
_rtP -> P_6 ) / _rtP -> P_4 ; numCycles = ( uint32_T ) muDoubleScalarFloor (
ratio ) ; if ( muDoubleScalarAbs ( ( real_T ) ( numCycles + 1U ) - ratio ) <
DBL_EPSILON * ratio ) { numCycles ++ ; } _rtDW -> numCompleteCycles =
numCycles ; ratio = ( ( real_T ) numCycles * _rtP -> P_4 + _rtP -> P_6 ) +
_rtP -> P_5 * _rtP -> P_4 / 100.0 ; if ( taskTimeV < ratio ) { _rtDW ->
currentValue = 1 ; _rtDW -> nextTime = ratio ; } else { _rtDW -> currentValue
= 0 ; _rtDW -> nextTime = ( real_T ) ( numCycles + 1U ) * _rtP -> P_4 + _rtP
-> P_6 ; } } else { _rtDW -> numCompleteCycles = _rtP -> P_6 != 0.0 ? - 1 : 0
; _rtDW -> currentValue = 0 ; _rtDW -> nextTime = _rtP -> P_6 ; } } else { if
( _rtDW -> nextTime <= taskTimeV ) { if ( _rtDW -> currentValue == 1 ) {
_rtDW -> currentValue = 0 ; _rtDW -> nextTime = ( real_T ) ( _rtDW ->
numCompleteCycles + 1LL ) * _rtP -> P_4 + _rtP -> P_6 ; } else { _rtDW ->
numCompleteCycles ++ ; _rtDW -> currentValue = 1 ; _rtDW -> nextTime = ( _rtP
-> P_5 * _rtP -> P_4 * 0.01 + ( real_T ) _rtDW -> numCompleteCycles * _rtP ->
P_4 ) + _rtP -> P_6 ; } } } _ssSetVarNextHitTime ( S , 0 , _rtDW -> nextTime
) ; if ( _rtDW -> currentValue == 1 ) { _rtB -> B_13_1_0 = _rtP -> P_3 ; }
else { _rtB -> B_13_1_0 = 0.0 ; } } _rtDW -> engine_speed = rtb_B_15_28_0 ;
vm_WriteLocalDSMNoIdx ( S , _rtDW -> dsmIdx_f , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/DataStoreWrite"
, 0 ) ; _rtDW -> throttle_angle = rtb_B_15_29_0 ; vm_WriteLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_m , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/DataStoreWrite3"
, 0 ) ; _rtDW -> throttle_flow = ( real32_T ) rtb_B_15_30_0 ;
vm_WriteLocalDSMNoIdx ( S , _rtDW -> dsmIdx_h , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/DataStoreWrite1"
, 0 ) ; _rtDW -> airbyfuel_meas = ( real32_T ) rtb_B_15_37_0 ;
vm_WriteLocalDSMNoIdx ( S , _rtDW -> dsmIdx_ha , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/DataStoreWrite2"
, 0 ) ; _rtB -> B_12_4_0 = rtb_B_13_0_0 ; isHit = ssIsSampleHit ( S , 1 , 0 )
; if ( ( isHit != 0 ) && ( ssIsMajorTimeStep ( S ) != 0 ) ) { zcEvent =
rt_ZCFcn ( RISING_ZERO_CROSSING , & _rtZCE -> fuel_controller_pwon_Trig_ZCE ,
_rtB -> B_12_4_0 ) ; if ( zcEvent != NO_ZCEVENT ) { _rtDW -> controller_mode
= ( _rtP -> P_77 != 0.0F ) ; vm_WriteLocalDSMNoIdx ( S , _rtDW -> dsmIdx_l ,
( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_pwon/DataStoreWrite1"
, 0 ) ; _rtDW -> commanded_fuel = _rtP -> P_78 ; vm_WriteLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_pwon/DataStoreWrite"
, 0 ) ; _rtDW -> airbyfuel_ref = _rtP -> P_79 ; vm_WriteLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_e , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_pwon/DataStoreWrite2"
, 0 ) ; _rtDW -> fuel_controller_pwon_SubsysRanBC = 4 ; } zcEvent = rt_ZCFcn
( RISING_ZERO_CROSSING , & _rtZCE -> fuel_controller_mode_10ms_Trig_ZCE ,
_rtB -> B_13_1_0 ) ; if ( zcEvent != NO_ZCEVENT ) { vm_ReadLocalDSMNoIdx ( S
, _rtDW -> dsmIdx_ha , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_mode_10ms/DataStoreRead2"
, 0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_9_3_0 = ( ( _rtDW -> airbyfuel_meas <= _rtB -> B_9_0_0 ) || _rtDW ->
UnitDelay_DSTATE ) ; } _rtB -> B_7_2_0 = _rtDW -> UnitDelay2_DSTATE + _rtP ->
P_72 ; _rtB -> B_7_6_0 = ( ( _rtB -> B_7_2_0 >= _rtP -> P_73 ) || _rtDW ->
UnitDelay1_DSTATE_e ) ; rtb_B_10_3_0 = ! _rtB -> B_7_6_0 ;
vm_ReadLocalDSMNoIdx ( S , _rtDW -> dsmIdx_m , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_mode_10ms/DataStoreRead4"
, 0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { if ( _rtDW
-> UnitDelay1_DSTATE_a ) { rtb_B_15_28_0 = _rtB -> B_8_0_0 ; } else {
rtb_B_15_28_0 = _rtB -> B_8_3_0 ; } _rtB -> B_8_5_0 = ( _rtDW ->
throttle_angle >= rtb_B_15_28_0 ) ; } _rtDW -> controller_mode = ( _rtB ->
B_9_3_0 || rtb_B_10_3_0 || _rtB -> B_8_5_0 ) ; vm_WriteLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_l , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_mode_10ms/DataStoreWrite"
, 0 ) ; if ( _rtB -> B_7_6_0 && _rtB -> B_8_5_0 ) { _rtDW -> airbyfuel_ref =
_rtP -> P_70 ; } else { _rtDW -> airbyfuel_ref = _rtP -> P_69 ; }
vm_WriteLocalDSMNoIdx ( S , _rtDW -> dsmIdx_e , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_mode_10ms/DataStoreWrite1"
, 0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
UnitDelay_DSTATE = _rtB -> B_9_3_0 ; } _rtDW -> UnitDelay2_DSTATE = _rtB ->
B_7_2_0 ; _rtDW -> UnitDelay1_DSTATE_e = _rtB -> B_7_6_0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
UnitDelay1_DSTATE_a = _rtB -> B_8_5_0 ; } _rtDW ->
fuel_controller_mode_10ms_SubsysRanBC = 4 ; } zcEvent = rt_ZCFcn (
RISING_ZERO_CROSSING , & _rtZCE -> fuel_controller_10ms_Trig_ZCE , _rtB ->
B_13_1_0 ) ; if ( zcEvent != NO_ZCEVENT ) { vm_ReadLocalDSMNoIdx ( S , _rtDW
-> dsmIdx_h , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_10ms/DataStoreRead"
, 0 ) ; rtb_B_15_29_0 = _rtDW -> throttle_flow ; vm_ReadLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_f , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_10ms/DataStoreRead1"
, 0 ) ; rtb_B_15_28_0 = ( ( _rtDW -> UnitDelay1_DSTATE_d * _rtDW ->
engine_speed * _rtP -> P_56 + _rtP -> P_55 ) + _rtDW -> UnitDelay1_DSTATE_d *
_rtDW -> UnitDelay1_DSTATE_d * _rtDW -> engine_speed * _rtP -> P_57 ) + _rtDW
-> engine_speed * _rtDW -> engine_speed * _rtDW -> UnitDelay1_DSTATE_d * _rtP
-> P_58 ; _rtB -> B_1_13_0 = ( rtb_B_15_29_0 - rtb_B_15_28_0 ) * _rtP -> P_60
* _rtP -> P_54 + _rtDW -> UnitDelay1_DSTATE_d ; vm_ReadLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_e , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_10ms/DataStoreRead4"
, 0 ) ; rtb_B_15_29_0 = _rtDW -> airbyfuel_ref ; rtb_B_15_28_0 /= _rtDW ->
airbyfuel_ref ; vm_ReadLocalDSMNoIdx ( S , _rtDW -> dsmIdx_l , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_10ms/DataStoreRead3"
, 0 ) ; rtb_B_10_3_0 = _rtDW -> controller_mode ; vm_ReadLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_ha , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_10ms/DataStoreRead2"
, 0 ) ; _rtB -> B_4_10_0 = ! rtb_B_10_3_0 ; if ( _rtB -> B_4_10_0 ) {
rtb_B_15_29_0 = _rtDW -> airbyfuel_meas - rtb_B_15_29_0 ; _rtB -> B_2_6_0 =
_rtP -> P_63 * rtb_B_15_29_0 * _rtP -> P_61 + _rtDW -> UnitDelay1_DSTATE ;
_rtB -> B_2_7_0 = _rtP -> P_62 * rtb_B_15_29_0 + _rtB -> B_2_6_0 ; _rtDW ->
feedback_PI_controller_SubsysRanBC = 4 ; } if ( rtb_B_10_3_0 ) {
rtb_B_15_29_0 = _rtB -> B_4_5_0 ; } else { rtb_B_15_29_0 = _rtB -> B_4_7_0 +
_rtB -> B_2_7_0 ; if ( rtb_B_15_29_0 > _rtP -> P_52 ) { rtb_B_15_29_0 = _rtP
-> P_52 ; } else { if ( rtb_B_15_29_0 < _rtP -> P_53 ) { rtb_B_15_29_0 = _rtP
-> P_53 ; } } } rtb_B_15_28_0 *= rtb_B_15_29_0 ; if ( rtb_B_15_28_0 > _rtP ->
P_67 ) { _rtDW -> commanded_fuel = _rtP -> P_67 ; } else if ( rtb_B_15_28_0 <
_rtP -> P_68 ) { _rtDW -> commanded_fuel = _rtP -> P_68 ; } else { _rtDW ->
commanded_fuel = rtb_B_15_28_0 ; } vm_WriteLocalDSMNoIdx ( S , _rtDW ->
dsmIdx , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/fuel_controller/fuel_controller_10ms/DataStoreWrite"
, 0 ) ; _rtDW -> UnitDelay1_DSTATE_d = _rtB -> B_1_13_0 ; if ( _rtB ->
B_4_10_0 ) { _rtDW -> UnitDelay1_DSTATE = _rtB -> B_2_6_0 ; } _rtDW ->
fuel_controller_10ms_SubsysRanBC = 4 ; } } isHit = ssIsSampleHit ( S , 2 , 0
) ; if ( isHit != 0 ) { vm_ReadLocalDSMNoIdx ( S , _rtDW -> dsmIdx , ( char_T
* )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/DataStoreRead"
, 0 ) ; rtb_B_15_28_0 = _rtDW -> commanded_fuel ; vm_ReadLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_l , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/DataStoreRead1"
, 0 ) ; rtb_B_13_4_0 = _rtDW -> controller_mode ; vm_ReadLocalDSMNoIdx ( S ,
_rtDW -> dsmIdx_e , ( char_T * )
 "AbstractFuelControl_breach/Air Fuel Control Model 1/AF_Controller/DataStoreRead2"
, 0 ) ; rtb_B_13_5_0 = _rtDW -> airbyfuel_ref ; _rtB -> B_15_41_0 = _rtP ->
P_34 * rtb_B_15_28_0 ; } _rtB -> B_15_42_0 = _rtB -> B_15_27_0 * _rtB ->
B_15_41_0 ; _rtB -> B_15_43_0 = _rtX -> Integrator_CSTATE_c ; _rtB ->
B_15_45_0 = _rtP -> P_39 * look2_binlxpw ( _rtB -> B_15_23_0 , _rtB ->
B_15_25_0 , _rtP -> P_37 , _rtP -> P_38 , _rtP -> P_36 , _rtP -> P_88 , 5U )
; _rtB -> B_15_46_0 = _rtB -> B_15_43_0 / _rtB -> B_15_45_0 ; _rtB ->
B_15_47_0 = _rtB -> B_15_42_0 + _rtB -> B_15_46_0 ; { if ( ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_Wallwetting_at_outport_0_PWORK . AQHandles ||
_rtDW -> HiddenToAsyncQueue_InsertedFor_Wallwetting_at_outport_0_PWORK .
SlioLTF ) && ssGetLogOutput ( S ) ) { sdiSlioSdiWriteSignal ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_Wallwetting_at_outport_0_PWORK . AQHandles ,
_rtDW -> HiddenToAsyncQueue_InsertedFor_Wallwetting_at_outport_0_PWORK .
SlioLTF , 0 , ssGetTaskTime ( S , 0 ) , ( void * ) & _rtB -> B_15_47_0 ) ; }
} _rtB -> B_15_49_0 = _rtX -> Integrator_CSTATE_h ; _rtB -> B_15_50_0 = _rtB
-> B_15_49_0 - _rtB -> B_15_34_0 ; _rtB -> B_15_51_0 = _rtP -> P_41 * _rtB ->
B_15_50_0 ; _rtB -> B_15_52_0 = _rtB -> B_15_4_0 / _rtB -> B_15_47_0 ; {
real_T * * uBuffer = ( real_T * * ) & _rtDW -> fuelsystemtransportdelay_PWORK
. TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & _rtDW ->
fuelsystemtransportdelay_PWORK . TUbufferPtrs [ 1 ] ; real_T * * xBuffer = (
real_T * * ) & _rtDW -> fuelsystemtransportdelay_PWORK . TUbufferPtrs [ 2 ] ;
real_T simTime = ssGetT ( S ) ; real_T appliedDelay ; _rtB -> B_15_53_0 =
AbstractFuelControl_breach_acc_rt_VTDelayfindtDInterpolate ( ( (
X_AbstractFuelControl_breach_T * ) ssGetContStates ( S ) ) ->
fuelsystemtransportdelay_CSTATE , * tBuffer , * uBuffer , * xBuffer , _rtDW
-> fuelsystemtransportdelay_IWORK . CircularBufSize , _rtDW ->
fuelsystemtransportdelay_IWORK . Head , _rtDW ->
fuelsystemtransportdelay_IWORK . Tail , & _rtDW ->
fuelsystemtransportdelay_IWORK . Last , simTime , 0.0 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
, _rtP -> P_43 , & appliedDelay ) ; } _rtB -> B_15_54_0 = _rtB -> B_15_53_0 -
_rtB -> B_15_49_0 ; _rtB -> B_15_55_0 = _rtP -> P_44 * _rtB -> B_15_54_0 ;
_rtB -> B_15_56_0 = _rtP -> P_45 * _rtB -> B_15_2_0 ; _rtB -> B_15_57_0 =
look2_binlxpw ( _rtB -> B_15_56_0 , _rtB -> B_15_25_0 , _rtP -> P_47 , _rtP
-> P_48 , _rtP -> P_46 , _rtP -> P_89 , 5U ) ; _rtB -> B_15_58_0 = _rtB ->
B_15_21_0 - _rtB -> B_15_4_0 ; _rtB -> B_15_59_0 = _rtP -> P_49 * _rtB ->
B_15_58_0 ; _rtB -> B_15_60_0 = _rtP -> P_50 * _rtB -> B_15_27_0 ; _rtB ->
B_15_62_0 = _rtB -> B_15_60_0 + _rtB -> B_15_61_0 ; _rtB -> B_15_63_0 = _rtB
-> B_15_41_0 * _rtB -> B_15_62_0 ; _rtB -> B_15_64_0 = _rtB -> B_15_63_0 -
_rtB -> B_15_46_0 ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) {
_rtB -> B_16_1_0 = rtb_B_13_5_0 ; ssCallAccelRunBlock ( S , 16 , 2 ,
SS_CALL_MDL_OUTPUTS ) ; } ssCallAccelRunBlock ( S , 16 , 3 ,
SS_CALL_MDL_OUTPUTS ) ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0
) { _rtB -> B_16_4_0 = rtb_B_13_4_0 ; ssCallAccelRunBlock ( S , 16 , 5 ,
SS_CALL_MDL_OUTPUTS ) ; } { if ( ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_EngineSpeed_at_outport_0_PWORK . AQHandles ||
_rtDW -> HiddenToAsyncQueue_InsertedFor_EngineSpeed_at_outport_0_PWORK .
SlioLTF ) && ssGetLogOutput ( S ) ) { sdiSlioSdiWriteSignal ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_EngineSpeed_at_outport_0_PWORK . AQHandles ,
_rtDW -> HiddenToAsyncQueue_InsertedFor_EngineSpeed_at_outport_0_PWORK .
SlioLTF , 0 , ssGetTaskTime ( S , 0 ) , ( void * ) & ( (
ExternalUPtrs_AbstractFuelControl_breach_T * ) ssGetU ( S ) ) -> Engine_Speed
) ; } } { if ( ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_PedalAngle_at_outport_0_PWORK . AQHandles ||
_rtDW -> HiddenToAsyncQueue_InsertedFor_PedalAngle_at_outport_0_PWORK .
SlioLTF ) && ssGetLogOutput ( S ) ) { sdiSlioSdiWriteSignal ( _rtDW ->
HiddenToAsyncQueue_InsertedFor_PedalAngle_at_outport_0_PWORK . AQHandles ,
_rtDW -> HiddenToAsyncQueue_InsertedFor_PedalAngle_at_outport_0_PWORK .
SlioLTF , 0 , ssGetTaskTime ( S , 0 ) , ( void * ) & ( (
ExternalUPtrs_AbstractFuelControl_breach_T * ) ssGetU ( S ) ) -> Pedal_Angle
) ; } } UNUSED_PARAMETER ( tid ) ; } static void mdlOutputsTID4 ( SimStruct *
S , int_T tid ) { B_AbstractFuelControl_breach_T * _rtB ;
P_AbstractFuelControl_breach_T * _rtP ; _rtP = ( (
P_AbstractFuelControl_breach_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_AbstractFuelControl_breach_T * ) _ssGetModelBlockIO ( S ) ) ; _rtB ->
B_15_8_0 = _rtP -> P_13 ; _rtB -> B_15_12_0 = _rtP -> P_16 ; _rtB ->
B_15_16_0 = _rtP -> P_17 ; _rtB -> B_15_32_0 = _rtP -> P_26 ; _rtB -> B_9_0_0
= _rtP -> P_76 ; _rtB -> B_8_0_0 = _rtP -> P_74 ; _rtB -> B_8_3_0 = _rtB ->
B_8_0_0 + _rtP -> P_75 ; _rtB -> B_4_5_0 = _rtP -> P_65 ; _rtB -> B_4_7_0 =
_rtP -> P_66 ; _rtB -> B_15_61_0 = _rtP -> P_51 ; UNUSED_PARAMETER ( tid ) ;
}
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) {
B_AbstractFuelControl_breach_T * _rtB ; P_AbstractFuelControl_breach_T * _rtP
; DW_AbstractFuelControl_breach_T * _rtDW ; _rtDW = ( (
DW_AbstractFuelControl_breach_T * ) ssGetRootDWork ( S ) ) ; _rtP = ( (
P_AbstractFuelControl_breach_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_AbstractFuelControl_breach_T * ) _ssGetModelBlockIO ( S ) ) ; { real_T * *
uBuffer = ( real_T * * ) & _rtDW -> fuelsystemtransportdelay_PWORK .
TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & _rtDW ->
fuelsystemtransportdelay_PWORK . TUbufferPtrs [ 1 ] ; real_T * * xBuffer = (
real_T * * ) & _rtDW -> fuelsystemtransportdelay_PWORK . TUbufferPtrs [ 2 ] ;
real_T simTime = ssGetT ( S ) ; _rtDW -> fuelsystemtransportdelay_IWORK .
Head = ( ( _rtDW -> fuelsystemtransportdelay_IWORK . Head < ( _rtDW ->
fuelsystemtransportdelay_IWORK . CircularBufSize - 1 ) ) ? ( _rtDW ->
fuelsystemtransportdelay_IWORK . Head + 1 ) : 0 ) ; if ( _rtDW ->
fuelsystemtransportdelay_IWORK . Head == _rtDW ->
fuelsystemtransportdelay_IWORK . Tail ) { if ( !
AbstractFuelControl_breach_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
fuelsystemtransportdelay_IWORK . CircularBufSize , & _rtDW ->
fuelsystemtransportdelay_IWORK . Tail , & _rtDW ->
fuelsystemtransportdelay_IWORK . Head , & _rtDW ->
fuelsystemtransportdelay_IWORK . Last , simTime - _rtP -> P_42 , tBuffer ,
uBuffer , xBuffer , ( boolean_T ) 0 , ( boolean_T ) 1 , & _rtDW ->
fuelsystemtransportdelay_IWORK . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"vtdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
fuelsystemtransportdelay_IWORK . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
fuelsystemtransportdelay_IWORK . Head ] = _rtB -> B_15_52_0 ; ( * xBuffer ) [
_rtDW -> fuelsystemtransportdelay_IWORK . Head ] = ( (
X_AbstractFuelControl_breach_T * ) ssGetContStates ( S ) ) ->
fuelsystemtransportdelay_CSTATE ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdateTID4 ( SimStruct * S , int_T tid ) { UNUSED_PARAMETER (
tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { B_AbstractFuelControl_breach_T
* _rtB ; P_AbstractFuelControl_breach_T * _rtP ;
X_AbstractFuelControl_breach_T * _rtX ; XDot_AbstractFuelControl_breach_T *
_rtXdot ; DW_AbstractFuelControl_breach_T * _rtDW ; _rtDW = ( (
DW_AbstractFuelControl_breach_T * ) ssGetRootDWork ( S ) ) ; _rtXdot = ( (
XDot_AbstractFuelControl_breach_T * ) ssGetdX ( S ) ) ; _rtX = ( (
X_AbstractFuelControl_breach_T * ) ssGetContStates ( S ) ) ; _rtP = ( (
P_AbstractFuelControl_breach_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_AbstractFuelControl_breach_T * ) _ssGetModelBlockIO ( S ) ) ; _rtXdot ->
p00543bar_CSTATE = _rtB -> B_15_59_0 ; _rtXdot -> Throttledelay_CSTATE = 0.0
; _rtXdot -> Throttledelay_CSTATE += _rtP -> P_11 * _rtX ->
Throttledelay_CSTATE ; _rtXdot -> Throttledelay_CSTATE += ( (
ExternalUPtrs_AbstractFuelControl_breach_T * ) ssGetU ( S ) ) -> Pedal_Angle
; _rtXdot -> Integrator_CSTATE = _rtB -> B_15_51_0 ; _rtXdot ->
Integrator_CSTATE_c = _rtB -> B_15_64_0 ; _rtXdot -> Integrator_CSTATE_h =
_rtB -> B_15_55_0 ; { real_T instantDelay ; instantDelay = _rtB -> B_15_57_0
; if ( instantDelay > _rtP -> P_42 ) { instantDelay = _rtP -> P_42 ; } if (
instantDelay < 0.0 ) { ( ( XDot_AbstractFuelControl_breach_T * ) ssGetdX ( S
) ) -> fuelsystemtransportdelay_CSTATE = 0 ; } else { ( (
XDot_AbstractFuelControl_breach_T * ) ssGetdX ( S ) ) ->
fuelsystemtransportdelay_CSTATE = 1.0 / instantDelay ; } } } static void
mdlInitializeSizes ( SimStruct * S ) { ssSetChecksumVal ( S , 0 , 4168702261U
) ; ssSetChecksumVal ( S , 1 , 1319474928U ) ; ssSetChecksumVal ( S , 2 ,
189067204U ) ; ssSetChecksumVal ( S , 3 , 3092185684U ) ; { mxArray *
slVerStructMat = NULL ; mxArray * slStrMat = mxCreateString ( "simulink" ) ;
char slVerChar [ 10 ] ; int status = mexCallMATLAB ( 1 , & slVerStructMat , 1
, & slStrMat , "ver" ) ; if ( status == 0 ) { mxArray * slVerMat = mxGetField
( slVerStructMat , 0 , "Version" ) ; if ( slVerMat == NULL ) { status = 1 ; }
else { status = mxGetString ( slVerMat , slVerChar , 10 ) ; } }
mxDestroyArray ( slStrMat ) ; mxDestroyArray ( slVerStructMat ) ; if ( (
status == 1 ) || ( strcmp ( slVerChar , "9.2" ) != 0 ) ) { return ; } }
ssSetOptions ( S , SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork (
S ) != sizeof ( DW_AbstractFuelControl_breach_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( B_AbstractFuelControl_breach_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofU ( S ) != sizeof
( ExternalUPtrs_AbstractFuelControl_breach_T ) ) { static char msg [ 256 ] ;
sprintf ( msg , "Unexpected error: Internal ExternalInputs sizes do "
"not match for accelerator mex file." ) ; ssSetErrorStatus ( S , msg ) ; } if
( ssGetSizeofY ( S ) != sizeof ( ExtY_AbstractFuelControl_breach_T ) ) {
static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal ExternalOutputs sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
P_AbstractFuelControl_breach_T ) ) { static char msg [ 256 ] ; sprintf ( msg
, "Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetModelRtp ( S , ( real_T *
) & AbstractFuelControl_breach_rtDefaultP ) ; rt_InitInfAndNaN ( sizeof (
real_T ) ) ; ( ( P_AbstractFuelControl_breach_T * ) ssGetModelRtp ( S ) ) ->
P_52 = rtInfF ; } static void mdlInitializeSampleTimes ( SimStruct * S ) {
slAccRegPrmChangeFcn ( S , mdlOutputsTID4 ) ; } static void mdlTerminate (
SimStruct * S ) { }
#include "simulink.c"
