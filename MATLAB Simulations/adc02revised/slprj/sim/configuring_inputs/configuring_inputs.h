#ifndef configuring_inputs_h_
#define configuring_inputs_h_
#ifndef configuring_inputs_COMMON_INCLUDES_
#define configuring_inputs_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "math.h"
#endif
#include "configuring_inputs_types.h"
#include "model_reference_types.h"
#include "rtw_modelmap_simtarget.h"
#include "rt_nonfinite.h"
#include <string.h>
#include <stddef.h>
struct mtpbypmub4e_ { real_T P_0 [ 3 ] ; real_T P_1 [ 3 ] ; real_T P_2 ;
real_T P_3 ; real_T P_4 ; real_T P_5 ; real_T P_6 ; real_T P_7 ; real_T P_8 ;
real_T P_9 ; real_T P_10 ; real_T P_11 ; real_T P_12 ; real_T P_13 ; real_T
P_14 [ 3 ] ; } ; struct m5xks5eepa { struct SimStruct_tag * _mdlRefSfcnS ;
const rtTimingBridge * timingBridge ; struct { rtwCAPI_ModelMappingInfo mmi ;
rtwCAPI_ModelMapLoggingInstanceInfo mmiLogInstanceInfo ; sysRanDType *
systemRan [ 2 ] ; int_T systemTid [ 2 ] ; } DataMapInfo ; struct { int_T
mdlref_GlobalTID [ 2 ] ; } Timing ; } ; typedef struct { hxg1owdhqb rtm ; }
ls5jnluhzoy ; extern void mv5wr4riqo ( SimStruct * _mdlRefSfcnS , int_T
mdlref_TID0 , int_T mdlref_TID1 , hxg1owdhqb * const g0tuafjos4 , void *
sysRanPtr , int contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI , const
char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ; extern
void mr_configuring_inputs_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) ; extern mxArray *
mr_configuring_inputs_GetDWork ( const ls5jnluhzoy * mdlrefDW ) ; extern void
mr_configuring_inputs_SetDWork ( ls5jnluhzoy * mdlrefDW , const mxArray *
ssDW ) ; extern void mr_configuring_inputs_RegisterSimStateChecksum (
SimStruct * S ) ; extern mxArray *
mr_configuring_inputs_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * configuring_inputs_GetCAPIStaticMap ( void )
; extern void gwczjw5orj ( real_T aqdrrwdfqa [ 3 ] , real_T eks12rzq1l [ 3 ]
, real_T l1qoxp4kck [ 3 ] ) ; extern void pmppdciymi ( real_T aqdrrwdfqa [ 3
] , real_T eks12rzq1l [ 3 ] , real_T l1qoxp4kck [ 3 ] ) ; extern void
deferexa3n ( real_T aqdrrwdfqa [ 3 ] , real_T eks12rzq1l [ 3 ] , real_T
l1qoxp4kck [ 3 ] ) ; extern void j4x43rulk4 ( void ) ; extern void
configuring_inputs ( hxg1owdhqb * const g0tuafjos4 , real_T aqdrrwdfqa [ 3 ]
, real_T eks12rzq1l [ 3 ] , real_T hdhqc2gs3n [ 3 ] , real_T l1qoxp4kck [ 3 ]
) ; extern void gsehe0mymu ( hxg1owdhqb * const g0tuafjos4 ) ;
#endif
