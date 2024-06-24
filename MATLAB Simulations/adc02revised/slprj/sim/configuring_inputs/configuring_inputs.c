#include "configuring_inputs.h"
#include "rtwtypes.h"
#include "configuring_inputs_private.h"
#include "mwmathutil.h"
#include "configuring_inputs_capi.h"
static RegMdlInfo rtMdlInfo_configuring_inputs [ 41 ] = { { "ls5jnluhzoy" ,
MDL_INFO_NAME_MDLREF_DWORK , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"ap3gfcfnhv" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "id2o3vc2dc" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "l5ugph2j2d" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "dkbf5dvso0" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "eamk2vipq5" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "mlinjyxxgk" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "plfmrj3puw" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "ijabtqpxdp" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "dcjzvvzjcl" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "aak1p0xpmo" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "n2flerfp2f" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "g4pe3jpzjt" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "pndhgg0tbl" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "hdem5l35ke" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "gsehe0mymu" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "j4x43rulk4" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "pmppdciymi" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "gwczjw5orj" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "mv5wr4riqo" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "deferexa3n" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , { "fbfidjqn2l" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "configuring_inputs" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , 0 , (
NULL ) } , { "lii44v2p2h" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , (
void * ) "configuring_inputs" } , { "mtpbypmub4e" ,
MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * ) "configuring_inputs"
} , { "m5xks5eepa" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT , 0 , - 1 , ( void * )
"configuring_inputs" } , { "hxg1owdhqb" , MDL_INFO_ID_GLOBAL_RTW_CONSTRUCT ,
0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_GetSimStateDisallowedBlocks" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_extractBitFieldFromCellArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_cacheBitFieldToCellArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_restoreDataFromMxArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_cacheDataToMxArrayWithOffset" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_extractBitFieldFromMxArray" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_cacheBitFieldToMxArray" , MDL_INFO_ID_MODEL_FCN_NAME ,
0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_restoreDataFromMxArray" , MDL_INFO_ID_MODEL_FCN_NAME ,
0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_cacheDataAsMxArray" , MDL_INFO_ID_MODEL_FCN_NAME , 0 ,
- 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_RegisterSimStateChecksum" , MDL_INFO_ID_MODEL_FCN_NAME
, 0 , - 1 , ( void * ) "configuring_inputs" } , {
"mr_configuring_inputs_SetDWork" , MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , (
void * ) "configuring_inputs" } , { "mr_configuring_inputs_GetDWork" ,
MDL_INFO_ID_MODEL_FCN_NAME , 0 , - 1 , ( void * ) "configuring_inputs" } , {
"configuring_inputs.h" , MDL_INFO_MODEL_FILENAME , 0 , - 1 , ( NULL ) } , {
"configuring_inputs.c" , MDL_INFO_MODEL_FILENAME , 0 , - 1 , ( void * )
"configuring_inputs" } } ; mtpbypmub4e mtpbypmub4 = { { 0.0 , 0.0 , 0.0 } , {
0.0 , 0.0 , 0.0 } , 0.1 , 0.1 , 1.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0
, 1.0 , 0.0 , { 1.0 , 1.0 , 1.0 } } ; void gwczjw5orj ( real_T aqdrrwdfqa [ 3
] , real_T eks12rzq1l [ 3 ] , real_T l1qoxp4kck [ 3 ] ) { aqdrrwdfqa [ 0 ] =
mtpbypmub4 . P_0 [ 0 ] ; eks12rzq1l [ 0 ] = mtpbypmub4 . P_1 [ 0 ] ;
l1qoxp4kck [ 0 ] = mtpbypmub4 . P_14 [ 0 ] ; aqdrrwdfqa [ 1 ] = mtpbypmub4 .
P_0 [ 1 ] ; eks12rzq1l [ 1 ] = mtpbypmub4 . P_1 [ 1 ] ; l1qoxp4kck [ 1 ] =
mtpbypmub4 . P_14 [ 1 ] ; aqdrrwdfqa [ 2 ] = mtpbypmub4 . P_0 [ 2 ] ;
eks12rzq1l [ 2 ] = mtpbypmub4 . P_1 [ 2 ] ; l1qoxp4kck [ 2 ] = mtpbypmub4 .
P_14 [ 2 ] ; } void pmppdciymi ( real_T aqdrrwdfqa [ 3 ] , real_T eks12rzq1l
[ 3 ] , real_T l1qoxp4kck [ 3 ] ) { aqdrrwdfqa [ 0 ] = mtpbypmub4 . P_0 [ 0 ]
; eks12rzq1l [ 0 ] = mtpbypmub4 . P_1 [ 0 ] ; l1qoxp4kck [ 0 ] = mtpbypmub4 .
P_14 [ 0 ] ; aqdrrwdfqa [ 1 ] = mtpbypmub4 . P_0 [ 1 ] ; eks12rzq1l [ 1 ] =
mtpbypmub4 . P_1 [ 1 ] ; l1qoxp4kck [ 1 ] = mtpbypmub4 . P_14 [ 1 ] ;
aqdrrwdfqa [ 2 ] = mtpbypmub4 . P_0 [ 2 ] ; eks12rzq1l [ 2 ] = mtpbypmub4 .
P_1 [ 2 ] ; l1qoxp4kck [ 2 ] = mtpbypmub4 . P_14 [ 2 ] ; } void deferexa3n (
real_T aqdrrwdfqa [ 3 ] , real_T eks12rzq1l [ 3 ] , real_T l1qoxp4kck [ 3 ] )
{ aqdrrwdfqa [ 0 ] = mtpbypmub4 . P_0 [ 0 ] ; eks12rzq1l [ 0 ] = mtpbypmub4 .
P_1 [ 0 ] ; l1qoxp4kck [ 0 ] = mtpbypmub4 . P_14 [ 0 ] ; aqdrrwdfqa [ 1 ] =
mtpbypmub4 . P_0 [ 1 ] ; eks12rzq1l [ 1 ] = mtpbypmub4 . P_1 [ 1 ] ;
l1qoxp4kck [ 1 ] = mtpbypmub4 . P_14 [ 1 ] ; aqdrrwdfqa [ 2 ] = mtpbypmub4 .
P_0 [ 2 ] ; eks12rzq1l [ 2 ] = mtpbypmub4 . P_1 [ 2 ] ; l1qoxp4kck [ 2 ] =
mtpbypmub4 . P_14 [ 2 ] ; } void configuring_inputs ( hxg1owdhqb * const
g0tuafjos4 , real_T aqdrrwdfqa [ 3 ] , real_T eks12rzq1l [ 3 ] , real_T
hdhqc2gs3n [ 3 ] , real_T l1qoxp4kck [ 3 ] ) { real_T tmp ; if (
rtmIsMajorTimeStep ( g0tuafjos4 ) && rtmIsSampleHit ( g0tuafjos4 , 1 , 0 ) )
{ aqdrrwdfqa [ 0 ] = mtpbypmub4 . P_0 [ 0 ] ; eks12rzq1l [ 0 ] = mtpbypmub4 .
P_1 [ 0 ] ; aqdrrwdfqa [ 1 ] = mtpbypmub4 . P_0 [ 1 ] ; eks12rzq1l [ 1 ] =
mtpbypmub4 . P_1 [ 1 ] ; aqdrrwdfqa [ 2 ] = mtpbypmub4 . P_0 [ 2 ] ;
eks12rzq1l [ 2 ] = mtpbypmub4 . P_1 [ 2 ] ; l1qoxp4kck [ 0 ] = mtpbypmub4 .
P_14 [ 0 ] ; l1qoxp4kck [ 1 ] = mtpbypmub4 . P_14 [ 1 ] ; l1qoxp4kck [ 2 ] =
mtpbypmub4 . P_14 [ 2 ] ; } tmp = rtmGetTaskTime ( g0tuafjos4 , 0 ) ;
hdhqc2gs3n [ 0 ] = muDoubleScalarSin ( mtpbypmub4 . P_4 * tmp + mtpbypmub4 .
P_5 ) * mtpbypmub4 . P_2 + mtpbypmub4 . P_3 ; hdhqc2gs3n [ 1 ] =
muDoubleScalarSin ( mtpbypmub4 . P_8 * tmp + mtpbypmub4 . P_9 ) * mtpbypmub4
. P_6 + mtpbypmub4 . P_7 ; hdhqc2gs3n [ 2 ] = muDoubleScalarSin ( mtpbypmub4
. P_12 * tmp + mtpbypmub4 . P_13 ) * mtpbypmub4 . P_10 + mtpbypmub4 . P_11 ;
} void j4x43rulk4 ( void ) { } void gsehe0mymu ( hxg1owdhqb * const
g0tuafjos4 ) { if ( ! slIsRapidAcceleratorSimulating ( ) ) {
slmrRunPluginEvent ( g0tuafjos4 -> _mdlRefSfcnS , "configuring_inputs" ,
"SIMSTATUS_TERMINATING_MODELREF_ACCEL_EVENT" ) ; } } void mv5wr4riqo (
SimStruct * _mdlRefSfcnS , int_T mdlref_TID0 , int_T mdlref_TID1 , hxg1owdhqb
* const g0tuafjos4 , void * sysRanPtr , int contextTid ,
rtwCAPI_ModelMappingInfo * rt_ParentMMI , const char_T * rt_ChildPath , int_T
rt_ChildMMIIdx , int_T rt_CSTATEIdx ) { ( void ) memset ( ( void * )
g0tuafjos4 , 0 , sizeof ( hxg1owdhqb ) ) ; g0tuafjos4 -> Timing .
mdlref_GlobalTID [ 0 ] = mdlref_TID0 ; g0tuafjos4 -> Timing .
mdlref_GlobalTID [ 1 ] = mdlref_TID1 ; g0tuafjos4 -> _mdlRefSfcnS = (
_mdlRefSfcnS ) ; if ( ! slIsRapidAcceleratorSimulating ( ) ) {
slmrRunPluginEvent ( g0tuafjos4 -> _mdlRefSfcnS , "configuring_inputs" ,
"START_OF_SIM_MODEL_MODELREF_ACCEL_EVENT" ) ; }
configuring_inputs_InitializeDataMapInfo ( g0tuafjos4 , sysRanPtr ,
contextTid ) ; if ( ( rt_ParentMMI != ( NULL ) ) && ( rt_ChildPath != ( NULL
) ) ) { rtwCAPI_SetChildMMI ( * rt_ParentMMI , rt_ChildMMIIdx , & (
g0tuafjos4 -> DataMapInfo . mmi ) ) ; rtwCAPI_SetPath ( g0tuafjos4 ->
DataMapInfo . mmi , rt_ChildPath ) ; rtwCAPI_MMISetContStateStartIndex (
g0tuafjos4 -> DataMapInfo . mmi , rt_CSTATEIdx ) ; } } void
mr_configuring_inputs_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) { * retVal = 0 ; { boolean_T regSubmodelsMdlinfo
= false ; ssGetRegSubmodelsMdlinfo ( mdlRefSfcnS , & regSubmodelsMdlinfo ) ;
if ( regSubmodelsMdlinfo ) { } } * retVal = 0 ; ssRegModelRefMdlInfo (
mdlRefSfcnS , modelName , rtMdlInfo_configuring_inputs , 41 ) ; * retVal = 1
; } static void mr_configuring_inputs_cacheDataAsMxArray ( mxArray *
destArray , mwIndex i , int j , const void * srcData , size_t numBytes ) ;
static void mr_configuring_inputs_cacheDataAsMxArray ( mxArray * destArray ,
mwIndex i , int j , const void * srcData , size_t numBytes ) { mxArray *
newArray = mxCreateUninitNumericMatrix ( ( size_t ) 1 , numBytes ,
mxUINT8_CLASS , mxREAL ) ; memcpy ( ( uint8_T * ) mxGetData ( newArray ) , (
const uint8_T * ) srcData , numBytes ) ; mxSetFieldByNumber ( destArray , i ,
j , newArray ) ; } static void mr_configuring_inputs_restoreDataFromMxArray (
void * destData , const mxArray * srcArray , mwIndex i , int j , size_t
numBytes ) ; static void mr_configuring_inputs_restoreDataFromMxArray ( void
* destData , const mxArray * srcArray , mwIndex i , int j , size_t numBytes )
{ memcpy ( ( uint8_T * ) destData , ( const uint8_T * ) mxGetData (
mxGetFieldByNumber ( srcArray , i , j ) ) , numBytes ) ; } static void
mr_configuring_inputs_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex
i , int j , uint_T bitVal ) ; static void
mr_configuring_inputs_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex
i , int j , uint_T bitVal ) { mxSetFieldByNumber ( destArray , i , j ,
mxCreateDoubleScalar ( ( real_T ) bitVal ) ) ; } static uint_T
mr_configuring_inputs_extractBitFieldFromMxArray ( const mxArray * srcArray ,
mwIndex i , int j , uint_T numBits ) ; static uint_T
mr_configuring_inputs_extractBitFieldFromMxArray ( const mxArray * srcArray ,
mwIndex i , int j , uint_T numBits ) { const uint_T varVal = ( uint_T )
mxGetScalar ( mxGetFieldByNumber ( srcArray , i , j ) ) ; return varVal & ( (
1u << numBits ) - 1u ) ; } static void
mr_configuring_inputs_cacheDataToMxArrayWithOffset ( mxArray * destArray ,
mwIndex i , int j , mwIndex offset , const void * srcData , size_t numBytes )
; static void mr_configuring_inputs_cacheDataToMxArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , const void * srcData ,
size_t numBytes ) { uint8_T * varData = ( uint8_T * ) mxGetData (
mxGetFieldByNumber ( destArray , i , j ) ) ; memcpy ( ( uint8_T * ) & varData
[ offset * numBytes ] , ( const uint8_T * ) srcData , numBytes ) ; } static
void mr_configuring_inputs_restoreDataFromMxArrayWithOffset ( void * destData
, const mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t
numBytes ) ; static void
mr_configuring_inputs_restoreDataFromMxArrayWithOffset ( void * destData ,
const mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t
numBytes ) { const uint8_T * varData = ( const uint8_T * ) mxGetData (
mxGetFieldByNumber ( srcArray , i , j ) ) ; memcpy ( ( uint8_T * ) destData ,
( const uint8_T * ) & varData [ offset * numBytes ] , numBytes ) ; } static
void mr_configuring_inputs_cacheBitFieldToCellArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal ) ; static
void mr_configuring_inputs_cacheBitFieldToCellArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal ) {
mxSetCell ( mxGetFieldByNumber ( destArray , i , j ) , offset ,
mxCreateDoubleScalar ( ( real_T ) fieldVal ) ) ; } static uint_T
mr_configuring_inputs_extractBitFieldFromCellArrayWithOffset ( const mxArray
* srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) ; static
uint_T mr_configuring_inputs_extractBitFieldFromCellArrayWithOffset ( const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) {
const uint_T fieldVal = ( uint_T ) mxGetScalar ( mxGetCell (
mxGetFieldByNumber ( srcArray , i , j ) , offset ) ) ; return fieldVal & ( (
1u << numBits ) - 1u ) ; } mxArray * mr_configuring_inputs_GetDWork ( const
ls5jnluhzoy * mdlrefDW ) { ( void ) mdlrefDW ; return ( NULL ) ; } void
mr_configuring_inputs_SetDWork ( ls5jnluhzoy * mdlrefDW , const mxArray *
ssDW ) { ( void ) ssDW ; ( void ) mdlrefDW ; } void
mr_configuring_inputs_RegisterSimStateChecksum ( SimStruct * S ) { const
uint32_T chksum [ 4 ] = { 3050175831U , 3426525117U , 1007668408U ,
445591152U , } ; slmrModelRefRegisterSimStateChecksum ( S ,
"configuring_inputs" , & chksum [ 0 ] ) ; } mxArray *
mr_configuring_inputs_GetSimStateDisallowedBlocks ( ) { return ( NULL ) ; }
#if defined(_MSC_VER)
#pragma warning(disable: 4505) //unreferenced local function has been removed
#endif
