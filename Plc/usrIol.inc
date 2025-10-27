**********************************************************************

POS_AX_27                AT %POS.posax27;  * Absolute Positions x1000
POS_AX_28                AT %POS.posax28;  * Absolute Positions x1000
POS_AX_29                AT %POS.posax29;  * Absolute Positions x1000
POS_AX_30                AT %POS.posax30;  * Absolute Positions x1000
POS_AX_34                AT %POS.posax34;  * Absolute Positions x1000
POS_AX_50                AT %POS.posax50;  * Absolute Positions x1000
POS_AX_52                AT %POS.posax52;  * Absolute Positions x1000
POS_AX_54                AT %POS.posax54;  * Absolute Positions x1000
POS_AX_60                AT %POS.posax60;  * Absolute Positions x1000
POS_AX_61                AT %POS.posax61;  * Absolute Positions x1000
POS_AX_62                AT %POS.posax62;  * Absolute Positions x1000
POS_AX_63                AT %POS.posax63;  * Absolute Positions x1000
POS_AX_64                AT %POS.posax64;  * Absolute Positions x1000
POS_AX_65                AT %POS.posax65;  * Absolute Positions x1000
POS_AX_70                AT %POS.posax70;  * Absolute Positions x1000
POS_AX_72                AT %POS.posax72;  * Absolute Positions x1000

**********************************************************************

MS_Open_Button	         AT %SPINDLE_SYSTEM.boolean.0;
SMobile_Open_Button	     AT %SPINDLE_SYSTEM.boolean.1;
MS_Close_Button          AT %SPINDLE_SYSTEM.boolean.2;
SMobile_Close_Button     AT %SPINDLE_SYSTEM.boolean.3;
MS_OpenLED               AT %SPINDLE_SYSTEM.boolean.4;
FS_OpenLED               AT %SPINDLE_SYSTEM.boolean.5;
MS_AlarmPiston	         AT %SPINDLE_SYSTEM.boolean.6;
FS_AlarmPiston	         AT %SPINDLE_SYSTEM.boolean.7;
MS_AlarmOil              AT %SPINDLE_SYSTEM.boolean.8;
FS_AlarmOil              AT %SPINDLE_SYSTEM.boolean.9;
Pump_OK                  AT %SPINDLE_SYSTEM.boolean.10;
Pump_Alarm               AT %SPINDLE_SYSTEM.boolean.11;
FS_WaitM                 AT %SPINDLE_SYSTEM.boolean.12;
MS_WaitM                 AT %SPINDLE_SYSTEM.boolean.13;  
FixedSpindleOpen_Button  AT %SPINDLE_SYSTEM.boolean.14;  
FixedSpindleClose_Button AT %SPINDLE_SYSTEM.boolean.15;  
MoveAx30GuiBtn           AT %SPINDLE_SYSTEM.boolean.16; 
MoveAx31GuiBtn           AT %SPINDLE_SYSTEM.boolean.17; 
MoveAx36GuiBtn           AT %SPINDLE_SYSTEM.boolean.18; 
Invert_Mob_Spin_Open     AT %SPINDLE_SYSTEM.boolean.19;

Time_Out_Sp              AT %SPINDLE_SYSTEM.values5;       * Time out spindle

PulseMm_Y                AT %SPINDLE_SYSTEM.values6;       * pulse / mm y
PulseMm_Z                AT %SPINDLE_SYSTEM.values7;       * pulse / mm z
MobPulseMmSpindle        AT %SPINDLE_SYSTEM.values8;       * 
OffSetUClamp             AT %SPINDLE_SYSTEM.values9;
AnSpindlMobOpen          AT %SPINDLE_SYSTEM.values10;
AnSpindlFixOpen          AT %SPINDLE_SYSTEM.values11;
ResAnalogPressure        AT %SPINDLE_SYSTEM.values12;
OpenSpindlePosition      AT %SPINDLE_SYSTEM.values13;
SafeDis_XW               AT %SPINDLE_SYSTEM.values14;
 
Offset_V                AT %SPINDLE_SYSTEM.values15;     * offset third spindle
Offset_W                AT %SPINDLE_SYSTEM.values16;     * offset second spindle
LimitOtherSide_W         AT %SPINDLE_SYSTEM.values17;     * offset second spindle


Choose_welding_face     AT %SPINDLE_SYSTEM.boolean.22;
Activate_0_welding	  AT %SPINDLE_SYSTEM.boolean.23;
Rot_90_Plus  	      AT %SPINDLE_SYSTEM.boolean.24;
Rot_90_Minus 		  AT %SPINDLE_SYSTEM.boolean.25;

PosSpindleX             AT %SPINDLE_SYSTEM.values18;
PosSpindleX1            AT %SPINDLE_SYSTEM.values19;
PosSpindleX3            AT %SPINDLE_SYSTEM.values20;

VelPosSpindleX_Auto     AT %SPINDLE_SYSTEM.values21;
VelPosSpindleX1_Auto    AT %SPINDLE_SYSTEM.values22;
VelPosSpindleX3_Auto    AT %SPINDLE_SYSTEM.values23;

GrantPlcMoveX           AT %SPINDLE_SYSTEM.values24.0;
GrantPlcMoveX1          AT %SPINDLE_SYSTEM.values24.1;
GrantPlcMoveX3          AT %SPINDLE_SYSTEM.values24.2;
AncolliXW_yHoldMov      AT %SPINDLE_SYSTEM.values24.3;
AncolliWV_yHoldMov      AT %SPINDLE_SYSTEM.values24.4;

SafeDis_WV              AT %SPINDLE_SYSTEM.values25;
Emer_SafeDis_XW         AT %SPINDLE_SYSTEM.values26;
Emer_SafeDis_WV         AT %SPINDLE_SYSTEM.values27;
SafeDis_W_L1            AT %SPINDLE_SYSTEM.values28;
SafeDis_W_Dustbin       AT %SPINDLE_SYSTEM.values29;

**************************************************************
**************************************************************
****************** SUPPORTS SYSTEM *************************** 
**************************************************************
**************************************************************
****************** SUPPORTS SYSTEM ***************************
SUPPORT_1_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.0;
SUPPORT_2_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.1;
SUPPORT_3_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.2;
SUPPORT_4_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.3;
SUPPORT_5_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.4;
SUPPORT_6_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.5;
SUPPORT_7_AutoEnable     AT %SUPPORTS_SYSTEM.boolean.6;
SUPPORT_1_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.7;
SUPPORT_2_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.8;
SUPPORT_3_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.9;
SUPPORT_4_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.10;
SUPPORT_5_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.11;
SUPPORT_6_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.12;
SUPPORT_7_AlarmDwn       AT %SUPPORTS_SYSTEM.boolean.13;
SUPPORT_1_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.14;
SUPPORT_2_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.15;
SUPPORT_3_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.16;
SUPPORT_4_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.17;
SUPPORT_5_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.18;
SUPPORT_6_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.19;
SUPPORT_7_AlarmDwnCent   AT %SUPPORTS_SYSTEM.boolean.20;
SUPPORT_1_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.21;
SUPPORT_2_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.22;
SUPPORT_3_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.23;
SUPPORT_4_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.24;
SUPPORT_5_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.25;
SUPPORT_6_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.26;
SUPPORT_7_AlarmOpenCent  AT %SUPPORTS_SYSTEM.boolean.27;
All_Sup_DWN              AT %SUPPORTS_SYSTEM.boolean.28;

ISO_SupLoader_Init       AT %SUPPORTS_SYSTEM.boolean.30;
ISO_SupUn_Init           AT %SUPPORTS_SYSTEM.boolean.31;

SUPPORT_71_AutoEnable    AT %SUPPORTS_SYSTEM.boolean1.0;
SUPPORT_72_AutoEnable    AT %SUPPORTS_SYSTEM.boolean1.1;
SUPPORT_73_AutoEnable    AT %SUPPORTS_SYSTEM.boolean1.2;
SUPPORT_74_AutoEnable    AT %SUPPORTS_SYSTEM.boolean1.3;
SUPPORT_75_AutoEnable    AT %SUPPORTS_SYSTEM.boolean1.4;
SUPPORT_6_AutoEnable_Un  AT %SUPPORTS_SYSTEM.boolean1.5;
SUPPORT_7_AutoEnable_Un  AT %SUPPORTS_SYSTEM.boolean1.6;
SUPPORT_72_AlarmDwn      AT %SUPPORTS_SYSTEM.boolean1.7;
SUPPORT_71_AlarmDwn      AT %SUPPORTS_SYSTEM.boolean1.8;
SUPPORT_73_AlarmDwn      AT %SUPPORTS_SYSTEM.boolean1.9;
SUPPORT_74_AlarmDwn      AT %SUPPORTS_SYSTEM.boolean1.10;
SUPPORT_75_AlarmDwn      AT %SUPPORTS_SYSTEM.boolean1.11;

ForceFlapUp_71           AT %SUPPORTS_SYSTEM.boolean1.14;
ForceFlapUp_72           AT %SUPPORTS_SYSTEM.boolean1.15;
ForceFlapUp_73           AT %SUPPORTS_SYSTEM.boolean1.16;
ForceFlapUp_74           AT %SUPPORTS_SYSTEM.boolean1.17;
ForceFlapUp_75           AT %SUPPORTS_SYSTEM.boolean1.18;

yActivatedCyCom_51       AT %SUPPORTS_SYSTEM.boolean1.19;
yActivatedCyCom_52       AT %SUPPORTS_SYSTEM.boolean1.20;
yActivatedCyCom_53       AT %SUPPORTS_SYSTEM.boolean1.21;
yActivatedCyCom_54       AT %SUPPORTS_SYSTEM.boolean1.22;
yActivatedCyCom_71       AT %SUPPORTS_SYSTEM.boolean1.23;
yActivatedCyCom_72       AT %SUPPORTS_SYSTEM.boolean1.24;
yActivatedCyCom_73       AT %SUPPORTS_SYSTEM.boolean1.25;
yActivatedCyCom_74       AT %SUPPORTS_SYSTEM.boolean1.26;
yActivatedCyCom_75       AT %SUPPORTS_SYSTEM.boolean1.27;

ISO_SupHor_Init          AT %SUPPORTS_SYSTEM.boolean1.30;
Opt_G0_SupDeCoup         AT %SUPPORTS_SYSTEM.boolean1.31;

SUPPORT_1_Quota          AT %SUPPORTS_SYSTEM.values0;
SUPPORT_2_Quota          AT %SUPPORTS_SYSTEM.values1;
SUPPORT_3_Quota          AT %SUPPORTS_SYSTEM.values2;
SUPPORT_4_Quota          AT %SUPPORTS_SYSTEM.values3;
SUPPORT_5_Quota          AT %SUPPORTS_SYSTEM.values4;
SUPPORT_6_Quota          AT %SUPPORTS_SYSTEM.values5;
SUPPORT_7_Quota          AT %SUPPORTS_SYSTEM.values6;

SUPPORT_1_Quota_Un       AT %SUPPORTS_SYSTEM.values14;
SUPPORT_2_Quota_Un       AT %SUPPORTS_SYSTEM.values15;
SUPPORT_3_Quota_Un       AT %SUPPORTS_SYSTEM.values16;
SUPPORT_4_Quota_Un       AT %SUPPORTS_SYSTEM.values17;
SUPPORT_5_Quota_Un       AT %SUPPORTS_SYSTEM.values18;
SUPPORT_6_Quota_Un       AT %SUPPORTS_SYSTEM.values19;
SUPPORT_7_Quota_Un       AT %SUPPORTS_SYSTEM.values20;

SUPPORTS_JogPlus         AT %SUPPORTS_SYSTEM.values7.0;
SUPPORTS_JogMinus        AT %SUPPORTS_SYSTEM.values7.1;
UNSUPPORTS_JogPlus       AT %SUPPORTS_SYSTEM.values7.2;
UNSUPPORTS_JogMinus      AT %SUPPORTS_SYSTEM.values7.3;

Hold_U_Z1SuppLoad        AT %SUPPORTS_SYSTEM.values8.5;
Hold_U_Z2SuppLoad        AT %SUPPORTS_SYSTEM.values8.6;
Hold_U_Z3SuppLoad        AT %SUPPORTS_SYSTEM.values8.7;
Hold_U_Z4SuppLoad        AT %SUPPORTS_SYSTEM.values8.8;
HoldZ1SuppLoad           AT %SUPPORTS_SYSTEM.values8.9;
HoldZ2SuppLoad           AT %SUPPORTS_SYSTEM.values8.10;
HoldZ3SuppLoad           AT %SUPPORTS_SYSTEM.values8.11;
HoldZ4SuppLoad           AT %SUPPORTS_SYSTEM.values8.12;
usrReqGrantSupLat1       AT %SUPPORTS_SYSTEM.values8.13;
usrReqGrantSupLat2       AT %SUPPORTS_SYSTEM.values8.14;
usrReqGrantSupLat3       AT %SUPPORTS_SYSTEM.values8.15;
usrReqGrantSupLat4       AT %SUPPORTS_SYSTEM.values8.16;
usrReqGrantSupL1         AT %SUPPORTS_SYSTEM.values8.17;
usrReqGrantSupL2         AT %SUPPORTS_SYSTEM.values8.18;
usrReqGrantSupL3         AT %SUPPORTS_SYSTEM.values8.19;
usrReqGrantSupL4         AT %SUPPORTS_SYSTEM.values8.20;
HoldO1SuppLoad_LAT       AT %SUPPORTS_SYSTEM.values8.21;
HoldO2SuppLoad_LAT       AT %SUPPORTS_SYSTEM.values8.22;
HoldO3SuppLoad_LAT       AT %SUPPORTS_SYSTEM.values8.23;
HoldO4SuppLoad_LAT       AT %SUPPORTS_SYSTEM.values8.24;
HoldO1Ax30_HOR           AT %SUPPORTS_SYSTEM.values8.25;
HoldO2Ax30_HOR           AT %SUPPORTS_SYSTEM.values8.26;
HoldO3Ax30_HOR           AT %SUPPORTS_SYSTEM.values8.27;
HoldO4Ax30_HOR           AT %SUPPORTS_SYSTEM.values8.28;

SUPPORTS_JogPlusLed      AT %SUPPORTS_SYSTEM.values9.0; 
SUPPORTS_JogMinusLed     AT %SUPPORTS_SYSTEM.values9.1; 
UNSUPPORTS_JogPlusLed    AT %SUPPORTS_SYSTEM.values9.2; 
UNSUPPORTS_JogMinusLed   AT %SUPPORTS_SYSTEM.values9.3; 

SUPPORTS_S1_Selected     AT %SUPPORTS_SYSTEM.values10.0;
SUPPORTS_S2_Selected     AT %SUPPORTS_SYSTEM.values10.1;
SUPPORTS_S3_Selected     AT %SUPPORTS_SYSTEM.values10.2;
SUPPORTS_S4_Selected     AT %SUPPORTS_SYSTEM.values10.3;
SUPPORTS_S5_Selected     AT %SUPPORTS_SYSTEM.values10.4;
SUPPORTS_S6_Selected     AT %SUPPORTS_SYSTEM.values10.5;
SUPPORTS_S7_Selected     AT %SUPPORTS_SYSTEM.values10.6;
SUPPORTS_C1_Selected     AT %SUPPORTS_SYSTEM.values10.7;
SUPPORTS_C2_Selected     AT %SUPPORTS_SYSTEM.values10.8;
SUPPORTS_C3_Selected     AT %SUPPORTS_SYSTEM.values10.9;
SUPPORTS_Za_Selected     AT %SUPPORTS_SYSTEM.values10.10;
SUPPORTS_R_Selected      AT %SUPPORTS_SYSTEM.values10.11;        
Use_Cent_Supports        AT %SUPPORTS_SYSTEM.values10.12;
SUPPORTS_S1_Selected_Un  AT %SUPPORTS_SYSTEM.values10.13;
SUPPORTS_S2_Selected_Un  AT %SUPPORTS_SYSTEM.values10.14;
SUPPORTS_S3_Selected_Un  AT %SUPPORTS_SYSTEM.values10.15;
SUPPORTS_S4_Selected_Un  AT %SUPPORTS_SYSTEM.values10.16;
SUPPORTS_S5_Selected_Un  AT %SUPPORTS_SYSTEM.values10.17;
SUPPORTS_S6_Selected_Un  AT %SUPPORTS_SYSTEM.values10.18;
SUPPORTS_S7_Selected_Un  AT %SUPPORTS_SYSTEM.values10.19;
SUPPORTS_C1_Selected_Un  AT %SUPPORTS_SYSTEM.values10.20;
SUPPORTS_C2_Selected_Un  AT %SUPPORTS_SYSTEM.values10.21;
SUPPORTS_C3_Selected_Un  AT %SUPPORTS_SYSTEM.values10.22;
SUPPORTS_S71_Selected    AT %SUPPORTS_SYSTEM.values10.23;
SUPPORTS_S72_Selected    AT %SUPPORTS_SYSTEM.values10.24;
SUPPORTS_S73_Selected    AT %SUPPORTS_SYSTEM.values10.25;
SUPPORTS_S74_Selected    AT %SUPPORTS_SYSTEM.values10.26;
SUPPORTS_S75_Selected    AT %SUPPORTS_SYSTEM.values10.27;

SUPPORTS_JogPlus_Enable  AT %SUPPORTS_SYSTEM.values11.0; 
SUPPORTS_JogMinus_Enable AT %SUPPORTS_SYSTEM.values11.1;
UNSUPPORTS_JogPlus_Enable  AT %SUPPORTS_SYSTEM.values11.2;
UNSUPPORTS_JogMinus_Enable AT %SUPPORTS_SYSTEM.values11.3;
 
SUPPORTS_Sel_S1_Enable    AT %SUPPORTS_SYSTEM.values12.0;
SUPPORTS_Sel_S2_Enable    AT %SUPPORTS_SYSTEM.values12.1;
SUPPORTS_Sel_S3_Enable    AT %SUPPORTS_SYSTEM.values12.2;
SUPPORTS_Sel_S4_Enable    AT %SUPPORTS_SYSTEM.values12.3;
SUPPORTS_Sel_S5_Enable    AT %SUPPORTS_SYSTEM.values12.4;
SUPPORTS_Sel_S6_Enable    AT %SUPPORTS_SYSTEM.values12.5;
SUPPORTS_Sel_S7_Enable    AT %SUPPORTS_SYSTEM.values12.6;

SUPPORTS_Sel_S1_Enable_Un AT %SUPPORTS_SYSTEM.values12.10;
SUPPORTS_Sel_S2_Enable_Un AT %SUPPORTS_SYSTEM.values12.11;
SUPPORTS_Sel_S3_Enable_Un AT %SUPPORTS_SYSTEM.values12.12;
SUPPORTS_Sel_S4_Enable_Un AT %SUPPORTS_SYSTEM.values12.13;
SUPPORTS_Sel_S5_Enable_Un AT %SUPPORTS_SYSTEM.values12.14;
SUPPORTS_Sel_S6_Enable_Un AT %SUPPORTS_SYSTEM.values12.15;
SUPPORTS_Sel_S7_Enable_Un AT %SUPPORTS_SYSTEM.values12.16;
SUPPORTS_Sel_C1_Enable_Un AT %SUPPORTS_SYSTEM.values12.17;
SUPPORTS_Sel_C2_Enable_Un AT %SUPPORTS_SYSTEM.values12.18;
SUPPORTS_Sel_C3_Enable_Un AT %SUPPORTS_SYSTEM.values12.19;

* Gui var not for plc
Btn_Load_GearON_2        AT %SUPPORTS_SYSTEM.values13.0;
Btn_Load_ReqPos_2        AT %SUPPORTS_SYSTEM.values13.1;
Btn_Load_GearON_3        AT %SUPPORTS_SYSTEM.values13.2;
Btn_Load_ReqPos_3        AT %SUPPORTS_SYSTEM.values13.3;
OptEnabAutoParkLoad      AT %SUPPORTS_SYSTEM.values13.4;
ParkAllLoadSuppCmd       AT %SUPPORTS_SYSTEM.values13.5;
Btn_Load_GearON_4        AT %SUPPORTS_SYSTEM.values13.6;
Btn_Load_ReqPos_4        AT %SUPPORTS_SYSTEM.values13.7;
ParkAllLoadSuppCmd_LAT    AT %SUPPORTS_SYSTEM.values13.8;
Btn_Load_GearON_2_LAT     AT %SUPPORTS_SYSTEM.values13.9;
Btn_Load_ReqPos_2_LAT     AT %SUPPORTS_SYSTEM.values13.10;
Btn_Load_GearON_3_LAT     AT %SUPPORTS_SYSTEM.values13.11;
Btn_Load_ReqPos_3_LAT     AT %SUPPORTS_SYSTEM.values13.12;
Btn_Load_GearON_4_LAT     AT %SUPPORTS_SYSTEM.values13.13;
Btn_Load_ReqPos_4_LAT     AT %SUPPORTS_SYSTEM.values13.14;
OptForceLatOpen           AT %SUPPORTS_SYSTEM.values13.15;
NoRoundLatOpen           AT %SUPPORTS_SYSTEM.values13.16;

TimeOutPistonDown        AT %SUPPORTS_SYSTEM.values21;
MaxPistionDownDis        AT %SUPPORTS_SYSTEM.values22;
CyComLenLoader           AT %SUPPORTS_SYSTEM.values23;
CyComLenUnloader         AT %SUPPORTS_SYSTEM.values24;
EnCyComTubeDim           AT %SUPPORTS_SYSTEM.values25;
EnCyComTubeDimUn         AT %SUPPORTS_SYSTEM.values26;

BtnGuiVshapeZ1_UP        AT %SUPPORTS_SYSTEM.values27.0;
BtnGuiVshapeZ2_UP        AT %SUPPORTS_SYSTEM.values27.1;
BtnGuiVshapeZ3_UP        AT %SUPPORTS_SYSTEM.values27.2;
BtnGuiVshapeZ4_UP        AT %SUPPORTS_SYSTEM.values27.3;
BtnGuiVshapeZ1_Dwn       AT %SUPPORTS_SYSTEM.values27.4;
BtnGuiVshapeZ2_Dwn       AT %SUPPORTS_SYSTEM.values27.5;
BtnGuiVshapeZ3_Dwn       AT %SUPPORTS_SYSTEM.values27.6;
BtnGuiVshapeZ4_Dwn       AT %SUPPORTS_SYSTEM.values27.7;
BtnGuiVshapeAllZ_UP      AT %SUPPORTS_SYSTEM.values27.8;
BtnGuiVshapeAllZ_Dwn     AT %SUPPORTS_SYSTEM.values27.9;

SUPPORTS_Max_Measure     AT %SUPPORTS_SYSTEM.values31;

***********************************************************************************************
Wx_PGM_USED              AT %PROG_ONE.values10; 
LoaderProcess            AT %PROG_ONE.values11; 

Current_Tube_Format      AT %PROG_ONE.values3;           * Current tube format  
Max_Tube_Format          AT %PROG_ONE.values4;           * Maximum tube format  
Use_Cent_Sup             AT %PROG_ONE.values5.1;
I_Tube_Inside            AT %PROG_ONE.boolean.11;
Tray_Active              AT %PROG_ONE.boolean.12;
Use_Tray_Final_Unload    AT %PROG_ONE.boolean.16;
Back_Final_Unload        AT %PROG_ONE.boolean.17;

LoadinInProgres          AT %GENERAL_SETUP.values0;
UnloadingInProgres       AT %GENERAL_SETUP.values1;  

IsoCommandsUsr           AT %GENERAL_SETUP.boolean0;

U_axisHeading            AT %LsIso54;                   * Position of U during heading
X_AxisHeading            AT %LsIso55;                   * Distance tube for X

***********************************************************************************************
* Loader

BtnGuiM1M2M3_Open        AT %LOADER.boolean.0;
BtnGuiM1M2M3_Close       AT %LOADER.boolean.1;
BtnGuiCentering_Open_1   AT %LOADER.boolean.2;
BtnGuiCentering_Close_1  AT %LOADER.boolean.3;
BtnGui_G_Up              AT %LOADER.boolean.4;
BtnGui_G_Dwn             AT %LOADER.boolean.5;
BtnGui_F_Forw            AT %LOADER.boolean.6;
BtnGui_F_Backw           AT %LOADER.boolean.7;
   
BtnGuiChainForw          AT %LOADER.boolean.8;
BtnGuiChainBackw         AT %LOADER.boolean.9;
BtnGuiCentering_Open_2   AT %LOADER.boolean.10;
BtnGuiCentering_Close_2  AT %LOADER.boolean.11;
BtnGuiCentering_Open_3   AT %LOADER.boolean.12;
BtnGuiCentering_Close_3  AT %LOADER.boolean.13;
BtnGuiCentering_Open_4   AT %LOADER.boolean.14;
BtnGuiCentering_Close_4  AT %LOADER.boolean.15;
Force_Open_Center_1      AT %LOADER.boolean.16;
Force_Open_Center_2      AT %LOADER.boolean.17;
Force_Open_Center_3      AT %LOADER.boolean.18;
Force_Open_Center_4      AT %LOADER.boolean.19;
BtnGuiCentering_Open_All AT %LOADER.boolean.20;
BtnGuiCentering_Cls_All  AT %LOADER.boolean.21;
BtnClrLoaderBreakPoint   AT %LOADER.boolean.22;

ISO_Loader_Ack           AT %LOADER.boolean.30;
ISO_Loader_Req           AT %LOADER.boolean.31;
 
WaitLoadM                AT %LOADER.boolean1.0; 

TubeLength               AT %LOADER.values0;    * length from server
Offset_U                 AT %LOADER.values1;
Offset_U_Mobile          AT %LOADER.values2;
Offset_U2                AT %LOADER.values3;
TubeLengthShifted        AT %LOADER.values4;

*                        AT %LOADER.values5;
*                        AT %LOADER.values6;
*                        AT %LOADER.values7;
ThreshLoadingSupDown     AT %LOADER.values8;  * Threshold for suport down during loading Danger parameter

ManLoadQtaU              AT %LOADER.values10;

ParkingPositionLoad      AT %LOADER.values15;
ParkingPosLoad_lat       AT %LOADER.values16;

UpSupVoffset             AT %LOADER.values20;


********************************************************************
********************* STORE ************************************    
                                                                    
STORE_Empty                           AT %STORE.boolean.0;
TubeOnGworklist                       AT %STORE.boolean.1;   
DisAbleTubeOnC                        AT %STORE.boolean.2;  
               
***                                                                    
***STORE_JogPlus                      AT %STORE.values0.0;             
***STORE_JogMinus                     AT %STORE.values0.1;             
***                                                                    
***STORE_Sel_Sec                        AT %STORE.values1.0;           
***STORE_Sel_Anti                       AT %STORE.values1.1;           
***STORE_Sel_Conv                       AT %STORE.values1.2;           
***STORE_Sel_Piston                     AT %STORE.values1.3;           
***STORE_Sel_Belts                      AT %STORE.values1.4;           
***STORE_Sel_Rail                       AT %STORE.values1.5;           
***STORE_Sel_UseStore                   AT %STORE.values1.6;    
***
***STORE_Sel_Piston_1                     AT %STORE.values1.7; 
***STORE_Sel_Piston_2                    AT %STORE.values1.8; 
***STORE_Sel_Piston_3                     AT %STORE.values1.9; 
***STORE_Sel_Piston_4                     AT %STORE.values1.10; 
***STORE_Sel_Piston_5                     AT %STORE.values1.11; 
***STORE_Sel_Piston_6                     AT %STORE.values1.12; 
***STORE_Sel_Piston_7                     AT %STORE.values1.13;         
***                                                                    
***STORE_JogPlusLed                   AT %STORE.values2.0;             
***STORE_JogMinusLed                  AT %STORE.values2.1;             
***                                                                    
***STORE_Sec_Selected                  AT %STORE.values3.0;            
***STORE_Anti_Selected                AT %STORE.values3.1;             
***STORE_Conv_Selected                 AT %STORE.values3.2;            
***STORE_Piston_Selected               AT %STORE.values3.3;            
***STORE_Belts_Selected               AT %STORE.values3.4;             
***STORE_Rail_Selected                AT %STORE.values3.5;             
***STORE_UseStore                       AT %STORE.values3.6;  
***STORE_Piston_Selected_1            AT %STORE.values3.7;     
***STORE_Piston_Selected_2            AT %STORE.values3.8;
***STORE_Piston_Selected_3            AT %STORE.values3.9;
***STORE_Piston_Selected_4            AT %STORE.values3.10;
***STORE_Piston_Selected_5            AT %STORE.values3.11;
***STORE_Piston_Selected_6            AT %STORE.values3.12;
***STORE_Piston_Selected_7            AT %STORE.values3.13;
***
***                                                                    
***STORE_JogPlus_Enable              AT %STORE.values4.0;              
***STORE_JogMinus_Enable              AT %STORE.values4.1;             
***                                                                    
***STORE_Sel_Sec_Enable               AT %STORE.values5.0;             
***STORE_Sel_Anti_Enable              AT %STORE.values5.1;             
***STORE_Sel_Conv_Enable              AT %STORE.values5.2;             
***STORE_Sel_Piston_Enable            AT %STORE.values5.3;             
***STORE_Sel_Belts_Enable            AT %STORE.values5.4;              
***STORE_Sel_Rail_Enable             AT %STORE.values5.5;              
***STORE_UseStore_Enable              AT %STORE.values5.6;     
***
***STORE_Sel_Piston_1_Enable         AT %STORE.values5.7;       
***STORE_Sel_Piston_2_Enable         AT %STORE.values5.8; 
***STORE_Sel_Piston_3_Enable         AT %STORE.values5.9; 
***STORE_Sel_Piston_4_Enable         AT %STORE.values5.10; 
***STORE_Sel_Piston_5_Enable         AT %STORE.values5.11; 
***STORE_Sel_Piston_6_Enable         AT %STORE.values5.12;   
***STORE_Sel_Piston_7_Enable         AT %STORE.values5.13; 
***        
****                                                                   
***STORE_Delay_Piston                 AT %STORE.values8;               
****                               AT %STORE.values9;                  
***                                                                    
***STORE_Piston_Up                     AT %STORE.values10.0;           
***STORE_Piston_Dwn                    AT %STORE.values10.1;          
***
***STORE_Piston_Up_1                   AT %STORE.values20.0;           
***STORE_Piston_Dwn_1                  AT %STORE.values20.1;  
***
***STORE_Piston_Up_2                   AT %STORE.values21.0;           
***STORE_Piston_Dwn_2                  AT %STORE.values21.1;  
***
***STORE_Piston_Up_3                   AT %STORE.values22.0;           
***STORE_Piston_Dwn_3                  AT %STORE.values22.1;  
***
***STORE_Piston_Up_4                   AT %STORE.values23.0;           
***STORE_Piston_Dwn_4                  AT %STORE.values23.1;  
***
***STORE_Piston_Up_5                   AT %STORE.values24.0;           
***STORE_Piston_Dwn_5                  AT %STORE.values24.1;  
***
***STORE_Piston_Up_6                   AT %STORE.values25.0;           
***STORE_Piston_Dwn_6                  AT %STORE.values25.1;  
***
***STORE_Piston_Up_7                   AT %STORE.values26.0;           
***STORE_Piston_Dwn_7                  AT %STORE.values26.1;   
***                                                                    
***STORE_Belts_Up                      AT %STORE.values11.0;           
***STORE_Belts_Dwn                     AT %STORE.values11.1;           
***                                                                    
***STORE_Rail_Fwd                      AT %STORE.values12.0;           
***STORE_Rail_Bwd                      AT %STORE.values12.1;           
***                                                                    
***STORE_Tolerance_Anti                AT %STORE.values14;             
***STORE_Tolerance_Sec                 AT %STORE.values15;             
***                                                                    
***STORE_AntiOverlap_Delay             AT %STORE.values16;              
***STORE_Tube_Time                     AT %STORE.values17;            
***STORE_Time_Unblock_Conv             AT %STORE.values18;              
***STORE_Time_Unblock_Belts            AT %STORE.values19;              
********************************************************************


BtnGui_B1_B2_Open                      AT %UNLOADER.boolean.0;
BtnGui_B1_B2_Close                     AT %UNLOADER.boolean.1;



TubeLengthInMach                       AT %UNLOADER.values0;

* No write from plc

ResetTubeInMac                         AT %UNLOADER.values1.0;
EnableUnloadFollower                   AT %UNLOADER.values1.1;
Flap71JogHome                          AT %UNLOADER.values1.2;
Flap71JogIncline                       AT %UNLOADER.values1.3;
Flap72JogHome                          AT %UNLOADER.values1.4;
Flap72JogIncline                       AT %UNLOADER.values1.5;
*EnableFollower                        AT %UNLOADER.values1.6;
ForceFlapUpAtBegin                     AT %UNLOADER.values1.7;
ForceGarbOpen                          AT %UNLOADER.values1.8;  
DisUnloadCon_V                         AT %UNLOADER.values1.9;  

BtnGuiGoInMach1                        AT %UNLOADER.values1.10;
BtnGuiGoInMach2                        AT %UNLOADER.values1.11;
BtnGuiToUnload1                        AT %UNLOADER.values1.12;
BtnGuiToUnload2                        AT %UNLOADER.values1.13;
Dis2ndFlapTar                          AT %UNLOADER.values1.14;
EnVSuppFeedRec                         AT %UNLOADER.values1.15;
EnFineScrapUnload                      AT %UNLOADER.values1.16;
EnSingleSpindle                        AT %UNLOADER.values1.17;  * Single spindle mode to cut one short piece 

* No write from plc

PosUHeading                            AT %UNLOADER.values2;
PosXHeading                            AT %UNLOADER.values3;

ThreshSuppUp                           AT %UNLOADER.values4;

ParkingPositionUnload                  AT %UNLOADER.values5;
OffsetUnloadSupport                    AT %UNLOADER.values6;
TubeLimitLength                        AT %UNLOADER.values7;
MaxTubeLengthDiscarge                  AT %UNLOADER.values8;

UnloadPosInMach                        AT %UNLOADER.values9;
UnLoadFlapOnOffTime                    AT %UNLOADER.values10;
OffsetUnSuppDwn_71                     AT %UNLOADER.values11;
OffsetUnSuppDwn_72                     AT %UNLOADER.values12;
VSuppFeedReduction                     AT %UNLOADER.values13;
Flap_O1_InMac_Pos                      AT %UNLOADER.values14;
OffsetUnSuppDwn_73                     AT %UNLOADER.values15;
OffsetUnSuppDwn_74                     AT %UNLOADER.values16;
OffsetUnSuppDwn_75                     AT %UNLOADER.values17;
UnloadSupQta                           AT %UNLOADER.values18;

Supports_Un_Up                         AT %UNLOADER.boolean.0; * at least one support is up


***********************************************************************
********************* MACHINE TIMES *****************************

**** AT %TIMES.boolean.0; * Partial Start Button (State)
**** AT %TIMES.boolean.1; * Partial Stop Button (State)
**** AT %TIMES.boolean.2; * Partial Reset Button (Trigger)
****
**** AT %TIMES.boolean.3; * Chrono Start Button (State)
**** AT %TIMES.boolean.4; * Chrono Stop Button (State)
**** AT %TIMES.boolean.5; * Chrono Reset Button (State)
****
**** AT %TIMES.values0; * Machine Times
**** AT %TIMES.values1; * Machine Times
**** AT %TIMES.values2; * Machine Times
****
**** AT %TIMES.values3; * Program Times
**** AT %TIMES.values4; * Program Times
**** AT %TIMES.values5; * Program Times
****
**** AT %TIMES.values6; * Cutting Times
**** AT %TIMES.values7; * Cutting Times
**** AT %TIMES.values8; * Cutting Times
****
**** AT %TIMES.values9; * Partial Machine Times
**** AT %TIMES.values10; * Partial Machine Times
**** AT %TIMES.values11; * Partial Machine Times
****
**** AT %TIMES.values12; * Partial Program Times
**** AT %TIMES.values13; * Partial Program Times
**** AT %TIMES.values14; * Partial Program Times
****
**** AT %TIMES.values15; * Partial Cutting Times
**** AT %TIMES.values16; * Partial Cutting Times
**** AT %TIMES.values17; * Partial Cutting Times
****
**** AT %TIMES.values18; * General Purpose Chronometer Times
**** AT %TIMES.values19; * General Purpose Chronometer Times
**** AT %TIMES.values20; * General Purpose Chronometer Times
****
**** AT %TIMES.values21; * Current Program Times
**** AT %TIMES.values22; * Current Program Times
**** AT %TIMES.values23; * Current Program Times
****
**** AT %TIMES.values24; * Current Cutting Times
**** AT %TIMES.values25; * Current Cutting Times
**** AT %TIMES.values26; * Current Cutting Times
****
**** AT %TIMES.values27; * General Throughput 
**** AT %TIMES.values28; * Water Throughput 
**** AT %TIMES.values29; * Partial Throughput 
**** AT %TIMES.values30; * Partial Water Throughput  

*****************************************************************
******************** AUTOMATIC LUBRICATOR ***********************

*** LUB_1_Hours_Frequency              AT %LUB_ONE.values0;
*** LUB_1_Hours_Last_Operation         AT %LUB_ONE.values1;
*** LUB_1_Time_Lubrication             AT %LUB_ONE.values2;
*** LUB_1_Time_Wait                    AT %LUB_ONE.values3;
*** LUB_1_Num_Lub                      AT %LUB_ONE.values4;
*** LUB_1_TimeOutPres                  AT %LUB_ONE.values5;

*******************************************************************
*** MeasureDone                          AT %cn7.vl62; 
Btn_GearON_2             AT %UNLOADER.boolean.1;
Btn_ReqPos_2             AT %UNLOADER.boolean.2;
Btn_GearON_3             AT %UNLOADER.boolean.3;
Btn_ReqPos_3             AT %UNLOADER.boolean.4;
OptEnabAutoPark          AT %UNLOADER.boolean.5;
ParkAllCmd               AT %UNLOADER.boolean.6;
InclineFlapL1_ISO        AT %UNLOADER.boolean.7;
InclineFlapL2_ISO        AT %UNLOADER.boolean.8;
InclineFlapL3_ISO        AT %UNLOADER.boolean.9;
InclineFlapL4_ISO        AT %UNLOADER.boolean.10;
InclineFlapL5_ISO        AT %UNLOADER.boolean.11;
OptSemiUnload            AT %UNLOADER.boolean.12;
OptJogNoAutoHorPrk       AT %UNLOADER.boolean.13;
*                        AT %UNLOADER.boolean.14;
FlapL3JogHome            AT %UNLOADER.boolean.15;
FlapL3JogIncline         AT %UNLOADER.boolean.16;
FlapL4JogHome            AT %UNLOADER.boolean.17;
FlapL4JogIncline         AT %UNLOADER.boolean.18;
FlapL5JogHome            AT %UNLOADER.boolean.19;
FlapL5JogIncline         AT %UNLOADER.boolean.20;
AllFlapsHome             AT %UNLOADER.boolean.21;
AllFlapsIncline          AT %UNLOADER.boolean.22;
ForceFlapUpZone          AT %UNLOADER.boolean.23;
HoldFlapInclined         AT %UNLOADER.boolean.24;

U_Posit_Z1_Down          AT %UNLOADER.values19;
U_Posit_Z2_Down          AT %UNLOADER.values20;
U_Posit_Z3_Down          AT %UNLOADER.values21;
U_Posit_Z4_Down          AT %UNLOADER.values22;
U_Posit_Z5_Down          AT %UNLOADER.values23;

OffSetSlaveClamp         AT %UNLOADER.values24;
ChuckOpenCloseTimeMain   AT %UNLOADER.values25;
FlapWidth                AT %UNLOADER.values26;

HoldZ1Supp               AT %UNLOADER.values27.0;
HoldZ2Supp               AT %UNLOADER.values27.1;
HoldZ3Supp               AT %UNLOADER.values27.2;
HoldZ4Supp               AT %UNLOADER.values27.3;
HoldZ5Supp               AT %UNLOADER.values27.4;

Hold_U_Z1Supp            AT %UNLOADER.values27.5;
Hold_U_Z2Supp            AT %UNLOADER.values27.6;
Hold_U_Z3Supp            AT %UNLOADER.values27.7;
Hold_U_Z4Supp            AT %UNLOADER.values27.8;
Hold_U_Z5Supp            AT %UNLOADER.values27.9;

Z1Supp_71InParkPos       AT %UNLOADER.values27.10;
Z2Supp_72InParkPos       AT %UNLOADER.values27.11;
Z3Supp_73InParkPos       AT %UNLOADER.values27.12;
Z4Supp_74InParkPos       AT %UNLOADER.values27.13;
Z5Supp_75InParkPos       AT %UNLOADER.values27.14;

SUP72_xParkCmd           AT %UNLOADER.values27.15;
SUP71_xParkCmd           AT %UNLOADER.values27.16;
SUP73_xParkCmd           AT %UNLOADER.values27.17;
SUP74_xParkCmd           AT %UNLOADER.values27.18;
SUP75_xParkCmd           AT %UNLOADER.values27.19;

MinPieceUnload           AT %UNLOADER.values28;
ChuckOpenCloseTime       AT %UNLOADER.values29;

usrReqGrantSup1          AT %UNLOADER.values30.0;
usrReqGrantSup2          AT %UNLOADER.values30.1;
usrReqGrantSup3          AT %UNLOADER.values30.2;
usrReqGrantSup4          AT %UNLOADER.values30.3;
usrReqGrantSup5          AT %UNLOADER.values30.4;

* Horizontal Unloading Followers Related --->

usrReqGrantSupHorAx85    AT %UNLOADER.values31.0;
usrReqGrantSupHorAx86    AT %UNLOADER.values31.1;
usrReqGrantSupHorAx87    AT %UNLOADER.values31.2;
usrReqGrantSupHorAx88    AT %UNLOADER.values31.3;
*                        AT %UNLOADER.values31.4;
*                        AT %UNLOADER.values31.5;
HoldSuppUn_HorAx85       AT %UNLOADER.values31.6;
HoldSuppUn_HorAx86       AT %UNLOADER.values31.7;
HoldSuppUn_HorAx87       AT %UNLOADER.values31.8;
HoldSuppUn_HorAx88       AT %UNLOADER.values31.9;
*                        AT %UNLOADER.values31.10;
*                        AT %UNLOADER.values31.11;
Hold_U_SupUnHorAx85      AT %UNLOADER.values31.12;
Hold_U_SupUnHorAx86      AT %UNLOADER.values31.13;
Hold_U_SupUnHorAx87      AT %UNLOADER.values31.14;
Hold_U_SupUnHorAx88      AT %UNLOADER.values31.15;
*                        AT %UNLOADER.values31.16;
*                        AT %UNLOADER.values31.17;
Hor_Ax72_yVerForceDwn    AT %UNLOADER.values31.18;
Hor_Ax73_yVerForceDwn    AT %UNLOADER.values31.19;
Hor_Ax74_yVerForceDwn    AT %UNLOADER.values31.20;
Hor_Ax75_yVerForceDwn    AT %UNLOADER.values31.21;

UnSupHorSafeZone         AT %UNLOADER.values32;
VShapeHorOffest          AT %UNLOADER.values33;

CyComOftAx51             AT %UNLOADER.values34;
CyComOftAx52             AT %UNLOADER.values35;
CyComOftAx53             AT %UNLOADER.values36;
CyComOftAx54             AT %UNLOADER.values37;				 
CyComOftAx55             AT %UNLOADER.values38;

CyComOftAx71             AT %UNLOADER.values39;
CyComOftAx72             AT %UNLOADER.values40;
CyComOftAx73             AT %UNLOADER.values41;
CyComOftAx74             AT %UNLOADER.values42;
CyComOftAx75             AT %UNLOADER.values43;

BLDFEED_72_85_Percent    AT %UNLOADER.values44;
BLDFEED_73_86_Percent    AT %UNLOADER.values45;
BLDFEED_74_87_Percent    AT %UNLOADER.values46;
BLDFEED_75_88_Percent    AT %UNLOADER.values47;
HorUnloadingFeed         AT %UNLOADER.values48;

* Horizontal Unloading Followers Related (END) <---


