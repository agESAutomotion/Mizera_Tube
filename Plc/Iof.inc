*******************************************************************************
*   SCHEDA n.1 SU CNC (16+16 INGRESSI)
*******************************************************************************

** 16 input - On CNC board position 1 - 1 

I_FC_Y1_POS                  AT %IW0.0;        * ax28 Y axis
I_FC_Y1_NEG                  AT %IW0.1;        * ax28 Y axis
I_FC_UV_NEG                  AT %IW0.2;        * ax30 X axis
I_FC_UV_POS                  AT %IW0.3;        * ax30 X axis
I_FC_W_NEG                   AT %IW0.4;        * ax36 X3
I_FC_W_POS                   AT %IW0.5;        * ax36 X3
I_FC_V_NEG                   AT %IW0.6;        * ax31 X1
I_FC_V_POS                   AT %IW0.7;        * ax31 X1
*                            AT %IW0.8; 
*                            AT %IW0.9; 
*                            AT %IW0.10; 
I_FC_Z1_NEG                  AT %IW0.11;       * ax29 Z axis
I_FC_Z1_POS                  AT %IW0.12;       * ax29 Z axis
I_FC_Z4_SUP_POS              AT %IW0.13;       * ax54 Z4 axis limit switch
I_FC_Z4_SUP_NEG              AT %IW0.14;       * ax54 Z4 axis limit switch
I_FC_Z3_SUP_NEG              AT %IW0.15;       * ax53 Z3 axis limit switch

** 16 input - On CNC board position 1 - 2

I_FC_Z3_SUP_POS              AT %PLCFLAGS.1;   * ax53 Z3 axis limit switch
I_FC_Z2_SUP_NEG              AT %IW1.1;        * ax51 Z2 axis limit switch
I_FC_Z2_SUP_POS              AT %IW1.2;        * ax51 Z2 axis limit switch
I_FC_Z1_SUP_NEG              AT %IW1.3;        * ax52 Z1 axis limit switch
I_FC_Z1_SUP_POS              AT %IW1.4;        * ax52 Z1 axis limit switch
I_FC_L1_POS                  AT %IW1.5;        * ax71 L1 axis limit switch
I_FC_L1_NEG                  AT %IW1.6;        * ax71 L1 axis limit switch
I_FC_L2_POS                  AT %IW1.7;        * ax72 L2 axis limit switch
I_FC_L2_NEG                  AT %IW1.8;        * ax72 L2 axis limit switch
I_FC_L3_POS                  AT %IW1.9;        * ax73 L3 axis limit switch
I_FC_L3_NEG                  AT %IW1.10;       * ax73 L3 axis limit switch
I_Bin_DoorDwn                AT %IW1.11;
I_Bin_DoorUp                 AT %IW1.12;
I_L1_Down                    AT %IW1.13;       * Safety reason to ensure support 1 fully down
I_Flap_Home                  AT %IW1.14;
I_FlapL2_Home                AT %IW1.15;       

*******************************************************************************
*   SCHEDA n.2 SU CNC (16+16 INGRESSI)
*******************************************************************************

** 16 input - On CNC board position 2 - 1 

I_FlapL3_Home                AT %IW4.0;
I_CenterZ4_Open              AT %IW4.1;
I_CenterZ3_Open              AT %IW4.2;
I_CenterZ2_Open              AT %IW4.3;
I_CenterZ1_Open              AT %IW4.4;
I_EMERGENZA                  AT %IW4.5; 
I_LSR_ERROR                  AT %IW4.6;  
I_CHILLER_ERR                AT %IW4.7;
I_LUB_LOW_LEVEL              AT %IW4.8;
I_SAFETY_DOOR                AT %IW4.9;
I_SAFETY_CabDoor             AT %IW4.10;
I_Photocell_Load             AT %IW4.11;
I_Photocell_Unload           AT %IW4.12;
I_PRESSIONE_ARIA             AT %PLCFLAGS.1; *%IW4.13;            
I_PRESSIONE_O2               AT %IW4.14;
I_PRESSIONE_N2               AT %IW4.15;

** 16 input - On CNC board position 2 - 2 

I_TubeMobile_Spindle         AT %IW5.0;
I_TubeFix_Spindle            AT %IW5.1;
I_Detec_Weld                 AT %IW5.2;
I_Pic_Weld                   AT %IW5.3;
I_Loader_Alarm               AT %IW5.4;
I_M1_M2_M3_ALARM             AT %IW5.5;
I_CylinderDwn_G_1            AT %IW5.6;
I_CylinderDwn_G_2            AT %IW5.7;
I_CylinderDwn_G_3            AT %IW5.8;
I_CylinderUp_G_1             AT %IW5.9;
I_CylinderUp_G_2             AT %IW5.10;
I_CylinderUp_G_3             AT %IW5.11;
I_CylinderBack_F_1           AT %IW5.12; 
I_CylinderBack_F_2           AT %IW5.13; 
I_CylinderBack_F_3           AT %IW5.14;
I_CylinderForw_F_1           AT %IW5.15;

*******************************************************************************
* BRD.007.461 INPUTS (NODE 1)
*******************************************************************************
I_CylinderForw_F_2           AT %IW610.0;
I_CylinderForw_F_3           AT %IW610.1;
I_TubeOnChain_D_1            AT %IW610.2;
I_TubeOnChain_D_2            AT %IW610.3;
*                            AT %IW610.4;
I_TubeOnSup_C_1              AT %IW610.5;  
I_TubeOnSup_C_2              AT %IW610.6;
I_M1_M2_M3_OPEN              AT %IW610.7;
I_M1_M2_M3_CLOSE             AT %IW610.8; 
I_Loader_Slow                AT %IW610.9;
I_Ready_To_Load              AT %PLCFLAGS.1; 
*                            AT %IW610.10;
*                            AT %IW610.11;
*                            AT %IW610.12;
I_TubeOnChain_D_3            AT %IW610.13;
*                            AT %IW610.14;
*                            AT %IW610.15;

*******************************************************************************
* BRD.007.461 INPUTS (NODE 2)
*******************************************************************************
*                            AT %IW611.0;
*                            AT %IW611.1;
*                            AT %IW611.2;
*                            AT %IW611.3;
*                            AT %IW611.4;
*                            AT %IW611.5;
*                            AT %IW611.6;          
I_PRESSIONE_ARIA_IMPIANTO    AT %PLCFLAGS.1;
I_AntiColide_1               AT %IW611.8;
I_AntiColide_2               AT %IW611.9;  
I_FC_W1_POS                  AT %IW611.10;        *ax9 FOCAL axis
I_FC_W1_NEG                  AT %IW611.11;       *ax9 FOCAL axis
I_HomeChain_E                AT %IW611.12;      
*                            AT %IW611.13;      
*                            AT %IW611.14;      
*                            AT %IW611.15;      

IA_Emower_cap                AT %ecat_ring0.EcatNode16.EcMDPIn0;

*******************************************************************************
*   SCHEDA n.1 SU CNC (16+16 USCITE)
*******************************************************************************

** 16 outputs - On CNC board position 1 - 1

O_EVGAS_OSSIGENO             AT %QW2.0;  *Out O2
O_EVGAS_OAZOTO               AT %QW2.1;  *Out N2
O_EVGAS_ARIA                 AT %QW2.2;  *Out AIR
O_LAMPADA_ROSSA              AT %QW2.3;  
O_LAMPADA_ARANCIONE          AT %QW2.4;  
O_LAMPADA_VERDE              AT %QW2.5; 
O_Mobile_Spindle_Slave_Close AT %QW2.6;
O_Mobile_Spindle_Slave_Open  AT %QW2.7;
O_SpindleFix_Close           AT %QW2.8;
O_SpindleFix_Open            AT %QW2.9;
O_Mobile_Spindle_Close       AT %QW2.10;
O_EV_AUX_GAS                 AT %QW2.11;
O_EV_ARIA_UGELLO             AT %QW2.12;
*                            AT %QW2.13;
O_Centering_Close_4          AT %QW2.14;
O_Centering_Close_3          AT %QW2.15;

** 16 outputs - On CNC board position 1 - 2

O_Centering_Close_2          AT %QW3.0;
O_Centering_Close_1          AT %QW3.1;
O_Flap_Incline               AT %QW3.2;
O_FlapL2_Incline             AT %QW3.3;
O_FlapL3_Incline             AT %QW3.4;
O_Bin_Door                   AT %QW3.5;  *(1 = Bin Door Up )
*                            AT %QW3.6;
START_ASPIRATORE             AT %QW3.7;  
O_LSR_LASER_ON               AT %QW3.8;  
O_LSR_GUIDE_ON               AT %QW3.9;  * Out A5 
O_LSR_REMOTE_START           AT %QW3.10;
O_Chain_Forw_D               AT %QW3.11;
O_Chain_Back_D               AT %QW3.12;
O_CylinderBack_F             AT %QW3.13; 
O_CylinderUp_G               AT %QW3.14;
O_CylinderForw_F             AT %QW3.15; * Loader Side

*******************************************************************************
*   SCHEDA n.2 SU CNC (16+16 USCITE)
*******************************************************************************
O_M1_M2_M3_Close             AT %QW6.0;
O_M1_M2_M3_Open              AT %QW6.1;
O_Arm_Fast                   AT %QW6.2;
O_Arm_Slow                   AT %QW6.3;
O_Rst_InverterAlm            AT %QW6.4;
O_Detec_Weld_Piston          AT %QW6.5;
O_Detec_Weld                 AT %QW6.6;
O_WeldDetec_En               AT %M5.0;
*                            AT %QW6.7;
*                            AT %QW6.8;
*                            AT %QW6.9;
*                            AT %QW6.10;
*                            AT %QW6.11;
*                            AT %QW6.12;
*                            AT %QW6.13;
*                            AT %QW6.14;
*                            AT %QW6.15;

*******************************************************************************
* BRD.007.461 INPUTS (NODE 2)
*******************************************************************************
O_Mobile_Spindle_Open        AT %M5.0;
O_LUB                        AT %M5.0;
O_Centering_Open_1           AT %M5.0;
O_Centering_Open_2           AT %M5.0;
O_Centering_Open_3           AT %M5.0;
O_Centering_Open_4           AT %M5.0;

*******************************************************************************
*   PWM BOARD I/O
******************************************************************************* 
I_IA_Feedback_GAS_OUT        AT %IW21;  
I_IA_Feedback_GAS_OUT_O2     AT %IW22; 

O_OA_GAS                     AT %QW926;
O_OA_GAS_2                   AT %QW927; *Gas pro. valve for N2

****************************************************************************************************
****************************************************************************************************
****************************************************************************************************
* END_MACHINE END_MACHINE END_MACHINE END_MACHINE END_MACHINE END_MACHINE END_MACHINE END_MACHINE  *
****************************************************************************************************
****************************************************************************************************
****************************************************************************************************

*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
*//////////////////////////////////////////////////////////////////////////////////////////////////*
I_FC_O1_POS               AT %PLCFLAGS.1;        * ax82 O1 axis limit switch
I_FC_O1_NEG               AT %PLCFLAGS.1;        * ax82 O1 axis limit switch
I_FC_O2_POS               AT %PLCFLAGS.1;        * ax81 Z4 axis limit switch
I_FC_O2_NEG               AT %PLCFLAGS.1;        * ax81 Z4 axis limit switch
I_FC_O3_POS               AT %PLCFLAGS.1;        * ax83 O1 axis limit switch
I_FC_O3_NEG               AT %PLCFLAGS.1;        * ax83 O1 axis limit switch
I_FC_O4_POS               AT %PLCFLAGS.1;        * ax84 Z4 axis limit switch
I_FC_O4_NEG               AT %PLCFLAGS.1;        * ax84 Z4 axis limit switch

I_FC_B1_POS               AT %PLCFLAGS.1; 
I_FC_B1_NEG               AT %PLCFLAGS.1; 
I_FC_L4_NEG               AT %PLCFLAGS.1;        * ax74 L4 axis limit switch
I_FC_L4_POS               AT %PLCFLAGS.1;        * ax74 L4 axis limit switch
						  
I_LSR_READY               AT %PLCFLAGS.1;   
I_FW_READY_nLight         AT %PLCFLAGS.1; 
I_EMISS_nLight            AT %PLCFLAGS.0;   
I_ERR_nLight              AT %PLCFLAGS.0; 
I_RDY_nLight              AT %PLCFLAGS.1;  
I_ExtCtr_Ready_nLight     AT %PLCFLAGS.1; 

I_PUL_START               AT %PLCFLAGS.0; 
I_PUL_STOP                AT %PLCFLAGS.0; 
I_PUL_RESET               AT %PLCFLAGS.0;

I_UNLOAD_DOOR             AT %PLCFLAGS.0;
I_LUB_LOW_PRESSURE        AT %PLCFLAGS.0;
I_OVER_PRES_LUB           AT %PLCFLAGS.0;
I_ASPIRATORE_OK           AT %PLCFLAGS.1;
						  
I_I_FC_Z_FOLL2_NEG        AT %PLCFLAGS.0;
I_I_FC_Z_FOLL2_POS        AT %PLCFLAGS.0; 

I_PRESSIONE_ARIA_BANCO    AT %PLCFLAGS.1; 
I_FC_C_POS                AT %PLCFLAGS.0;
I_FC_C_NEG                AT %PLCFLAGS.0;
I_FC_X1_POS               AT %PLCFLAGS.0;
I_FC_X1_NEG               AT %PLCFLAGS.0;
I_Grating_Alm             AT %PLCFLAGS.0;
I_SafetyDevice_Bypass     AT %PLCFLAGS.0;
I_UnlckFence_Btn          AT %PLCFLAGS.0;
 
* IPG - Resonator Input

I_LSR_EMISSION_ON         AT %PLCFLAGS.1;  
I_LSR_GUIDE_ON            AT %PLCFLAGS.0;  
I_LSR_ANALOG_CONTROL      AT %PLCFLAGS.0;  
I_LSR_ASSIGNED            AT %PLCFLAGS.1;  
I_LSR_LASER_ON            AT %PLCFLAGS.0;  
I_LSR_WARNING             AT %PLCFLAGS.0;  

I_IA_Feedb_HOERB_GAS_OUT  AT %PLCFLAGS.0;  *%ecat_ring0.EcatNode28.EcMDPIn1;
I_HOERBIGER_READY         AT %PLCFLAGS.0;  *%ecat_ring0.EcatNode28.EcMDPIn3.1;
						  
Safety_Door_Shield        AT %M5.0; 
Unload_Door_Shield        AT %M5.0; 
Safety_Door_Lock          AT %M5.0; 

O_Guard_Door_1            AT %M5.0;
O_Guard_Door_2            AT %M5.0;
O_EXTERNAL_SAFETY_DEVICE  AT %M5.0; 
O_RESET_SAFETY_DEVICE     AT %M5.0;  *
O_LUCE_CABINA             AT %M5.0;  * MACHINE LIGHT
O_Guard_Door_1_Dwn        AT %M5.0;
O_Guard_Door_2_Dwn        AT %M5.0;
O_Bin_Door_Dwn            AT %M5.0;

O_LSR_REQUEST             AT %M5.0;  * Out A1
O_LSR_RESET               AT %M5.0;  * Out A4

O_BUZZER_ALARM            AT %M5.0;
O_BUZZER_START_CIC		  AT %M5.0;  *
						  
O_CLAMP_C_CLOSE           AT %M5.0;
O_CLAMP_C_OPEN            AT %M5.0;
O_CLAMP_C1_CLOSE          AT %M5.0;
O_CLAMP_C1_OPEN           AT %M5.0;
						  
O_Lock_Safety_Door        AT %M5.0;
O_Safety_Bypass           AT %M5.0;
O_Unlock_Fence            AT %M5.0;
O_Unlock_Fence_LED        AT %M5.0;

* Valve

O_PILOT_HOERB             AT %M5.0; *%ecat_ring0.EcatNode8.EcMDPOut2.4; *
O_CALIB_REQ_HOERB         AT %M5.0; *%ecat_ring0.EcatNode8.EcMDPOut2.3;
O_EV_OPEN_N2_HOERB        AT %M5.0; *%ecat_ring0.EcatNode8.EcMDPOut2.1;
O_EV_OPEN_O2_HOERB        AT %M5.0; *%ecat_ring0.EcatNode8.EcMDPOut2.0;
O_EV_OPEN_ARIA_HOERB      AT %M5.0; *%ecat_ring0.EcatNode8.EcMDPOut2.2;

OA_GAS_HOERB              AT %M5;   *%ecat_ring0.EcatNode8.EcMDPOut1;
 
O_LED_AUXON               AT %M5.0;
O_LED_START               AT %M5.0;
O_APRI_PORTA              AT %M5.0;
O_Free002_7               AT %M5.0;
O_CN_READY                AT %M5.0;
O_EN_DRIVE                AT %M5.0;
O_ASPIRATORE_ON           AT %M5.0;
O_CHILLER_ON              AT %M5.0;
O_EV_MISURATORE_BAS       AT %M5.0;
START_ASPIRATORE_FUMI     AT %M5.0;
O_EVGAS_PRES              AT %M5.0;
O_LSR_ANALOG_CONTROL      AT %M5.0;

* Precitec Head

I_PREC_INPOS              AT %PLCFLAGS.1;    * Precitec focal in position
I_PREC_ERROR              AT %PLCFLAGS.0;    * Precitec focal in error

I_PREC_FAR                AT %PLCFLAGS.0;    * A10_Precitec:Lontano(Normal operation-Sensor outside measuring range)
I_PREC_COLLISION          AT %PLCFLAGS.0;    * A11_Precitec:Tip\Body Touch o Nozzle_Lost (TIP TOUCH:A11=1,A14=0,A15=0)
I_PREC_BROKEN             AT %PLCFLAGS.0;    * CABLE_CUT_A12_Precitec:Cavo interrotto ]
I_PREC_READY              AT %PLCFLAGS.1;    * A13_Precitec:Pronto
I_PREC_BODY_TOUCH         AT %PLCFLAGS.0;    * A14_Precitec:Collisione Corpo sensore (BODY TOUCH:A11=1,A14=1,A15=0)
I_PREC_REACH              AT %PLCFLAGS.0;    * A15_Precitec:Nozzle Lost [Posizione raggiunta] (NOZZLE LOST:A11=1,A14=0,A15=1)
I_PREC_CAPACITIVO         AT %M5;  

O_PREC_POSITION           AT %M5.0;          * Precitec focal Automatic
O_PREC_REF                AT %M5.0;          * Precitec focal Reference
O_PREC_FOCALE             AT %M5;            * Precitec focal analog command

O_PREC_CALIBRAZIONE       AT %M5.0;          * A3 Calibration Request
O_PREC_SELCHAR_0          AT %M5.0;          * A4 Select Char Bit0
O_PREC_SELCHAR_1          AT %M5.0;          * A5 Select Char Bit1
O_PREC_SELCHAR_2          AT %M5.0;          * A6 Select Char Bit2
O_PREC_STROBE             AT %M5.0;          * A7 Calibration Strobe

*// CHC-1000S, EDS-C100, QC300 Capacitive Sensor *//

I_ECAT_FAR                AT %PLCFLAGS.0;    * A10  %ecat_ring0.EcatNode28.EcMDPIn1.0
I_ECAT_COLLISION          AT %PLCFLAGS.0;    * A11  %ecat_ring0.EcatNode28.EcMDPIn1.1
I_ECAT_BROKEN             AT %PLCFLAGS.0;    * A12  %ecat_ring0.EcatNode28.EcMDPIn1.2
I_ECAT_READY              AT %PLCFLAGS.1;    * A13  %ecat_ring0.EcatNode28.EcMDPIn1.3
I_ECAT_BODY_TOUCH         AT %PLCFLAGS.0;    * A14  %ecat_ring0.EcatNode28.EcMDPIn1.4
I_ECAT_REACH              AT %PLCFLAGS.0;    * A15  %ecat_ring0.EcatNode28.EcMDPIn1.5
I_ECAT_CAPACITIVO         AT %M5;            *      %ecat_ring0.EcatNode28.EcMDPIn0

O_ECAT_CALIBRAZIONE       AT %M5.0;          * A3   %ecat_ring0.EcatNode28.EcMDPOut1.0
O_ECAT_SELCHAR_0          AT %M5.0;          * A4   %ecat_ring0.EcatNode28.EcMDPOut1.1
O_ECAT_SELCHAR_1          AT %M5.0;          * A5   %ecat_ring0.EcatNode28.EcMDPOut1.2
O_ECAT_SELCHAR_2          AT %M5.0;          * A6   %ecat_ring0.EcatNode28.EcMDPOut1.3
O_ECAT_STROBE             AT %M5.0;          * A7   %ecat_ring0.EcatNode28.EcMDPOut1.4

*// ******************************************** *//

* RAYCUS

I_S_ERROR_RAYCUS          AT %PLCFLAGS.0;
I_RDY_RAYCUS              AT %PLCFLAGS.1;
O_Guide_RAYCUS            AT %M5.0;

***************************

I_FC_L5_POS               AT %PLCFLAGS.1;
I_FC_L5_NEG               AT %PLCFLAGS.1;
