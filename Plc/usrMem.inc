VAR_IN_OUT 
TM32_Mobile_Spindle_Slave_Open      AT  %tmShutters2.TS_commandWords0.0;   
TM32_SpindleFix_Open                AT  %tmShutters1.TS_commandWords0.0; 
TM32_Mobile_Spindle_Open            AT  %tmShutters0.TS_commandWords0.0;  

I_I_Support_1_Dwn                   AT  %USR_M0.13;            * %USR_M0.13
I_I_Support_2_Dwn                   AT  %USR_M0.14;            * %USR_M0.14
I_I_Support_3_Dwn                   AT  %USR_M0.15;            * %USR_M0.15
I_I_Support_4_Dwn                   AT  %USR_M0.16;            * %USR_M0.16
I_I_Support_5_Dwn                   AT  %USR_M0.17;            * %USR_M0.17
I_I_Support_6_Dwn                   AT  %USR_M0.18;            * %USR_M0.18
I_I_Support_7_Dwn                   AT  %USR_M0.19;            * %USR_M0.19
I_I_Support_1_Dwn_Un                AT  %USR_M0.20;            * %USR_M0.20
I_I_Support_2_Dwn_Un                AT  %USR_M0.21;            * %USR_M0.21
I_I_Support_3_Dwn_Un                AT  %USR_M0.22;            * %USR_M0.22
I_I_Support_4_Dwn_Un                AT  %USR_M0.23;            * %USR_M0.23
I_I_Support_5_Dwn_Un                AT  %USR_M0.24;            * %USR_M0.24
I_I_Support_6_Dwn_Un                AT  %USR_M0.25;            * %USR_M0.25
I_I_Support_7_Dwn_Un                AT  %USR_M0.26;            * %USR_M0.26
I_I_Mobile_Spindle_Open             AT  %USR_M0.27;            * %USR_M0.27
I_I_Mobile_Spindle_Close            AT  %USR_M0.28;            * %USR_M0.28
I_I_Mobile_Spindle_Slave_Open       AT  %USR_M0.29;            * %USR_M0.29
I_I_Mobile_Spindle_Slave_Close      AT  %USR_M0.30;            * %USR_M0.30
I_I_FixedSpindle_Open               AT  %USR_M0.31;            * %USR_M0.31

I_I_Support_1_Up                    AT  %USR_M1.13;            * %USR_M1.13
I_I_Support_2_Up                    AT  %USR_M1.14;            * %USR_M1.14
I_I_Support_3_Up                    AT  %USR_M1.15;            * %USR_M1.15
I_I_Support_4_Up                    AT  %USR_M1.16;            * %USR_M1.16
I_I_Support_5_Up                    AT  %USR_M1.17;            * %USR_M1.17
I_I_Support_6_Up                    AT  %USR_M1.18;            * %USR_M1.18
I_I_Support_7_Up                    AT  %USR_M1.19;            * %USR_M1.19
I_I_Support_1_Up_Un                 AT  %USR_M1.20;            * %USR_M1.20
I_I_Support_2_Up_Un                 AT  %USR_M1.21;            * %USR_M1.21
I_I_Support_3_Up_Un                 AT  %USR_M1.22;            * %USR_M1.22
I_I_Support_4_Up_Un                 AT  %USR_M1.23;            * %USR_M1.23
I_I_Support_5_Up_Un                 AT  %USR_M1.24;            * %USR_M1.24
I_I_Support_6_Up_Un                 AT  %USR_M1.25;            * %USR_M1.25
I_I_Support_7_Up_Un                 AT  %USR_M1.26;            * %USR_M1.26
I_I_TubeMobile_Spindle              AT  %USR_M1.27;            * %USR_M1.27 
I_I_TubeFix_Spindle                 AT  %USR_M1.28;            * %USR_M1.28 
I_I_FlapL2_Home                     AT  %USR_M1.29;            * %USR_M1.29
I_I_FlapL2_Inclined                 AT  %USR_M1.30;            * %USR_M1.30
I_I_FixedSpindle_Close              AT  %USR_M1.31;            * %USR_M1.31

I_I_MobileSpind_An                  AT  %USR_M2; 
I_I_FixedSpind_An                   AT  %USR_M3; 						        
I_I_DistSens_Y                      AT  %USR_M4;   
I_I_DistSens_Z                      AT  %USR_M5;							        
I_I_MobSpind_Y                      AT  %USR_M6;               * Mobile spindle potentiometer sensor Y
I_I_MobSpind_Z                      AT  %USR_M7;               * Mobile spindle potentiometer sensor Z							        
I_I_FixSpind_Y                      AT  %USR_M8;               * Fixed spindle potentiometer sensor Y
I_I_FixSpind_Z                      AT  %USR_M9;               * Fixed spindle potentiometer sensor Z
I_I_Pressure_Mobile                 AT  %USR_M10;              * Feedback mobile spindle pressure
I_I_Pressure_Fix                    AT  %USR_M11;              * Feedback fix spindle pressure

I_I_Butt_Open_Mobile                AT  %USR_M12.0;
I_I_Butt_Close_Mobile               AT  %USR_M12.1;
I_I_Butt_Open_Fix                   AT  %USR_M12.2;
I_I_Butt_Close_Fix                  AT  %USR_M12.3;
I_I_Ready_To_Load                   AT  %USR_M12.4;
I_I_Loader_Reach_Pos                AT  %USR_M12.5;            * Loader JQ E: P
I_I_Loader_Back_Pos                 AT  %USR_M12.6;            * Loader JQ E: N
I_I_Loader_Error                    AT  %USR_M12.7;
I_I_Flap_Inclined                   AT  %USR_M12.8;
I_I_Flap_Home                       AT  %USR_M12.9;
I_I_Loader_Alarm                    AT  %USR_M12.10;
I_I_Mobile_Spindle_Slave_Open2      AT  %USR_M12.12; 
*                                   AT  %USR_M12.13;           
*                                   AT  %USR_M12.14;           
*                                   AT  %USR_M12.15;           
*                                   AT  %USR_M12.16;           
*                                   AT  %USR_M12.17;           
*                                   AT  %USR_M12.18;           
*                                   AT  %USR_M12.19;           
*                                   AT  %USR_M12.20;           
*                                   AT  %USR_M12.21;           
*                                   AT  %USR_M12.22;           
*                                   AT  %USR_M12.23;           
*                                   AT  %USR_M12.24;           	
*                                   AT  %USR_M12.25;           
*                                   AT  %USR_M12.26;           	
I_I_FlapL3_Home                     AT  %USR_M12.27;
I_I_FlapL3_Inclined                 AT  %USR_M12.28;
I_I_FlapL4_Home                     AT  %USR_M12.29;
I_I_FlapL4_Inclined                 AT  %USR_M12.30;
I_I_FlapL5_Home                     AT  %USR_M12.31;


I_I_FlapL5_Inclined                 AT  %USR_M13.0;
I_I_SawClampZ_Rot_Open              AT  %USR_M13.1;
I_I_SawClampZ_Rot_Close             AT  %USR_M13.2;
I_I_INDEX_MAND                      AT  %USR_M13.3;
I_I_CW                              AT  %USR_M13.4;
I_I_CCW                             AT  %USR_M13.5;
I_I_M1_M2_M3_ALARM                  AT  %USR_M13.6;
I_I_B1_B2_OPEN                      AT  %USR_M13.7;
I_I_B1_B2_CLOSE                     AT  %USR_M13.8;
I_I_B1_B2_ALARM                     AT  %USR_M13.9;
I_I_M1_M2_M3_OPEN                   AT  %USR_M13.10;
I_I_M1_M2_M3_CLOSE                  AT  %USR_M13.11;
I_I_L1_Down                         AT  %USR_M13.12;
*                                   AT  %USR_M13.13;        
* 2nd 3 spindle machine 
I_I_TubeOnChain_D                   AT  %USR_M13.14;                 
I_I_CylinderUp_G                    AT  %USR_M13.15;                 
I_I_CylinderDwn_G                   AT  %USR_M13.16;
I_I_Loader_Slow                     AT  %USR_M13.17;                 
I_I_TubeOnSup_C                     AT  %USR_M13.18;       
I_I_CylinderForw_F                  AT  %USR_M13.19;                 
I_I_CylinderBack_F                  AT  %USR_M13.20;
I_I_HomeChain_E                     AT  %USR_M13.21;  
I_I_Bin_DoorUp                      AT  %USR_M13.22;  
I_I_Bin_DoorDwn                     AT  %USR_M13.23;  
I_I_Centering_Open_1                AT  %USR_M13.24;  
I_I_Centering_Open_2                AT  %USR_M13.25;  
I_I_Centering_Open_3                AT  %USR_M13.26;  
I_I_Centering_Open_4                AT  %USR_M13.27;  
I_I_Pul_Mobile_Spindle_Close        AT  %USR_M13.28;  
I_I_Pul_Mobile_Spindle_Open         AT  %USR_M13.29;  
   
O_O_Mobile_Spindle_Slave_Open       AT  %USR_M20.0;
O_O_Mobile_Spindle_Slave_Close      AT  %USR_M20.1;
O_O_Mobile_Spindle_Open             AT  %USR_M20.2;
O_O_Mobile_Spindle_Close            AT  %USR_M20.3;
O_O_Mobile_Spindle_Slave_Brake      AT  %USR_M20.4;
O_Pump                              AT  %USR_M20.8;
*                                   AT  %USR_M20.9;
*                                   AT  %USR_M20.10;
*                                   AT  %USR_M20.11;
O_O_Ev_Support1_Up		            AT  %USR_M20.12;
O_O_Ev_Support2_Up	                AT  %USR_M20.13;
O_O_Ev_Support3_Up	                AT  %USR_M20.14;
O_O_Ev_Support4_Up	                AT  %USR_M20.15;     
O_O_Ev_Support5_Up	                AT  %USR_M20.16;
O_O_Ev_Support6_Up	                AT  %USR_M20.17;
O_O_Ev_Support7_Up	                AT  %USR_M20.18;     
O_O_Ev_Support1_Dwn	                AT  %USR_M20.19;
O_O_Ev_Support2_Dwn	                AT  %USR_M20.20;
O_O_Ev_Support3_Dwn	                AT  %USR_M20.21;
O_O_Ev_Support4_Dwn	                AT  %USR_M20.22;
O_O_Ev_Support5_Dwn	                AT  %USR_M20.23;
O_O_Ev_Support6_Dwn	                AT  %USR_M20.24;
O_O_Ev_Support7_Dwn	                AT  %USR_M20.25;    

O_O_Ev_Support1_Up_Un		        AT  %USR_M21.0;   
O_O_Ev_Support2_Up_Un	            AT  %USR_M21.1;   
O_O_Ev_Support3_Up_Un	            AT  %USR_M21.2;   
O_O_Ev_Support4_Up_Un	            AT  %USR_M21.3;   
O_O_Ev_Support5_Up_Un	            AT  %USR_M21.4;   
O_O_Ev_Support6_Up_Un	            AT  %USR_M21.5;   
O_O_Ev_Support7_Up_Un	            AT  %USR_M21.6;   
O_O_Ev_Support1_Dwn_Un	            AT  %USR_M21.7;   
O_O_Ev_Support2_Dwn_Un	            AT  %USR_M21.8;   
O_O_Ev_Support3_Dwn_Un	            AT  %USR_M21.9;   
O_O_Ev_Support4_Dwn_Un	            AT  %USR_M21.10;   
O_O_Ev_Support5_Dwn_Un	            AT  %USR_M21.11;   
O_O_Ev_Support6_Dwn_Un	            AT  %USR_M21.12;   
O_O_Ev_Support7_Dwn_Un	            AT  %USR_M21.13;  	        
*                                   AT  %USR_M21.14;
O_O_Loader_Back                     AT  %USR_M21.15;
O_O_Bin_Door                        AT  %USR_M21.16;
O_O_FlapL2_Incline                  AT  %USR_M21.17;
O_O_FlapL2_Home                     AT  %USR_M21.18;
O_O_Flap_Incline                    AT  %USR_M21.19;
O_O_Flap_Home                       AT  %USR_M21.20;
O_O_SpindleFix_Open                 AT  %USR_M21.21;
O_O_SpindleFix_Close                AT  %USR_M21.22;
O_O_SawClampY_Open                  AT  %USR_M21.23;
O_O_SawClampY_Close                 AT  %USR_M21.24;
O_O_SawClampZ_Open                  AT  %USR_M21.25;
O_O_SawClampZ_Close                 AT  %USR_M21.26;
O_O_FlapL3_Home                     AT  %USR_M21.27;
O_O_FlapL3_Incline                  AT  %USR_M21.28;
O_O_FlapL4_Home                     AT  %USR_M21.29;
O_O_FlapL4_Incline                  AT  %USR_M21.30;
O_O_FlapL5_Home                     AT  %USR_M21.31;

Fun_M720_Ch0                        AT  %USR_M22.0;             * Mobile spindle open
Fun_M721_Ch0		                AT  %USR_M22.1;             * Mobile spindle close
Fun_M722_Ch0                        AT  %USR_M22.2;             * Fix spindle open 
Fun_M723_Ch0                        AT  %USR_M22.3;             * Fix spindle close
Fun_M724_Ch0                        AT  %USR_M22.4;             * Auto Up command for supports
Fun_M725_Ch0                        AT  %USR_M22.5;             * Auto Up command for supports
Fun_M726_Ch0                        AT  %USR_M22.6;             * Auto Up command for supports
Fun_M727_Ch0                        AT  %USR_M22.7;
Fun_M728_Ch0                        AT  %USR_M22.8;
Fun_M54_Ch7                         AT  %USR_M22.9;             * Auto down
Fun_M825_Ch0                        AT  %USR_M22.10;            * Auto down unload
Fun_M730_Ch0                        AT  %USR_M22.11;            * Request for load pipe
Fun_M731_Ch0                        AT  %USR_M22.12;            * Request back loader
Fun_M732_Ch0                        AT  %USR_M22.13;            * Read length
Fun_M733_Ch0                        AT  %USR_M22.18;            * UP SUPP 1
Fun_M734_Ch0                        AT  %USR_M22.19;            * UP SUPP 2
Fun_M735_Ch0                        AT  %USR_M22.20;            * UP SUPP 3
Fun_M736_Ch0                        AT  %USR_M22.21;            * UP SUPP 4
Fun_M826_Ch0                        AT  %USR_M22.14;            * L1 Home
Fun_M827_Ch0                        AT  %USR_M22.15;            * L1 Incline
Fun_M829_Ch0                        AT  %USR_M22.16;            * L2 Home
Fun_M830_Ch0                        AT  %USR_M22.17;            * L2 Incline
Fun_M431_Ch0                        AT  %USR_M22.18;            * Slow Backword Loader Chain
Fun_M432_Ch0                        AT  %USR_M22.19;            
Fun_M981_Ch0                        AT  %USR_M22.22;            * Middle fixed spindle open (3 spindles)
Fun_M982_Ch0                        AT  %USR_M22.23;            * Middle fixed spindle close (3 spindles)
Fun_M983_Ch0                        AT  %USR_M22.24;            * Saw Clamp Y open (3 spindles + Saw)
Fun_M984_Ch0                        AT  %USR_M22.25;            * Saw Clamp Y close (3 spindles + Saw)
Fun_M985_Ch0                        AT  %USR_M22.26;            * Saw Clamp Z open (3 spindles + Saw)
Fun_M986_Ch0                        AT  %USR_M22.27;            * Saw Clamp Z close (3 spindles + Saw)
Fun_M987_Ch0                        AT  %USR_M22.28;            * Flap L3 Home (3 spindles + Saw)
Fun_M988_Ch0                        AT  %USR_M22.29;            * Flap L3 Incline (3 spindles + Saw)
Fun_M989_Ch0                        AT  %USR_M22.30;            * Flap L4 Home (3 spindles + Saw)
Fun_M990_Ch0                        AT  %USR_M22.31;            * Flap L4 Incline (3 spindles + Saw)

MainChFeedrate                      AT  %USR_M23;               * feedrate for channel
                             
Reset                               AT  %USR_M24.0;             * Reset Command               
Start_Ch0                           AT  %USR_M24.1;             * Start Request to Ch0        
WaitMfunc0Usr                       AT  %USR_M24.2;             * wait m function on ch0      
ProgRunCh0                          AT  %USR_M24.3;             * program in run ch0          
Ch7_Load_Jog                        AT  %USR_M24.4;             * ch7 in jog                  
EmergButton                         AT  %USR_M24.5;             * General Emergency button    
PlcEmerGen                          AT  %USR_M24.6;             * General Plc Emergency  
PRGRUN_CN8                          AT  %USR_M24.7;             * Program run cn8
Ch0_Auto                            AT  %USR_M24.8;             * 
Ch0_Jog                             AT  %USR_M24.9;             * 
Ch0_Inc                             AT  %USR_M24.10;            * 
Ch0_Ref_Gen                         AT  %USR_M24.11;            * 
Ch0_Ref_Sin                         AT  %USR_M24.12;            * 
ResetChannels                       AT  %USR_M24.13;            * 
Ch0Alarm                            AT  %USR_M24.14;            *
Pul_Stop_Req                        AT  %USR_M24.15;            *
AxStopSincGRPMacc                   AT  %USR_M24.16;            *
MainAxesPower                       AT  %USR_M24.17;            *
MainAxesPowerAllowed                AT  %USR_M24.18;            *
GstMaccSelZCh8                      AT  %USR_M24.19;            *
M_HoldAx_U                          AT  %USR_M24.20;            * 
AllowFlapDown                       AT  %USR_M24.21;            *
GstMaccSelYCh8                      AT  %USR_M24.22;            *
GstMaccSelUCh8                      AT  %USR_M24.23;            *
GstMaccSelVCh8                      AT  %USR_M24.24;            *
GstMaccSelWCh8                      AT  %USR_M24.25;            *
Pon_UnSupp_71                       AT  %USR_M24.26;
Pon_UnSupp_72                       AT  %USR_M24.27;
Pon_UnSupp_73                       AT  %USR_M24.28;
Pon_UnSupp_74                       AT  %USR_M24.29;
Pon_UnSupp_75                       AT  %USR_M24.30;
M_HoldAx_V                          AT  %USR_M24.31;

DistanceMm_Y                        AT  %USR_M25;               *
DistanceMm_Z                        AT  %USR_M26;               *
DistMobMm_Y                         AT  %USR_M27;               *
DistMobMm_Z                         AT  %USR_M28;               *
DistFixMm_Y                         AT  %USR_M29;               *
DistFixMm_Z                         AT  %USR_M30;               *
O_O_Pressure_Mobile                 AT  %USR_M31;
O_O_Pressure_MobileS                AT  %USR_M32;
PressMobClose                       AT  %USR_M33;
PressMobileSClose                   AT  %USR_M34;
MobClosePressReq                    AT  %USR_M35;
MobileSClosePressReq                AT  %USR_M36;
OFFSETM4                            AT  %USR_M37;
tmp_lim_holder                      AT  %USR_M38;
safedisPosQta                       AT  %USR_M39;
safedisNegQta                       AT  %USR_M40;

O_O_FlapL5_Incline                  AT  %USR_M41.0;
O_O_SawClampZ_Rot_Open              AT  %USR_M41.1;
O_O_SawClampZ_Rot_Close             AT  %USR_M41.2;
O_O_M1_M2_M3_Open                   AT  %USR_M41.3;             * J
O_O_M1_M2_M3_Close                  AT  %USR_M41.4;             * J
O_O_Centering_Open_1                AT  %USR_M41.5;
O_O_Centering_Close_1               AT  %USR_M41.6;
O_O_B1_B2_Open                      AT  %USR_M41.7;
O_O_B1_B2_Close                     AT  %USR_M41.8;
O_O_CylinderUp_G                    AT  %USR_M41.9;
O_O_CylinderDwn_G                   AT  %USR_M41.10;
O_O_Chain_Forw_D                    AT  %USR_M41.11;
O_O_Chain_Back_D                    AT  %USR_M41.12;
O_O_Arm_Slow                        AT  %USR_M41.13;
O_O_Arm_Fast                        AT  %USR_M41.14;
O_O_CylinderForw_F                  AT  %USR_M41.15;
O_O_CylinderBack_F                  AT  %USR_M41.16;
O_O_Rst_InverterAlm                 AT  %USR_M41.17;
O_O_Centering_Open_2                AT  %USR_M41.18;
O_O_Centering_Close_2               AT  %USR_M41.19;
O_O_Centering_Open_3                AT  %USR_M41.20;
O_O_Centering_Close_3               AT  %USR_M41.21;
O_O_Centering_Open_4                AT  %USR_M41.22;
O_O_Centering_Close_4               AT  %USR_M41.23;
O_O_WeldDetec_En                    AT  %USR_M41.24;
O_O_Detec_Weld_Piston               AT  %USR_M41.25;
O_O_Detec_Weld                      AT  %USR_M41.26;
* Status for pressure holding chucks
MidSpdClosing                       AT  %USR_M41.27;
MidSpdOpening                       AT  %USR_M41.28;
SlaveSpdClosing                     AT  %USR_M41.29;
SlaveSpdOpening                     AT  %USR_M41.30;
*                                   AT  %USR_M41.31;

Fun_M991_Ch0                        AT  %USR_M42.0;             * Flap L5 Home (3 spindles + Saw)
Fun_M992_Ch0                        AT  %USR_M42.1;             * Flap L5 Incline (3 spindles + Saw)
Fun_M993_Ch0                        AT  %USR_M42.2;             * All Flaps to unload forced to be flat
Fun_M994_Ch0                        AT  %USR_M42.3;             * Saw Clamp Rotate Close
Fun_M330_Ch0                        AT  %USR_M42.4;             * Open M1 M2 M3
Fun_M331_Ch0                        AT  %USR_M42.5;             * Close M1 M2 M3
Fun_M332_Ch0                        AT  %USR_M42.6;             * Open Centering Clamp
Fun_M333_Ch0                        AT  %USR_M42.7;             * Close Centering Clamp
Fun_M334_Ch0                        AT  %USR_M42.8;             * V shape
Fun_M335_Ch0                        AT  %USR_M42.9;             * Flat shape
Fun_M336_Ch0                        AT  %USR_M42.10;            * o1 to mach
Fun_M337_Ch0                        AT  %USR_M42.11;            * o1 to unload
Fun_M338_Ch0                        AT  %USR_M42.12;            * o2 to mach
Fun_M339_Ch0                        AT  %USR_M42.13;            * o2 to unload
Fun_M440_Ch0                        AT  %USR_M42.14;            * Move axis 30
Fun_M441_Ch0                        AT  %USR_M42.15;            * Move axis 31
Fun_M442_Ch0                        AT  %USR_M42.16;            * Move axis 36
Fun_M434_Ch0                        AT  %USR_M42.17;            * V shape (Unloader)
Fun_M435_Ch0                        AT  %USR_M42.18;            * Flat shape (Unloader)
Fun_M455_Ch0                        AT  %USR_M42.19;
Fun_M443_Ch0                        AT  %USR_M42.20;            * D Chain forward to machine
Fun_M444_Ch0                        AT  %USR_M42.21;            * D Chain backward
Fun_M445_Ch0                        AT  %USR_M42.22;            * Whole Loader Homming Position
Fun_M446_Ch0                        AT  %USR_M42.23;            * Open Centering Clamp 2
Fun_M447_Ch0                        AT  %USR_M42.24;            * Close Centering Clamp 2
Fun_M448_Ch0                        AT  %USR_M42.25;            * Open Centering Clamp 3
Fun_M449_Ch0                        AT  %USR_M42.26;            * Close Centering Clamp 3
Fun_M450_Ch0                        AT  %USR_M42.27;            * Open Centering Clamp 4
Fun_M451_Ch0                        AT  %USR_M42.28;            * Close Centering Clamp 4
Fun_M452_Ch0                        AT  %USR_M42.29;            * Open All the Centering Clamps
Fun_M453_Ch0                        AT  %USR_M42.30;
Fun_M454_Ch0                        AT  %USR_M42.31;
               
dynamicBsafe                        AT  %USR_M43;

Pon_C_SAW                           AT  %USR_M44.0;
PRGRUN_CN9                          AT  %USR_M44.1;             * Program run cn9
Pon_UnSupp_81                       AT  %USR_M44.2;
Pon_UnSupp_82                       AT  %USR_M44.3;
Pon_UnSupp_83                       AT  %USR_M44.4;
Pon_UnSupp_84                       AT  %USR_M44.5;
Pon_UnSupp_85                       AT  %USR_M44.6;
Emer_Sup_QtaErr                     AT  %USR_M44.7;
Pon_UnSupp_86                       AT  %USR_M44.8;
Pon_UnSupp_87                       AT  %USR_M44.9;
Pon_UnSupp_88                       AT  %USR_M44.10;
I_I_AX85_NEG	                    AT  %USR_M44.11;												   
I_I_AX85_POS                        AT  %USR_M44.12;
I_I_AX86_NEG	                    AT  %USR_M44.13;												   
I_I_AX86_POS                        AT  %USR_M44.14;
I_I_AX87_NEG	                    AT  %USR_M44.15;												   
I_I_AX87_POS                        AT  %USR_M44.16;
I_I_AX88_NEG	                    AT  %USR_M44.17;												   
I_I_AX88_POS                        AT  %USR_M44.18;
I_I_SUPPORT_71_Dwn                  AT  %USR_M44.19;
I_I_SUPPORT_71_Up                   AT  %USR_M44.20;
I_I_SUPPORT_72_Dwn                  AT  %USR_M44.21;
I_I_SUPPORT_72_Up                   AT  %USR_M44.22;
I_I_SUPPORT_73_Dwn                  AT  %USR_M44.23;
I_I_SUPPORT_73_Up                   AT  %USR_M44.24;
I_I_SUPPORT_74_Dwn                  AT  %USR_M44.25;
I_I_SUPPORT_74_Up                   AT  %USR_M44.26;
I_I_SUPPORT_75_Dwn                  AT  %USR_M44.27;
I_I_SUPPORT_75_Up                   AT  %USR_M44.28;

O_O_Ev_Support71_Up                 AT  %USR_M45.0;
O_O_Ev_Support71_Dwn                AT  %USR_M45.1;
O_O_Ev_Support72_Up                 AT  %USR_M45.2;
O_O_Ev_Support72_Dwn                AT  %USR_M45.3;
O_O_Ev_Support73_Up                 AT  %USR_M45.4;
O_O_Ev_Support73_Dwn                AT  %USR_M45.5;
O_O_Ev_Support74_Up                 AT  %USR_M45.6;
O_O_Ev_Support74_Dwn                AT  %USR_M45.7;
O_O_Ev_Support75_Up                 AT  %USR_M45.8;
O_O_Ev_Support75_Dwn                AT  %USR_M45.9;

PosUMis                             AT  %USR_M47;
SupervisePos_X1                     AT  %USR_M48;

END_VAR 