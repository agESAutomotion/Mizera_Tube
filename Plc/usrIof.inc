***************************************************************
*
* usrIof.inc Input Output for user routines 
*
***************************************************************
I_Flap_Inclined_A              AT %PLCFLAGS.0;
I_Flap_Inclined_B              AT %PLCFLAGS.0;
I_FlapL2_Inclined_A            AT %PLCFLAGS.0;
I_FlapL2_Inclined_B            AT %PLCFLAGS.0;
I_FlapL3_Inclined_A            AT %PLCFLAGS.0;
I_FlapL3_Inclined_B            AT %PLCFLAGS.0;
I_FlapL4_Inclined_A            AT %PLCFLAGS.0;  
I_FlapL4_Inclined_B            AT %PLCFLAGS.0; 
I_Pul_Mobile_Spindle_Close     AT %PLCFLAGS.0; 
I_Pul_Mobile_Spindle_Open      AT %PLCFLAGS.0; 
I_Mobile_Spindle_Slave_Open    AT %PLCFLAGS.0; 
I_Mobile_Spindle_Slave_Open2   AT %PLCFLAGS.0; 

I_Loader_Back_Pos              AT %PLCFLAGS.1;
I_Loader_Reach_Pos             AT %PLCFLAGS.1;

I_SUPPORT_72_Dwn               AT %PLCFLAGS.1;
I_SUPPORT_72_Up                AT %PLCFLAGS.1;
I_SUPPORT_71_Dwn               AT %PLCFLAGS.1;
I_SUPPORT_71_Up                AT %PLCFLAGS.1;
I_SUPPORT_73_Dwn               AT %PLCFLAGS.1;
I_SUPPORT_73_Up                AT %PLCFLAGS.1;
I_SUPPORT_74_Dwn               AT %PLCFLAGS.1;
I_SUPPORT_74_Up                AT %PLCFLAGS.1;
I_SUPPORT_75_Dwn               AT %PLCFLAGS.1;
I_SUPPORT_75_Up                AT %PLCFLAGS.1;

I_DistSens_Y                   AT %M6;
I_DistSens_Z                   AT %M6;
					          
I_MobSpind_Y                   AT %M6;       * Mobile spindle potentiometer sensor Y
I_MobSpind_Z                   AT %M6;       * Mobile spindle potentiometer sensor Z
I_FixSpind_Y                   AT %M6;       * Fixed spindle potentiometer sensor Y
I_FixSpind_Z                   AT %M6;       * Fixed spindle potentiometer sensor Z
					          
I_Pressure_Mobile              AT %M6;
I_Pressure_Fix                 AT %M6;

O_Pressure_Mobile              AT %M6;
O_Pressure_MobileS             AT %M6;

O_Centering_Open               AT %M5.0;
O_Centering_Close              AT %M5.0;
O_B1_B2_Open                   AT %M5.0;
O_B1_B2_Close                  AT %M5.0;
O_CylinderDwn_G                AT %M5.0;
 
I_FixedSpindle_Open            AT %PLCFLAGS.0;
I_FixedSpindle_Close           AT %PLCFLAGS.0;
I_SawClampZ_Open               AT %PLCFLAGS.0;
I_SawClampZ_Close              AT %PLCFLAGS.0;
I_SawClampZ_Rot_Open           AT %PLCFLAGS.0;
I_SawClampZ_Rot_Close          AT %PLCFLAGS.0;
I_SawClampY_Open               AT %PLCFLAGS.0;
I_SawClampY_Close              AT %PLCFLAGS.0;
I_FlapL3_Inclined              AT %PLCFLAGS.0;
I_FlapL4_Home                  AT %PLCFLAGS.0;
I_FlapL4_Inclined              AT %PLCFLAGS.0;
I_FlapL5_Home                  AT %PLCFLAGS.0;
I_FlapL5_Inclined              AT %PLCFLAGS.0;

I_Mobile_Spindle_Open          AT %PLCFLAGS.0;
I_Mobile_Spindle_Close         AT %PLCFLAGS.0; 
I_Mobile_Spindle_Slave_Close   AT %PLCFLAGS.0; 

I_Butt_Open_Mobile             AT %PLCFLAGS.0;
I_Butt_Close_Mobile            AT %PLCFLAGS.0;
I_Butt_Open_Fix                AT %PLCFLAGS.0;
I_Butt_Close_Fix               AT %PLCFLAGS.0;
					          
I_Support_1_Dwn                AT %PLCFLAGS.1;  
I_Support_2_Dwn                AT %PLCFLAGS.1;  
I_Support_3_Dwn                AT %PLCFLAGS.1;  
I_Support_4_Dwn                AT %PLCFLAGS.1;  
I_Support_5_Dwn                AT %PLCFLAGS.1; 
I_Support_6_Dwn                AT %PLCFLAGS.1;
I_Support_7_Dwn                AT %PLCFLAGS.1;

I_Support_1_Dwn_Un             AT %PLCFLAGS.0;  
I_Support_2_Dwn_Un             AT %PLCFLAGS.0; 
I_Support_3_Dwn_Un             AT %PLCFLAGS.0; 
I_Support_4_Dwn_Un             AT %PLCFLAGS.0; 
I_Support_5_Dwn_Un             AT %PLCFLAGS.0; 
I_Support_6_Dwn_Un             AT %PLCFLAGS.0;
I_Support_7_Dwn_Un             AT %PLCFLAGS.0;

I_Support_1_Up                 AT %PLCFLAGS.0;        
I_Support_2_Up                 AT %PLCFLAGS.0;        
I_Support_3_Up                 AT %PLCFLAGS.0;        
I_Support_4_Up                 AT %PLCFLAGS.0;        
I_Support_5_Up                 AT %PLCFLAGS.0;        
I_Support_6_Up                 AT %PLCFLAGS.0;        
I_Support_7_Up                 AT %PLCFLAGS.0;        

I_Support_1_Up_Un              AT %PLCFLAGS.0;       
I_Support_1_Up_Un_aux          AT %PLCFLAGS.0;
I_Support_2_Up_Un              AT %PLCFLAGS.0;       
I_Support_3_Up_Un              AT %PLCFLAGS.0;       
I_Support_4_Up_Un              AT %PLCFLAGS.0;       
I_Support_5_Up_Un              AT %PLCFLAGS.0;       
I_Support_6_Up_Un              AT %PLCFLAGS.0;       
I_Support_7_Up_Un              AT %PLCFLAGS.0;       

I_B1_B2_OPEN                   AT %PLCFLAGS.1;
I_B1_B2_CLOSE                  AT %PLCFLAGS.1;
I_B1_B2_ALARM                  AT %PLCFLAGS.0;
        
I_FC_ZunSaw_POS                AT %PLCFLAGS.0;
I_FC_ZunSwa_NEG                AT %PLCFLAGS.0;
I_FC_Zun_73_POS                AT %PLCFLAGS.0;
I_FC_Zun_73_NEG                AT %PLCFLAGS.0;
I_FC_Zun_74_POS                AT %PLCFLAGS.0;
I_FC_Zun_74_NEG                AT %PLCFLAGS.0;
I_FC_Zun_75_POS                AT %PLCFLAGS.0;
I_FC_Zun_75_NEG                AT %PLCFLAGS.0;

O_Ev_Support1_Up               AT %M5.0; 
O_Ev_Support1_Dwn              AT %M5.0; 
O_Ev_Support2_Up               AT %M5.0; 
O_Ev_Support2_Dwn              AT %M5.0; 
O_Ev_Support3_Up               AT %M5.0; 
O_Ev_Support3_Dwn              AT %M5.0; 
O_Ev_Support4_Up               AT %M5.0; 						       
O_Ev_Support4_Dwn              AT %M5.0; 
O_Ev_Support5_Up               AT %M5.0; 
O_Ev_Support5_Dwn              AT %M5.0; 
O_Ev_Support6_Up               AT %M5.0;
O_Ev_Support6_Dwn              AT %M5.0;
O_Ev_Support7_Up               AT %M5.0;
O_Ev_Support7_Dwn              AT %M5.0;
						       
O_Ev_Support1_Up_Un            AT %M5.0;
O_Ev_Support1_Dwn_Un           AT %M5.0;
O_Ev_Support2_Up_Un            AT %M5.0; 
O_Ev_Support2_Dwn_Un           AT %M5.0; 
O_Ev_Support3_Up_Un            AT %M5.0; 
O_Ev_Support3_Dwn_Un           AT %M5.0; 
O_Ev_Support4_Up_Un            AT %M5.0; 
O_Ev_Support4_Dwn_Un           AT %M5.0; 
O_Ev_Support5_Up_Un            AT %M5.0; 
O_Ev_Support5_Dwn_Un           AT %M5.0; 
O_Ev_Support6_Up_Un            AT %M5.0;
O_Ev_Support6_Dwn_Un           AT %M5.0;
O_Ev_Support7_Up_Un            AT %M5.0;
O_Ev_Support7_Dwn_Un           AT %M5.0;

O_Mobile_Spindle_Slave_Brake   AT %M5.0;

O_Loader_Back                  AT %M5.0;

O_FlapL2_Home                  AT %M5.0; * garbage
O_Flap_Home                    AT %M5.0; * L1 incline
O_SawClampY_Open               AT %M5.0;
O_SawClampY_Close              AT %M5.0;
O_SawClampZ_Open               AT %M5.0;
O_SawClampZ_Close              AT %M5.0;
O_SawClampZ_Rot_Open           AT %M5.0;
O_SawClampZ_Rot_Close          AT %M5.0;
O_FlapL3_Home                  AT %M5.0;
O_FlapL4_Home                  AT %M5.0;
O_FlapL4_Incline               AT %M5.0;
O_FlapL5_Home                  AT %M5.0;
O_FlapL5_Incline               AT %M5.0;

O_Ev_Support72_Up              AT %M5.0;
O_Ev_Support72_Dwn             AT %M5.0;
O_Ev_Support71_Up              AT %M5.0;
O_Ev_Support71_Dwn             AT %M5.0;
O_Ev_Support73_Up              AT %M5.0;
O_Ev_Support73_Dwn             AT %M5.0;
O_Ev_Support74_Up              AT %M5.0;
O_Ev_Support74_Dwn             AT %M5.0;
O_Ev_Support75_Up              AT %M5.0;
O_Ev_Support75_Dwn             AT %M5.0;

