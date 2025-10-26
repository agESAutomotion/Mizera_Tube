VAR_IN_OUT
SquarePipeCal           AT  %CalPipe.General.0;      * (CalPipe.General.0) Table calibration for square pipe
RoundPipeCal            AT  %CalPipe.General.1;      * (CalPipe.General.1) Table calibration for round pipe
RunCalibration          AT  %CalPipe.General.2;      * (CalPipe.General.2) Run calibration from GUI
*                       AT  %CalPipe.General.3;      * Used in Iol.inc
PipeAlongX              AT  %CalPipe.General.4;      * (CalPipe.General.4) Pipe is cutted along X axis
PipeAlongY              AT  %CalPipe.General.5;      * (CalPipe.General.5) Pipe is cutted along Y axis
EnPipeCalibProgr        AT  %CalPipe.General.6;      * (CalPipe.General.6) Enable auto-pipe calibration from program
FactoryParamCopy        AT  %CalPipe.General.7;      * (CalPipe.General.7) Factory parameters copy 
ResetPipeCal            AT  %CalPipe.General.8;      * (CalPipe.General.8) Button to reset the value of calibration
EnUserOffset            AT  %CalPipe.General.9;      * (CalPipe.General.9) Enable user defined calibration offsets 

PosForSingFC            AT  %CalPipe.General1;       * (CalPipe.General1) Programmed position for singfc command inside spindle calibration
SpindleCalProc          AT  %CalPipe0.General2;      * (CalPipe0.General2) index to comunicate to plc the spindle singfc start
PipeCalCycleTouch       AT  %CalPipe0.General3;      * (CalPipe0.General3) index to communicate in laser touch the setpoint for the calibration
                   
Usr_CorrectionGen       AT  %CalPipe.UsrPara0;
Usr_CorrectionC0        AT  %CalPipe.UsrPara1;
Usr_CorrectionC90       AT  %CalPipe.UsrPara2;
Usr_CorrectionC180      AT  %CalPipe.UsrPara3;
Usr_CorrectionC270      AT  %CalPipe.UsrPara4;
				   
TAB_T_KINE              AT  %tab1.t_yKine;
T_OART_X0               AT  %tab1.t_ttab.t_oart.t_dX;
T_OART_Y0               AT  %tab1.t_ttab.t_oart.t_dY;
T_OART_Z0               AT  %tab1.t_ttab.t_oart.t_dZ;
T_OART_ORIENT           AT  %tab1.t_ttab.t_oart.t_yOrient;
T_OART_FRAME_C          AT  %tab1.t_ttab.t_oart.t_dFrameC;
T_OART_FRAME_B          AT  %tab1.t_ttab.t_oart.t_dFrameB;
T_OART_FRAME_A          AT  %tab1.t_ttab.t_oart.t_dFrameA;
T_OART_EC               AT  %tab1.t_ttab.t_oart.t_dEC;
T_OART_ECREV            AT  %tab1.t_ttab.t_oart.t_bECRev;
T_OART_ECAX             AT  %tab1.t_ttab.t_oart.t_yECAx;
T_OART_X1               AT  %tab1.t_ttab.t_oart.t_dXx;
T_OART_Y1               AT  %tab1.t_ttab.t_oart.t_dYy;
T_OART_TILT             AT  %tab1.t_ttab.t_oart.t_dTilt;
T_OART_EB               AT  %tab1.t_ttab.t_oart.t_dEB;
T_OART_EBREV            AT  %tab1.t_ttab.t_oart.t_bEBRev;
T_OART_EBAX             AT  %tab1.t_ttab.t_oart.t_yEBAx;
T_OART_X2               AT  %tab1.t_ttab.t_oart.t_dXX;
T_OART_Y2               AT  %tab1.t_ttab.t_oart.t_dYY;
T_OART_Z2               AT  %tab1.t_ttab.t_oart.t_dZZ;
T_OART_CAMBER           AT  %tab1.t_ttab.t_oart.t_dCamber;
T_OART_FRAME2_C         AT  %tab1.t_ttab.t_oart.t_dFramebC;
T_OART_FRAME2_B         AT  %tab1.t_ttab.t_oart.t_dFramebB;
T_OART_FRAME2_A         AT  %tab1.t_ttab.t_oart.t_dFramebA;

*** Tabella (tavola) per calibrazione 2D 
T2D_TAB_T_KINE           AT  %tab2.t_yKine;
T2D_T_OART_X0            AT  %tab2.t_ttab.t_oart.t_dX;
T2D_T_OART_Y0            AT  %tab2.t_ttab.t_oart.t_dY;
T2D_T_OART_Z0            AT  %tab2.t_ttab.t_oart.t_dZ;
T2D_T_OART_ORIENT        AT  %tab2.t_ttab.t_oart.t_yOrient;
T2D_T_OART_FRAME_C       AT  %tab2.t_ttab.t_oart.t_dFrameC;
T2D_T_OART_FRAME_B       AT  %tab2.t_ttab.t_oart.t_dFrameB;
T2D_T_OART_FRAME_A       AT  %tab2.t_ttab.t_oart.t_dFrameA;
T2D_T_OART_EC            AT  %tab2.t_ttab.t_oart.t_dEC;
T2D_T_OART_ECREV         AT  %tab2.t_ttab.t_oart.t_bECRev;
T2D_T_OART_ECAX          AT  %tab2.t_ttab.t_oart.t_yECAx;
T2D_T_OART_X1            AT  %tab2.t_ttab.t_oart.t_dXx;
T2D_T_OART_Y1            AT  %tab2.t_ttab.t_oart.t_dYy;
T2D_T_OART_TILT          AT  %tab2.t_ttab.t_oart.t_dTilt;
T2D_T_OART_EB            AT  %tab2.t_ttab.t_oart.t_dEB;
T2D_T_OART_EBREV         AT  %tab2.t_ttab.t_oart.t_bEBRev;
T2D_T_OART_EBAX          AT  %tab2.t_ttab.t_oart.t_yEBAx;
T2D_T_OART_X2            AT  %tab2.t_ttab.t_oart.t_dXX;
T2D_T_OART_Y2            AT  %tab2.t_ttab.t_oart.t_dYY;
T2D_T_OART_Z2            AT  %tab2.t_ttab.t_oart.t_dZZ;
T2D_T_OART_CAMBER        AT  %tab2.t_ttab.t_oart.t_dCamber;
T2D_T_OART_FRAME2_C      AT  %tab2.t_ttab.t_oart.t_dFramebC;
T2D_T_OART_FRAME2_B      AT  %tab2.t_ttab.t_oart.t_dFramebB;
T2D_T_OART_FRAME2_A      AT  %tab2.t_ttab.t_oart.t_dFramebA;

*** Tabella (tavola) per calibrazione 3D 
T3D_TAB_T_KINE           AT  %tab3.t_yKine;
T3D_T_OART_X0            AT  %tab3.t_ttab.t_oart.t_dX;
T3D_T_OART_Y0            AT  %tab3.t_ttab.t_oart.t_dY;
T3D_T_OART_Z0            AT  %tab3.t_ttab.t_oart.t_dZ;
T3D_T_OART_ORIENT        AT  %tab3.t_ttab.t_oart.t_yOrient;
T3D_T_OART_FRAME_C       AT  %tab3.t_ttab.t_oart.t_dFrameC;
T3D_T_OART_FRAME_B       AT  %tab3.t_ttab.t_oart.t_dFrameB;
T3D_T_OART_FRAME_A       AT  %tab3.t_ttab.t_oart.t_dFrameA;
T3D_T_OART_EC            AT  %tab3.t_ttab.t_oart.t_dEC;
T3D_T_OART_ECREV         AT  %tab3.t_ttab.t_oart.t_bECRev;
T3D_T_OART_ECAX          AT  %tab3.t_ttab.t_oart.t_yECAx;
T3D_T_OART_X1            AT  %tab3.t_ttab.t_oart.t_dXx;
T3D_T_OART_Y1            AT  %tab3.t_ttab.t_oart.t_dYy;
T3D_T_OART_TILT          AT  %tab3.t_ttab.t_oart.t_dTilt;
T3D_T_OART_EB            AT  %tab3.t_ttab.t_oart.t_dEB;
T3D_T_OART_EBREV         AT  %tab3.t_ttab.t_oart.t_bEBRev;
T3D_T_OART_EBAX          AT  %tab3.t_ttab.t_oart.t_yEBAx;
T3D_T_OART_X2            AT  %tab3.t_ttab.t_oart.t_dXX;
T3D_T_OART_Y2            AT  %tab3.t_ttab.t_oart.t_dYY;
T3D_T_OART_Z2            AT  %tab3.t_ttab.t_oart.t_dZZ;
T3D_T_OART_CAMBER        AT  %tab3.t_ttab.t_oart.t_dCamber;
T3D_T_OART_FRAME2_C      AT  %tab3.t_ttab.t_oart.t_dFramebC;
T3D_T_OART_FRAME2_B      AT  %tab3.t_ttab.t_oart.t_dFramebB;
T3D_T_OART_FRAME2_A      AT  %tab3.t_ttab.t_oart.t_dFramebA;

*** Tabella Tavola dati di fabbrica
TAB_T_KINE_6            AT  %tab6.t_yKine;
T_OART_X0_6             AT  %tab6.t_ttab.t_oart.t_dX;
T_OART_Y0_6             AT  %tab6.t_ttab.t_oart.t_dY;
T_OART_Z0_6             AT  %tab6.t_ttab.t_oart.t_dZ;
T_OART_ORIENT_6         AT  %tab6.t_ttab.t_oart.t_yOrient;
T_OART_FRAME_C_6        AT  %tab6.t_ttab.t_oart.t_dFrameC;
T_OART_FRAME_B_6        AT  %tab6.t_ttab.t_oart.t_dFrameB;
T_OART_FRAME_A_6        AT  %tab6.t_ttab.t_oart.t_dFrameA;
T_OART_EC_6             AT  %tab6.t_ttab.t_oart.t_dEC;
T_OART_ECREV_6          AT  %tab6.t_ttab.t_oart.t_bECRev;
T_OART_ECAX_6           AT  %tab6.t_ttab.t_oart.t_yECAx;
T_OART_X1_6             AT  %tab6.t_ttab.t_oart.t_dXx;
T_OART_Y1_6             AT  %tab6.t_ttab.t_oart.t_dYy;
T_OART_TILT_6           AT  %tab6.t_ttab.t_oart.t_dTilt;
T_OART_EB_6             AT  %tab6.t_ttab.t_oart.t_dEB;
T_OART_EBREV_6          AT  %tab6.t_ttab.t_oart.t_bEBRev;
T_OART_EBAX_6           AT  %tab6.t_ttab.t_oart.t_yEBAx;
T_OART_X2_6             AT  %tab6.t_ttab.t_oart.t_dXX;
T_OART_Y2_6             AT  %tab6.t_ttab.t_oart.t_dYY;
T_OART_Z2_6             AT  %tab6.t_ttab.t_oart.t_dZZ;
T_OART_CAMBER_6         AT  %tab6.t_ttab.t_oart.t_dCamber;
T_OART_FRAME2_C_6       AT  %tab6.t_ttab.t_oart.t_dFramebC;
T_OART_FRAME2_B_6       AT  %tab6.t_ttab.t_oart.t_dFramebB;
T_OART_FRAME2_A_6       AT  %tab6.t_ttab.t_oart.t_dFramebA;

END_VAR