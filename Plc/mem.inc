*******************************************************************************
*   MEM.INC
*   ESAutomotion
*******************************************************************************

PLC_SIMULATED               AT %M33;            * (=1)PLC simulated [RESERVED !!!]

*** Anlog INPUTs
IA_A18_19_DIST_LINEAR       AT %M34;            * Capacitive sensor
I_IA_Feed_Override          AT %M35;            * Potenciometer for Feed Override
IA_Feedback_GAS_OUT         AT %M36;            * Gas proportional valve feedback (OUTPUT)
IA_Feedback_GAS_IN_AN       AT %M37;            * Gas proportional valve feedback (INPUT)
IA_FBACK_ECAT_TYPE0         AT %M38;            * Gas proportional valve feedback on bus ecat
IA_Feedb_HOERB_GAS_OUT      AT %M39;            * Hoerbiger gas proportional valve feedback (OUTPUT)
IA_Feedback_GAS_IN_DIG      AT %M40;            * Hoerbiger gas proportional valve feedback (INPUT)
IA_A18_19_DIST_LINEAR_FIR   AT %M41;            * Capacitive sensor filtered (for RTCP control)

*** Anlog OUTPUTSs
OA_GAS                      AT %M45;            * Gas analog valve
O_OA_GAS_HOERB              AT %M46;            * Gas analog valve
OA_GAS_ECAT_TYPE0           AT %M47;            * Gas valve on bus ecat

*** Emergency ....
I_I_EMERGENZA               AT %M50.0;          * Machine OK (no EMERG.)

*** Buttons and keys
I_I_PUL_START               AT %M51.0;          * Start
I_I_PUL_STOP                AT %M51.1;          * Stop
I_I_PUL_RESET               AT %M51.2;          * Reset
I_I_HW_START                AT %M51.3;          * Start (handwheel)
I_I_HW_STOP                 AT %M51.4;          * Stop (handwheel)
I_I_HW_RESET                AT %M51.5;          * Reset (handwheel)
I_I_HW_JOGPLUS              AT %M51.6;          * Jog+ (handwheel)
I_I_HW_JOGMINUS             AT %M51.7;          * jog- (handwheel)
I_I_JOY_X_P                 AT %M51.8;          * X+ (Joystick)
I_I_JOY_Y_P                 AT %M51.9;          * Y+ (Joystick)
I_I_JOY_X_M                 AT %M51.10;         * X- (Joystick)
I_I_JOY_Y_M                 AT %M51.11;         * Y- (Joystick)
I_I_PUL_TORCH_UP            AT %M51.12;         * Jog+ pushbutton (Torch Up)
I_I_PUL_TORCH_DW            AT %M51.13;         * Jog- pushbutton (Torch Down)
I_I_SEL_REF                 AT %M51.16;         * Sel REF state
I_I_SEL_SINGLEREF           AT %M51.17;         * Sel SINGLE REF state
I_I_SEL_AUT                 AT %M51.18;         * Sel AUT state
I_I_SEL_JOG                 AT %M51.19;         * Sel JOG state
I_I_SEL_INC                 AT %M51.20;         * Sel INC state
I_I_SEL_MDI                 AT %M51.21;         * Sel MDI state

*** Drives ok
I_I_DRIVES_OK               AT %PLCFLAGS.1;           * AT %M52.0;          * All drives are ok
I_I_READY_DRIVE_X           AT %PLCFLAGS.1;           * AT %M52.1;          * Ready signal drive X
I_I_READY_DRIVE_Y           AT %PLCFLAGS.1;           * AT %M52.2;          * Ready signal drive Y
I_I_READY_DRIVE_Z           AT %PLCFLAGS.1;           * AT %M52.3;          * Ready signal drive Z
I_I_READY_DRIVE_UV          AT %PLCFLAGS.1;           * AT %M52.4;          * Ready signal drive U/V
I_I_READY_DRIVE_W           AT %PLCFLAGS.1;           * AT %M52.5;          * Ready signal drive W (external focal)
I_I_READY_DRIVE_C           AT %PLCFLAGS.1;           * AT %M52.6;          * Ready signal drive C
I_I_READY_DRIVE_C1          AT %PLCFLAGS.1;           * AT %M52.7;          * Ready signal drive C1
I_I_READY_DRIVE_C2          AT %PLCFLAGS.1;           * AT %M52.7;          * Ready signal drive C1
I_I_READY_DRIVE_A           AT %PLCFLAGS.1;           * AT %M52.8;          * Ready signal drive A
I_I_READY_DRIVE_B           AT %PLCFLAGS.1;           * AT %M52.9;          * Ready signal drive B
I_I_READY_DRIVE_Z_FOLL      AT %PLCFLAGS.1;          * AT %M52.10;         * Ready signal drive Z_follower


*** Limit switchs
I_I_FC_X1_POS               AT %M53.0;          * X1 maximun limit switch
I_I_FC_X1_NEG               AT %M53.1;          * X1 minimumm limit switch
I_I_FC_Y1_POS               AT %M53.2;          * Y1 maximun limit switch
I_I_FC_Y1_NEG               AT %M53.3;          * Y1 minimumm limit switch
I_I_FC_Z1_POS               AT %M53.4;          * Z1 maximun limit switch
I_I_FC_Z1_NEG               AT %M53.5;          * Z1 minimumm limit switch
I_I_FC_UV_POS               AT %M53.6;          * U/V maximun limit switch
I_I_FC_UV_NEG               AT %M53.7;          * U/V minimumm limit switch
I_I_FC_W1_POS               AT %M53.8;          * W maximun limit switch
I_I_FC_W1_NEG               AT %M53.9;          * W minimumm limit switch
I_I_FC_A1_POS               AT %M53.10;         * A1 maximun limit switch
I_I_FC_A1_NEG               AT %M53.11;         * A1 minimumm limit switch
I_I_FC_B1_POS               AT %M53.12;         * B1 maximun limit switch
I_I_FC_B1_NEG               AT %M53.13;         * B1 minimumm limit switch
I_I_FC_C_POS                AT %M53.14;         * C maximun limit switch
I_I_FC_C_NEG                AT %M53.15;         * C minimumm limit switch
I_I_FC_C1_POS               AT %M53.16;         * C1 maximun limit switch
I_I_FC_C1_NEG               AT %M53.17;         * C1 minimumm limit switch
I_I_FC_V_POS                AT %M53.18;         * U/V maximun limit switch
I_I_FC_V_NEG                AT %M53.19;         * U/V minimumm limit switch
I_I_FC_C2_POS               AT %M53.20;         * C2 maximun limit switch
I_I_FC_C2_NEG               AT %M53.21;         * C2 minimumm limit switch
I_I_FC_Z1_SUP_NEG           AT %M53.22;         * C saw maximun limit switch
I_I_FC_Z2_SUP_NEG           AT %M53.23;         * C saw minimumm limit switch
I_I_FC_Z3_SUP_NEG           AT %M53.24;         * C saw maximun limit switch
I_I_FC_Z4_SUP_NEG           AT %M53.25;         * C saw minimumm limit switch
I_I_FC_W_POS                AT %M53.26;          * W maximun limit switch
I_I_FC_W_NEG                AT %M53.27;          * W minimumm limit switch
I_I_FC_L1_NEG               AT %M53.28;         * C saw maximun limit switch
I_I_FC_L2_NEG               AT %M53.29;         * C saw minimumm limit switch
I_I_FC_L3_NEG               AT %M53.30;          * W maximun limit switch
I_I_FC_L4_NEG               AT %M53.31;          * W minimumm limit switch

*** Reference switch
I_I_INDEX_X1                AT %M54.0;          * X1 Axis Homing Switch
I_I_INDEX_Y1                AT %M54.1;          * Y1 Axis Homing Switch
I_I_INDEX_Z1                AT %M54.2;          * Z1 Axis Homing Switch
I_I_INDEX_UV1               AT %M54.3;          * U/V1 Axis Homing Switch
I_I_INDEX_W1                AT %M54.4;          * W1 Axis Homing Switch
I_I_INDEX_A1                AT %M54.5;          * A1 Axis Homing Switch
I_I_INDEX_B1                AT %M54.6;          * B1 Axis Homing Switch
I_I_INDEX_C                 AT %M54.7;          * C1 Axis Homing Switch
I_I_INDEX_C1                AT %M54.8;          * C1 Axis Homing Switch
I_I_INDEX_V                 AT %M54.9;          * V Axis homing Switch 
I_I_INDEX_C2                AT %M54.10;         * C2 Axis Homing Switch
I_I_WELDING_SENSOR          AT %M54.16;         * Sensor for welding search
I_I_INDEX_W                 AT %M54.17;          * W Axis homing Switch 
I_I_FC_Z1_SUP_POS           AT %M54.18;         * C saw maximun limit switch
I_I_FC_Z2_SUP_POS           AT %M54.19;         * C saw minimumm limit switch
I_I_FC_Z3_SUP_POS           AT %M54.20;         * C saw maximun limit switch
I_I_FC_Z4_SUP_POS           AT %M54.21;         * C saw minimumm limit switch
I_I_FC_L1_POS               AT %M54.22;         * C saw maximun limit switch
I_I_FC_L2_POS               AT %M54.23;         * C saw minimumm limit switch
I_I_FC_L3_POS               AT %M54.24;          * W maximun limit switch
I_I_FC_L4_POS               AT %M54.25;          * W minimumm limit switch
I_I_FC_L5_POS               AT %M54.26;      
I_I_FC_L5_NEG               AT %M54.27;      
*                           AT %M54.28;      
*                           AT %M54.29;      
I_I_PHOTO_CELL              AT %M54.30;
I_I_CHILLER_ERR             AT %M54.31;

I_I_PRESSIONE_ARIA_IMPIANTO AT %M55.0;          * Pressione aria Impianto OK
I_I_PRESSIONE_N2            AT %M55.1;          * Pressione Azoto OK
I_I_PRESSIONE_O2            AT %M55.2;          * Pressione Ossigeno OK
I_I_PRESSIONE_ARIA          AT %M55.3;          * Pressione Aria di taglio OK
I_I_PRESSIONE_ARIA_BANCO    AT %M55.4;          * Pressione aria Banco OK
I_I_LUB_LOW_LEVEL           AT %M55.5;          * Lubrication: low oil level
I_I_LUB_LOW_PRESSURE        AT %M55.9;          * Lubrication: low oil pressure
I_I_LUBRIC_PUMP_PRESSURE    AT %M55.6;          * Lubrication: low pressure pump
I_I_FBACK_ECAT_TYPE0_STAT   AT %M55.7;          * Feedback status valve on bus ecat (if used)
I_I_HOERBIGER_READY         AT %M55.8;          * Hoerbiger ready on bus ecat
I_I_OVER_PRES_LUB           AT %M55.10;          * 
I_I_ASPIRATORE_OK           AT %M55.11;
I_I_Cool_Oil_Alm            AT %M55.12;
I_I_Detec_Weld              AT %M55.13;
I_I_Pic_Weld                AT %M55.14;
I_I_AntiColide              AT %M55.15;
I_I_FC_O4_POS               AT %M55.16;
I_I_FC_O3_POS               AT %M55.17;
I_I_FC_O2_POS               AT %M55.18;
I_I_FC_O1_POS               AT %M55.19;
I_I_FC_O4_NEG               AT %M55.20;
I_I_FC_O3_NEG               AT %M55.21;
I_I_FC_O2_NEG               AT %M55.22;
I_I_FC_O1_NEG               AT %M55.23;

O_O_LUCE_CABINA             AT %M60.0;          * machine light
O_O_LUCE_SU_TESTA_LASER     AT %M60.1;          * head light
O_O_LAMPADA_ROSSA           AT %M60.2;          * Red lamp
O_O_LAMPADA_ARANCIONE       AT %M60.3;          * Orange lamp
O_O_LAMPADA_VERDE           AT %M60.4;          * Green lamp
O_O_CNC_OK                  AT %M60.5;          * CNC OK
O_O_BUZZER_START_CIC        AT %M60.6;          * Star cycle buzzer
O_O_LUB                     AT %M60.7;          * Lubrication
O_O_WELDING_PISTON          AT %M60.8;          * Piston to enable Sensor for welding search
O_O_Guard_Door_1            AT %M60.9;          * Guard Door 1
O_O_Guard_Door_2            AT %M60.10;         * Guard Door 2

O_O_EV_ARIA_UGELLO          AT %M60.15;         * Elettrovalvola aria ugello
O_O_EV_ASSISTENZA_TAGLIO    AT %M60.16;         * Elettrovalvola assistenza taglio
O_O_EV_OPEN_N2              AT %M60.17;         * Elettrovalvola apertura Azoto
O_O_EV_OPEN_O2              AT %M60.18;         * Elettrovalvola apertura Ossigeno
O_O_EV_OPEN_ARIA            AT %M60.19;         * Elettrovalvola apertura Aria taglio
O_O_EV_AUX_GAS              AT %M60.20;         * Elettrovalvola ausiliaria (a valle della proporzionale)
O_O_ENAB_VALVE_HOERB        AT %M60.21;         * Enable proportional valve for oxygen (Hoerbiger) 
O_O_PILOT_HOERB             AT %M60.22;         * Hoerbiger: gas command ev
O_O_CALIB_REQ_HOERB         AT %M60.23;         * Hoerbiger: calibration request
O_O_EV_OPEN_N2_HOERB        AT %M60.24;         * Hoerbiger: N2 ev
O_O_EV_OPEN_O2_HOERB        AT %M60.25;         * Hoerbiger: O2 ev
O_O_EV_OPEN_ARIA_HOERB      AT %M60.26;         * Hoerbiger: air ev
O_O_GAS_ECAT_TYPE0_ENAB     AT %M60.27;         * Enable proportional valve on bus ecat
O_O_HOER_RESET              AT %M60.28;
O_O_CHILLER_ON              AT %M60.29;

O_O_BRAKE_Z1                AT %M61.0;          * Brake Z1 axis
O_O_BRAKE_Z2                AT %M61.1;          * Brake Z2 axis
O_O_RESET_DRIVE_C           AT %M61.2;          * C axis drive reset (disable electric shaft)
O_O_RESET_DRIVE_C1          AT %M61.3;          * C1 axis drive reset (disable electric shaft)
O_O_RESET_DRIVE_C2          AT %M61.4;          * C2 axis drive reset (disable electric shaft)

*** Pipe calibrtion and Support and clamp system
O_O_PIPE_RUN                AT %M62.0;
O_O_SUPPORT_1_DOWN          AT %M62.1;
O_O_SUPPORT_1_UP            AT %M62.2;
O_O_SUPPORT_2_DOWN          AT %M62.3;
O_O_SUPPORT_2_UP            AT %M62.4;
O_O_SUPPORT_3_DOWN          AT %M62.5;
O_O_SUPPORT_3_UP            AT %M62.6;
O_O_SUPPORT_4_DOWN          AT %M62.7;
O_O_SUPPORT_4_UP            AT %M62.8;
O_O_CLAMP_C_CLOSE           AT %M62.9;
O_O_CLAMP_C_OPEN            AT %M62.10;
O_O_CLAMP_C1_CLOSE          AT %M62.11;
O_O_CLAMP_C1_OPEN           AT %M62.12;

*******************************************************************************
*   PLC SETTED
*******************************************************************************
PRGRUN_CN0                  AT %M0.0;           * M0.0 cn0.rc8.0  Programma in corso
PRGRUN_CN2                  AT %M0.1;           * M0.1 cn2.rc8.0  Programma in corso
PRGRUN_CN3                  AT %M0.2;           * M0.2 cn3.rc8.0  Programma in corso
PRGRUN_CN4                  AT %M0.3;           * M0.3 cn4.rc8.0  Programma in corso
PRGRUN_CN0_END              AT %M0.4;           * M0.4 Programma su canale 0 terminato con M30
M30000_CN3                  AT %M0.5;           * M0.5 M30000_CN3: Attendi Servizio per Unit_1[(M10000 iniziolav.) / M10001(finelav.)]
M30010_CN3                  AT %M0.6;           * M0.6 Sblocco Richiesta servizio (M10000_CN0 inizialavorazione)
M30011_CN3                  AT %M0.7;           * M0.7 Sblocco Richiesta servizio (M10001_CN0 finelavorazione)
M10000_CN0                  AT %M0.8;           * M0.8 M10000_CN0: Invoca Unit_1 (inizialavorazione)
M10001_CN0                  AT %M0.9;           * M0.9 M10001_CN0: Invoca Unit_1 (finelavorazione)
EMER_GEN                    AT %M0.10;          * M0.10 Emergenza generale
RETRACERUN                  AT %M0.11;          * M0.11 Ciclo di retrace in Run
REF_RUN                     AT %M0.12;          * M0.12 Taratura in corso
M30_CN0                     AT %M0.13;          * M0.13 M30:EndProgram CN0
M10_CN0                     AT %M0.14;          * M0.14 M10: Attiva Hold + Jog CN0
PRGRUN_CN5                  AT %M0.27;          * M0.27 cn5.rc8.0  Programma in corso
M2_CN0                      AT %M0.28;          * M0.28 M2: EndProgram CN0 M2
*                           AT %M0.31;          * M0.31 NON USARE (TEMPORANEO IN USO PER 2222.pgm

Ordine_Moto_X1              AT %M71.0;          * M71.0 Ordine di moto asse X1
AsseX1_Moto_pos             AT %M71.1;          * M71.1 Asse X1 in moto +
AsseX1_Moto_neg             AT %M71.2;          * M71.2 Asse X1 in moto -
***                        
Ordine_Moto_Y1              AT %M71.8;          * M71.8 Ordine di moto asse Y1
AsseY1_Moto_pos             AT %M71.9;          * M71.9 Asse Y1 in moto +
AsseY1_Moto_neg             AT %M71.10;         * M71.10 Asse Y1 in moto -
***                        
Ordine_Moto_Z1              AT %M71.16;         * M71.16 Ordine di moto asse Z1
AsseZ1_Moto_pos             AT %M71.17;         * M71.17 Asse Z1 in moto +
AsseZ1_Moto_neg             AT %M71.18;         * M71.18 Asse Z1 in moto -

ChannelFeedrate             AT %M72;            * feedrate for channel

BlcAvX1_user                AT %M74.0;          * M74.0 Blocco avanzamento asse X1 da user.plc
BlcAvX1_collis              AT %M74.1;          * M74.1 Blocco avanz. asse X1 per anticollisione
BlcAvX1_Z1_no_OK            AT %M74.2;          * M74.2 Blocco avanz. asse X1 per Asse Z1 non posiz. OK
BlcAvX1_Sens_Capac          AT %M74.3;          * M74.3 Blocco avanz. asse X1 Sens. Capacitivo in contatto con la lamiera
BlcAvY1_user                AT %M74.4;          * M74.4 Blocco avanzamento asse Y da user.plc
BlcAvY1_Z1_no_OK            AT %M74.5;          * M74.5 Blocco avanz. asse Y per Asse Z non posiz. OK
BlcAvY1_Sens_Capac          AT %M74.6;          * M74.6 Blocco avanz. asse Y Sens. Capacitivo in contatto con la lamiera
Ax_Z1_posiz_OK              AT %M74.7;          * M74.7 Asse Z1 in posizione di sicurezza
BL_AV_ASSI_X_OVR            AT %M74.8;          * M74.8 Blocco avanzamento assi dopo Override=0 o NC_STOP
Hold_moving_axW             AT %M74.9;

I_I_BARRIERE_SCARICO_OK     AT %PLCFLAGS.1;         * M81.31 Barriere ottiche scarico lamiera OK
I_I_PLS_RESET_BARRIERE      AT %M89.26;         * M89.26 Pulsante reset barriere
O_O_EV_OR_ALLGAS            AT %M90.15;         * M90.15 Elettrovalvola apertura di almeno un gas

GoInMac1                    AT %M91.0;
GoToUnload1                 AT %M91.1;
GoInMac2                    AT %M91.2;
GoToUnload2                 AT %M91.3;
MoveAx30_Auto               AT %M91.4;
MoveAx31_Auto               AT %M91.5;
IN_M4_MODE                  AT %M91.6;
IN_M3_MODE                  AT %M91.7;
IN_M2_MODE                  AT %M91.8;

* %M92 (reserved !)
* %M93 (reserved !)
* %M94 (reserved !)

EMERGENZA_PLC_CN            AT %M100.0;         * M100.0  Emergenza da PLC o da CN
MACCHINA_OK                 AT %M100.1;         * M100.1  Macchina OK
RESET_MACCHINA              AT %M100.2;         * M100.2  Reset macchina
ALARM_ACT                   AT %M100.3;         * M100.3  Allarme attivo
RESET_ALLARMI               AT %M100.4;         * M100.4  Cancellazione allarmi attiva
Barriere_ottiche_OK         AT %M100.6;         * M100.6  Barriere ottiche in sicurezza
FN_MACCHINA_ON              AT %M100.7;         * M100.7  Fronte di discesa Macchina ON
FP_MACCHINA_ON              AT %M100.8;         * M100.8  Fronte di salita Macchina ON
FP_PS_RESET_BARRIERE        AT %M100.9;         * M100.9  Fronte di salita pulsante Reset Barriere
FN_PS_RESET_BARRIERE        AT %M100.10;        * M100.10  Fronte di discesa pulsante Reset Barriere
GEN_REF_RUN                 AT %M100.16;        * M100.16 Ciclo di Reference GENERALE in corso
InibTipTouchAllarm          AT %M100.22;        * M100.22 Inibizione allarme Tip Touch per restart ciclo
MACCH_IN_MANUALE            AT %M100.24;        * M100.24 SELEZIONE MACCHINA IN MANUALE
MACCH_IN_AUTO               AT %M100.25;        * M100.25 SELEZIONE MACCHINA IN AUTOMATICO
MACCH_IN_SIM                AT %M100.28;        * M100.28 MACCHINA IN SIMULAZIONE
Duty0WhenHoldSup            AT %M100.29;        * M100.29 No laser shoot in case spindle with tube in holding
   
GAS_test                    AT %M107.0;         * M107.0 Test gas in progress for prop. valve
GAS_prun                    AT %M107.1;         * M107.1 Cutting gas active from part program
SPURGOrun                   AT %M107.2;         * M107.2 Gas purge in progress
GAS_prun_close_propv        AT %M107.3;         * M107.3 Cutting gas active from part program (with close prop valve)

ProgRunForGas               AT %M107.10;        * M107.10 Program running for gas managment
GasEnabOnZUp                AT %M107.11;        * M107.11 Gas enable conditions on Z up simulation
AirEnabForAPA               AT %M107.12;        * M107.12 Cutting air enable for APA cycle
PropTestRes                 AT %M107.13;        * M107.13 Proportional valve test reset from gas commands
                                               
PressGasOk                  AT %M107.20;        * M107.20 Gas pressure in range
GasPVDiwal                  AT %M107.21;        * M107.21 Cutting gas prop. valve type lanny diwal
GasPVHoerbiger              AT %M107.22;        * M107.22 Cutting gas prop. valve type hoerbiger
GasPropEnabFromV            AT %M107.23;        * M107.23 Gas prop. valve enable from valve configuration
Hold_gas_refill             AT %M107.24;        * M107.24 Hoerbiger insufficient gas pressure
                                               
Memo_Emission_ON_da_Funz_M  AT %M107.25;        * M107.25 Memoria Emission ON da Funzione M

Prec_Pos_Run                AT %M110.15;        * Focal positioning in run

ContrJogFromBtn             AT %M123.6;
SetPointReached_JumpL3      AT %M189.1;

CMD_C_C1_UNLOCK             AT %M200;           * M200 Unlock gearing C-C1
FBK_C_C1_UNLOCK             AT %M201;           * M201 Feedback gearing C-C1

CMD_HOLDMOVING_CH3          AT %M202;           * M202 Hold moving command axis CH3 (bit0=X, bit1=Y...)
CMD_HOLDMOVING_CH5          AT %M203;           * M203 Hold moving command axis CH5 (bit0=X, bit1=Y...)
CMD_HOLDMOVING_CH8          AT %M207;
CMD_HOLDMOVING_CH9          AT %M208;

CMD_C_C1_OPENLOOP           AT %M204;           * M204 Open loop (bit0=1 C, bit1=1 C1)

CMD_RESONATOR_OFF           AT %M205;           * M205 Resonator OFF

CMD_HOLD_MAC                AT %M206.0;         * M206.0 Hold (CH0, CH3)

CMD_USERISO                 AT %M210.0;         * M210.0 Start UserIso 1000.pgm
CMD_USERISO_END             AT %M210.1;         * M210.1 End UserIso 1000.pgm
GESTAX2_xHoldMoving         AT %M210.2;
GESTAX50_xCloseLoop         AT %M210.3;

CMD_C_C2_UNLOCK             AT %M211;           * M211 Unlock gearing C-C1
FBK_C_C2_UNLOCK             AT %M212;           * M212 Feedback gearing C-C1
GRANT5                     AT %M213;           * GRANT REQ FROM USER
