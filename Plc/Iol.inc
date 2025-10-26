*******************************************************************************
*   IOL.INC
*   ESAutomotion
*   LASER machine
*******************************************************************************
VAR

*** config_machine
#defcom %config_machine0.3      Selezione LASER
#defcom %config_machine0.5      Gestione CARICO/SCARICO
#defcom %config_machine0.6      Gestione LUNETTE
#defcom %config_machine0.7      Motor cylinder composite support type (Loader)
#defcom %config_machine0.8      Motor cylinder composite support type (Unloader)
#defcom %config_machine2.0      LASER - Gestione valvola proporzionale sul bus ECAT
#defcom %config_machine4        LASER - Tipo gestione valv. prop. sul bus ECAT (0=DIWAL,1=HOERBIGER...)
#defcom %config_machine23       Pressione ARIA per gestione APA
#defcom %config_machine25       Head Model (0=Precitec ,1=WSX ,2=HighYag)
#defcom %config_machine26       Capacitive Sensor (0=Precitec ,1=Lasermech,2= HighYag)
#defcom %config_machine27       Pallet changing present
#defcom %config_machine28       Generator selected (0=YLS, 1=YLR)
#defcom %config_machine29       Proportional valve manage only with OXYGEN
#defcom %config_machine30       Focal analog resolution
#defcom %config_machine32       Pipe cutting: 1 = tool, 2 = surface, 6 = Z


*** PlcOp0
#defcom %PlcOp0.2               DryRun
#defcom %PlcOp0.3               Qwerty keyboard enabled
#defcom %PlcOp0.4               Disable load par in G65x
#defcom %PlcOp0.5               Check work table
#defcom %PlcOp0.10              Enable laser pointer (for acquire origin)
#defcom %PlcOp0.13              Alternative nozzle (for bevel)
#defcom %PlcOp0.14              =0 absolute acquiring origin, 1= inc
#defcom %PlcOp0.31              =0 track, 1=piece & geometry management


*** LSRPlcOp
#defcom %LSRPlcOp0.4            Enable Z up after HOLD
#defcom %LSRPlcOp0.5            Worklist mode enabled
#defcom %LSRPlcOp0.7            Enable stop at the end of the track/geometry
#defcom %LSRPlcOp0.8            Z axis in simulation (always in UP position)
#defcom %LSRPlcOp0.9            Parking axes in the end of the track (if LSRPlcOp0.7=1) xPIPE
#defcom %LSRPlcOp0.10           Pipe calibration with touching 
#defcom %LSRPlcOp0.12           Disable TIP TOUCH during piercing
#defcom %LSRPlcOp0.19           Enable stop at the end of the piece
#defcom %LSRPlcOp0.20           FAR INPUT management enabled
#defcom %LSRPlcOp0.22           Pipe Welding function enabling
#defcom %LSRPlcOp1.3            Enable the focal linearization
#defcom %LSRPlcOp1.5            Enable Nozzle cleaning
#defcom %LSRPlcOp1.9            Capacitive linearization from CNC
#defcom %LSRPlcOp1.13           Enable lubrication by minute, default by hour 
#defcom %LSRPlcOp1.16           Enable retrace back after HOLD
#defcom %LSRPlcOp1.22           Enable cooling point


*** LSRCostK
#defcom %LSRCostK31             Capacitive sensor range
#defcom %LSRCostK8              Spindle maximum disaligment


*** Cost_User
#defcom %Cost_User20            Buzzer timer
#defcom %Cost_User22            Clean X axis quota
#defcom %Cost_User24            Clean Y axis quota
#defcom %Cost_User25            Service position: Z axis qta
#defcom %Cost_User26            Service position: X axis qta
#defcom %Cost_User27            Service position: Y axis qta
#defcom %Cost_User28            Service position: X/Y axes speed
#defcom %Cost_User29            Service position: B axis qta



*** SoftBtn
#defcom %SoftBtn0.0             Reset PIECE and GEOMETRY
#defcom %SoftBtn0.8             Spindle aligment
#defcom %SoftBtn0.9             Ciclo di parcheggio componenti caricatore (OMP)
#defcom %SoftBtn0.10            Ciclo caricatore manuale (OMP)
#defcom %SoftBtn0.20            Set nominal curve (calibration checking function)
#defcom %SoftBtn0.21            Start calibraion checking function
#defcom %SoftBtn0.22            Start rtcp control function
#defcom %SoftBtn0.30            Semi Tube Loading
#defcom %SoftBtn0.31            Weld seam identification
#defcom %SoftBtn1.0             Slettore manuale: incremento dato selezionato in "ManSelector"
#defcom %SoftBtn1.1             Slettore manuale: decremento dato selezionato in "ManSelector"


*******************************************************************************
#defcom %FunzRequest               Funzione richiesta per Unit_1
#defcom %C19                       Tabella di lavoro selezionata (G650) (@)

#defcom %TCP0                      a_x
#defcom %TCP1                      tf_a13
#defcom %TCP2                      a_y
#defcom %TCP3                      tf_a23
#defcom %TCP4                      a_z
#defcom %TCP5                      tf_a33

#defcom %Epsilon                  Epsilon
#defcom %OffsetPiercing           Offset set piercing
#defcom %OffsetCut                Offset set cut


*******************************************************************************
#defcom %TstP1                    Velocita` attuale sulla traiettoria (mm/min)
#defcom %TstP2.3                  Sim-faded-out
#defcom %TstP2.4                  Signal-lock

*******************************************************************************
HW_ENCTYPE     AT %cn2.cc1.5;  * HW Type :0=S500, 1=S510
* --------------------------------------------------------------

ch0_in_ref           AT %cn0.rc0.8;       * channel 0 in ref
ch3_in_ref           AT %cn3.rc0.8;       * channel 3 in ref
ch5_in_ref           AT %cn5.rc0.8;       * channel 5 in ref 
                                          
ch0_in_jog           AT %cn0.rc0.9;       * channel 0 in jog
ch3_in_jog           AT %cn3.rc0.9;       * channel 3 in jog
ch5_in_jog           AT %cn5.rc0.9;       * channel 5 in jog

ch0_in_aut           AT %cn0.rc0.12;      * channel 0 in aut
ch3_in_aut           AT %cn3.rc0.12;      * channel 3 in aut
ch5_in_aut           AT %cn5.rc0.12;      * channel 5 in aut
                                          
ch0_in_inc           AT %cn0.rc0.10;      * channel 3 in jnc
ch3_in_inc           AT %cn3.rc0.10;      * channel 3 in inc
ch5_in_inc           AT %cn5.rc0.10;      * channel 5 in inc

ch7_in_ref           AT %cn7.rc0.8;
ch8_in_ref           AT %cn8.rc0.8;
ch9_in_ref           AT %cn9.rc0.8;                                    
                                          
Ch0InAlarm           AT %cn0.rc8.3;       * ch0 in alarm 


*******************************************************************************
* INGRESSI TASTIERA ESA JOYSTICK
*******************************************************************************
XxCTRL_ENAB_JOG      AT %CTRLBTN.1;  * CTRLBTN.1  CTRL che abilita i pulsanti joystick
XxEsaJoy_JOG_Xplus   AT %ARROWBTN.4;  * ARRWBTN.4 Pulsanti Jog Joystick
XxEsaJoy_JOG_Xminus  AT %ARROWBTN.3;  * ARRWBTN.3 Pulsanti Jog Joystick
XxEsaJoy_JOG_Yplus   AT %ARROWBTN.1;  * ARRWBTN.1 Pulsanti Jog Joystick
XxEsaJoy_JOG_Yminus  AT %ARROWBTN.6;  * ARRWBTN.6 Pulsanti Jog Joystick
XxEsaJoy_XY_plus     AT %ARROWBTN.2;  * ARRWBTN.2 Pulsanti Jog Joystick
XxEsaJoy_XY_minus    AT %ARROWBTN.5;  * ARRWBTN.5 Pulsanti Jog Joystick
XxEsaJoy_XplYmin     AT %ARROWBTN.7;  * ARRWBTN.7 Pulsanti Jog Joystick
XxEsaJoy_XminYpl     AT %ARROWBTN.0;  * ARRWBTN.0 Pulsanti Jog Joystick
XxPUL_TAB_Yoy        AT %CTRLBTN.0;   * CTRLBTN.0; Pulsanti Tab Joystick tastiera esa integrata

*******************************************************************************
*    Selezione MODI OPERATIVI DA TASTIERA ESA
*******************************************************************************
* Seleziona Hardware S510
XxS510_PulSelREF         AT %EXTIN.0;      * EXTIN.0 REF   Mode from S510 Keyboard
XxS510_PulSelAUT         AT %EXTIN.4;      * EXTIN.4 AUTO  Mode from S510 Keyboard
XxS510_PulSelJOG         AT %EXTIN.1;      * EXTIN.1 JOG   Mode from S510 Keyboard
XxS510_PulSelINC         AT %EXTIN.2;      * EXTIN.2 INC   Mode from S510 Keyboard
XxS510_PulSelMDI         AT %EXTIN.3;      * EXTIN.3 MDI   Mode from S510 Keyboard
XxS510_P_UpTorcia        AT %EXTBTN3.0;    * EXTBTN2.7 Torch UP from S510 Keyboard
XxS510_P_DwnTorcia       AT %EXTBTN3.1;    * EXTBTN3.0 Torch DN from S510 Keyboard

* Seleziona Hardware S500
XxS500_PulSelREF         AT %EXTBTN3.2;    * EXTBTN3.2 REF   Mode from S500 Keyboard
XxS500_PulSelAUT         AT %EXTBTN3.3;    * EXTBTN3.3 AUTO  Mode from S500 Keyboard
XxS500_PulSelJOG         AT %EXTBTN3.4;    * EXTBTN3.4 JOG   Mode from S500 Keyboard
XxS500_PulSelINC         AT %EXTBTN3.5;    * EXTBTN3.5 INC   Mode from S500 Keyboard
XxS500_PulSelMDI         AT %EXTBTN3.6;    * EXTBTN3.6 MDI   Mode from S500 Keyboard
XxS500_P_UpTorcia        AT %EXTBTN2.3;    * EXTBTN2.3 Torch UP from S500 Keyboard
XxS500_P_DwnTorcia       AT %EXTBTN2.2;    * EXTBTN2.2 Torch DN from S500 Keyboard

*******************************************************************************
* ASSICIAZIONI A REGISTRI DI STATO CANALE ASSI
*******************************************************************************
PRGSTOP_CN0         AT %cn0.rc8.1;    *(cn0.rc8.1)  Programma interrotto CN0
PRGSTOP_CN2         AT %cn2.rc8.1;    *(cn2.rc8.1)  Programma interrotto CN2
PRGSTOP_CN3         AT %cn3.rc8.1;    *(cn3.rc8.1)  Programma interrotto CN3

LATENCY_L_TO_H      AT %ax21.ra77;    *(main_proc.ra77)Latenza Attivazione DigitalBeam FlyCut
LATENCY_H_TO_L      AT %ax21.ra78;    *(main_proc.ra78)Latenza Disattivazione DigitalBeam FlyCut
EN_T_ALIGNED_SETIO  AT %cg3.23;       *(cg3.23)Opz.CN "Ab.Allineamento SetIo tramite Timer Hardware"


*******************************************************************************
*   @@@BLOCK SEARCH (RICERCA BLOCCO)
*******************************************************************************
CURRENT_LINE     AT %R12;     * Numero riga corrente del part program in esecuzione
MEM_XQUOTE       AT %R13;     * R13 Memorizza quota X per collimazione ricerca blocco
MEM_YQUOTE       AT %R14;     * R14 Memorizza quota Y per collimazione ricerca blocco
MEM_INTPROG      AT %R15.0;   * R15.0 Mem. programma valido per ripresa pezzo
BlackOut         AT %R55.31;  * (R55.31)Interruzione per BlackOut

GRAPH_DRAW_RUN   AT %ui33.7;        * ui33.7 Graphics drawing in progress
GRAPH_DRAW       AT %ui33.10;       * ui33.10 Graphics drawing
COLLIMATE_OK     AT %ui33.31;       * ui33.31 Collimate and point collimate or path collimate done (block search)

* Da ISO a PLC - Registro C18 --------------------------------------------------
****************************************************************************************************
**Assegnazioni registri di Comando \ Sincronismi Generali da ISO su %C18 ***************************
****************************************************************************************************
* ..\CNC\C\ISO\-1840.cfs/124:     *(C18.0)  Spegnimento anticipato
* ..\CNC\C\ISO\fnzm82.srt/5:      *(C18.1)  Congela regolazione
AcqOrigEnd      AT %C18.2;              *(C18.2)  Acquisizione origine effettuata

*ParkT2           AT %C18.6; * (C18.6)Richiesta Fuori ingombro testa a  (cn4)
AcqOrigOn       AT %C18.7;              *(C18.7)  Acquisizione origine in corso
TastInCorso2    AT %uxh_PlIso2.8;       *(uxh_PlIso2.4)Tastatura in corso (fase di rilascio)
*                                       *(C18.11) N.U.
*                                       *(C18.12) N.U.
*                                       *(C18.13) N.U.
* ..\CNC\C\ISO\-1800.cfs/291:     *(C18.16) = 1
CapacitiveTestRun   AT %C18.17;         *(C18.17) Capacitive test running
IncaseofbevelCut    AT %C18.18;         *(C18.18) G800 G11 G13
*                                       *(C18.19) N.U.
*                                       *(C18.20) N.U.
*                                       *(C18.21) N.U.
Cycle_81_84Active AT %C18.22;           *(C18.22) Ciclo G81/G84 in esecuzione
*                                       *(C18.25)
APAinRun        AT %C18.31;             *(C18.31) APA in run (per gestione aria durante APA)



ISO_I_FAR_A10                   AT %C31.0;      *(C31.0) FAR input (used in ISO, into calibration fun)
ISO_I_TIP_BODY_TOUCH_A11        AT %C31.1;      *(C31.1) Tip-Body Touch o Nozzle_Lost (used in ISO, into calibration fun)
ISO_I_READY_A13		            AT %C31.2;      *(C31.2) READY (used in ISO, into calibration fun)
ISO_I_NOZZLE_LOST_A15           AT %C31.3;      *(C31.3) POS reached (used in ISO, into calibration fun)


****************************************************************************************************
*                                     GENERICI DI MACCHINA                                         *
* ------------------------------------------------------------------------------------------------ *
****************************************************************************************************
Sel_INC_orig        AT %PlcOp0.14; * (PlcOp0.14)Selezione acquisizione Manuale Origine Incrementale
AuxFaseEnd          AT %cn0.rc8.24; * cn0.rc8.24 Fase Posizionamento AuxComand Terminata
****************************************************************************************************
*                                   (Plc Generali macchina)
*                     Variabili scritte da PLC per scambio dati con ISO e GUI
* ------------------------------------------------------------------------------------------------ *
****************************************************************************************************
*gPlc0
Gest5Ax             AT %gPlc0.0;  *(gPlc0.0)Gestione MAcchina 5 assi
Pul_Start           AT %gPlc0.1;  *(gPlc0.1)Start pushbutton
Pul_Stop            AT %gPlc0.2;  *(gPlc0.2)Stop pushbutton
Pul_Reset           AT %gPlc0.3;  *(gPlc0.3)Reset pushbutton
Pul_Jogp            AT %gPlc0.4;  *(gPlc0.4)jog+ pushbutton
Pul_Jogm            AT %gPlc0.5;  *(gPlc0.5)jog- pushbutton
R_TrigReset         AT %gPlc0.6;  *(gPlc0.6)Fronte Up Pul Reset
M40000_CN4          AT %gPlc0.8;  *(gPlc0.8)M40000_CN4: Attendi Servizio per Unit_2[]
***M40001_CN4          AT %gPlc0.8;  * ((gPlc0.8))M40001_CN4: Servizio eseguito da Unit_2[]
MACC_TARATA         AT %gPlc0.9;  *(gPlc0.9)Assi tarati
MACC_ALARM          AT %gPlc0.10; *(gPlc0.10)Macchina in allarme
Ack_Mfun_PLC        AT %gPlc0.11; *(gPlc0.11)Sblocco funzioni M da PLC.plc
Stop_Req_PLC        AT %gPlc0.12; *(gPlc0.12)Richiesta stop ciclo da PLC.plc
Blav_Req_PLC        AT %gPlc0.13; *(gPlc0.13)Richiesta blocco avanzamento XY da PLC.plc
JOY_X_P             AT %gPlc0.14; *(gPlc0.14)Joystik X + selezionato
JOY_X_M             AT %gPlc0.15; *(gPlc0.15)Joystik X - selezionato
JOY_Y_P             AT %gPlc0.16; *(gPlc0.16)Joystik Y + selezionato
JOY_Y_M             AT %gPlc0.17; *(gPlc0.17)Joystik Y - selezionato
EnJogRtcp           AT %gPlc0.18; *(gPlc0.18)Abilitazione Jog RTCP tuning
REPOS_ON            AT %gPlc0.19; *(gPlc0.19)Comando Repos Attivo
REPOS_ON_ICON       AT %gPlc0.20; *(gPlc0.20)Visualizza stato Icona REPOS
JOG_ENABLE_ICON     AT %gPlc0.21; *(gPlc0.21)Visualizza stato Icona JOG_ENABLE
JOG_ENABLE          AT %gPlc0.22; *(gPlc0.22)Jog Enable
Pul_Reset_EMERG     AT %gPlc0.23; *(gPlc0.23)Tasto reset premuto pi� di 4 secondi per SETALARM canali
*                   AT %gPlc0.24; *(gPlc0.24)
EnabPresTube        AT %gPlc0.25; *(gPlc0.25)Abilita Preset Tubo
ReqPresTube         AT %gPlc0.26; *(gPlc0.26)Richiesta Preset Tubo
PAG_ORIG_SEL        AT %gPlc0.27; *(gPlc0.27)Pagina Origini Selezionata
StartAuxMode        AT %gPlc0.28; *(gPlc0.28)Rich.START per funzioni ausiliarie (1000.pgm)
ExeAuxMode          AT %gPlc0.29; *(gPlc0.29)Esecuzione ausiliaria 1000.PGM
Pon                 AT %gPlc0.30; *(gPlc0.30) Power on main axis
GeneralCloseLoop    AT %gPlc0.31; *(gPlc0.31) Allow general close loop


*gPlc1
QtaMastAx           AT %gPlc1;    * (gPlc1) Quota Asse master della coppia in Gantry

*gPlc2
Pres_Allin_GantryOk AT %gPlc2.0;  *(gPlc2.0)Gantry Allineato o eseguito preset Quota Master su slave
EnAcqusQtaCorrez    AT %gPlc2.1;  *(gPlc2.1)Abilita Acquis. correz. Quota di taratura asse slave Gantry
PresetDaErrore      AT %gPlc2.2;  *(gPlc2.2)Rich. Preset/Allineamento quota slave a master per errore allin.
GantryOn_UIPL       AT %gPlc2.3;  *(gPlc2.3)Gantry attivo per visualizzazione stato in Sinottico
GantryExec          AT %gPlc2.4;  *(gPlc2.4)Gentry Abilitato (Informazione per GUI)
RefAlarm            AT %gPlc2.5;  *(gPlc2.5) Errore allineamento durante ciclo zero(Gantry)
EndCycRefGantry     AT %gPlc2.6;  *(gPlc2.6) Fine ciclo taratura gantry
JOY_U_P             AT %gPlc2.7;  *(gPlc2.7)Joystik U + selezionato
JOY_U_M             AT %gPlc2.8;  *(gPlc2.8)Joystik U - selezionato
JOY_C_P             AT %gPlc2.9;  *(gPlc2.9)Joystik C + selezionato
JOY_C_M             AT %gPlc2.10; *(gPlc2.10)Joystik C - selezionato

TecLsrActive        AT %gPlc2.11; *(gPlc2.11)Tecnologia Laser selezionata
HoldReqAUTO         AT %gPlc2.12; *(gPlc2.12)Mem. Richiesta HOLD in Automatico
ApaPlateSense       AT %gPlc2.13; *(gPlc2.13)Sensore Rilevamento lamiera per "APA"
PhotocellsOn        AT %gPlc2.15; *(gPlc2.15)Photocells on
RICBLOCCO           AT %gPlc2.19; *(gPlc2.19)Comando Ricerca Blocco
ForceSimulaPls      AT %gPlc2.20; *(gPlc2.20)Forxamento simulazione per VRTC in fase di Ricerca blocco
RicBloccoActive     AT %gPlc2.21; *(gPlc2.21) Memoria ricerca blocco attiva
StopUsrPlc          AT %gPlc2.22; *(gPlc2.22)Richiesta STOP da User_Plc
I_HW_X_SEL_USR      AT %gPlc2.23; *(gPlc2.23)X selected da volantino pulsantiera ausiliaria
I_HW_Y_SEL_USR      AT %gPlc2.24; *(gPlc2.24)Y selected da volantino pulsantiera ausiliaria
I_HW_Z_SEL_USR      AT %gPlc2.25; *(gPlc2.25)Z selected da volantino pulsantiera ausiliaria
I_HW_RATE_X100_USR  AT %gPlc2.26; *(gPlc2.26)rate x100   da volantino pulsantiera ausiliaria
I_HW_RATE_X1000_USR AT %gPlc2.27; *(gPlc2.27)rate x1000  da volantino pulsantiera ausiliaria
PulsAuxComand       AT %gPlc2.28; *(gPlc2.28)Comando abilitazione da pulsantiera ausiliaria
GUI_HW_ENAB         AT %gPlc2.29; *(gPlc2.29)Abilitazione comandi da volantino da GUI
I_HW_Z1_SEL_USR     AT %gPlc2.30; *(gPlc2.30)Z selected da volantino pulsantiera ausiliaria
SelAx_Cn0           AT %gPlc2.31; *(gPlc2.31)Asse selezionato per CN0

*gPlc (DW)
QtaAfterSizeM       AT %gPlc3;    *(gPlc3)Quota Master Gantry dopo taratura
QtaAfterSizeS       AT %gPlc4;    *(gPlc4)Quota Slave Gantry dopo taratura
GuiModeSelect       AT %gPlc5;    *(gPlc5)Selez. Modo Operativo a GUI Per visualizzaz.LIST.BOX
VisListBoxMSG       AT %gPlc6;    *(gPlc6)Visualizzazione messaggi nella ListBox in pagina SetUp Gantry
OrigineX            AT %gPlc7;    *(gPlc7)Origine attiva X
OrigineY            AT %gPlc8;    *(gPlc8)Origine attiva Y
VisLedOrigGui       AT %gPlc9;    *(gPlc9)Visualizzaz. led Origine attiva
CycleTime           AT %gPlc12;   *(gPlc12)Tempo ciclo (s)

*gPlc13
ReqAcqOrig          AT %gPlc13.1; *(gPlc13.1)Richiesta acquisizione origine
HEADZSEL            AT %gPlc13.4; *(gPlc13.4)Testa 1 selezionata
HEADZ1SEL           AT %gPlc13.5; *(gPlc13.5)Testa 2 selezionata
Z_Z1_SEL            AT %gPlc13.6; *(gPlc13.6)Testa Z + Z1 selezionata
AxStopSincGRP       AT %gPlc13.11; *(gPlc13.11)Assi Fermi per sincronismo GRP[ax]
CS_X_Overtravel     AT %gPlc13.12; *(gPlc13.12)Camma sw asse X oltre limite max
Ack_Mfun_UsrM       AT %gPlc13.13; *(gPlc13.13) Unlock M-Functions from User_M
AxStop_ForUser      AT %gPlc13.14; *(gPlc13.14) Info Assi fermi a User
StartCycleUserReq   AT %gPlc13.15; *(gPlc13.15) Richiesta START ciclo da Logica "User.plc"
StopCycleUserReq    AT %gPlc13.16; *(gPlc13.16) Richiesta STOP ciclo da Logica "User.plc"
ResetCycleUserReq   AT %gPlc13.17; *(gPlc13.17) Richiesta RESET ciclo da Logica "User.plc"
GantryExec2         AT %gPlc13.19;  *(gPlc13.19)Gentry Abilitato (Informazione per GUI)
EndCycRefGantry2    AT %gPlc13.20;  *(gPlc13.20)Fine ciclo taratura gantry
RefAlarm2           AT %gPlc13.21;  *(gPlc13.21)Alarma of pallet gantry
Pres_Allin_GantryOk2 AT %gPlc13.22; *(gPlc13.22)Gantry Allineato o eseguito preset Quota Master su slave
GantryOn_UIPL2      AT %gPlc13.23; *(gPlc13.23)Gantry attivo per visualizzazione stato in Sinottico
ReqPuliziaUgello    AT %gPlc13.24; *(gPlc13.24)Richiesta routine di pulizia ugello su spazzola
SelForaS            AT %gPlc13.25; * (gPlc13.25)Selezione Asse S da Pagina Foratrice selezionata
ReqCapacitveTest    AT %gPlc13.27; *(gPlc13.27) Capacitive test request
Pall_Coll           AT %gPlc13.28; *(gPlc13.28) Possible pallet collision
ReqServicePosition  AT %gPlc13.29; *(gPlc13.29) Request go Axes to Service Position


*gPlc14 (DW)
RotazOrig           AT %gPlc15;   *(gPlc15)Rotazione origine attiva
AxesOverride        AT %gPlc16;   *(gPlc16) feed degli assi (%)

VisSpeed_Y          AT %gPlc17;   *(gPlc17)Velocita` asse Y
VisSpeed_V          AT %gPlc18;   *(gPlc18)Velocita` asse V

M_HOLD              AT %gPlc19.0; *(gPlc19.0)Mem. Richiesta Hold
usr_RESET           AT %gPlc19.1; *(gPlc19.1)Request for RESET from User PLC
usr_HoldEmissionOn  AT %gPlc19.2; *(gPlc19.2)Request for LASER Emission ON from User PLC
JOY_Z_P             AT %gPlc19.3; *(gPlc19.3)Joystik Z + selezionato
JOY_Z_M             AT %gPlc19.4; *(gPlc19.4)Joystik Z - selezionato


DwnL4FromU                  AT %gPlc19.12;   *(gPlc19.12)  Event move "out of space L4"
DwnL5FromU                  AT %gPlc19.13;   *(gPlc19.13)  Event move "out of space L5"
Down_L4                     AT %gPlc19.14;   *(gPlc19.14)  Move out of space L4
Down_L5                     AT %gPlc19.15;   *(gPlc19.15)  Move out of space L5
mem_GEAR_L1_L3              AT %gPlc19.16;   *(gPlc19.16)  mem gearing L1 L3	
mem_GEAR_L1_L2				AT %gPlc19.17;   *(gPlc19.17)  mem gearing L1 L2																	  
Lev_L2_L3                   AT %gPlc19.18;   *(gPlc19.18)  leveling request L1 L2 L3
Down_L2                     AT %gPlc19.19;   *(gPlc19.19)  Move out of space L2
Down_L3                     AT %gPlc19.20;   *(gPlc19.20)  Move out of space L3
Down_L1                     AT %gPlc19.21;   *(gPlc19.21)  Move out of space L1
Up_L1                       AT %gPlc19.22;   *(gPlc19.22)  Move Up L1
DwnL1FromU                  AT %gPlc19.23;   *(gPlc19.23)  Event move "out of space L1"
DwnL2FromU                  AT %gPlc19.24;   *(gPlc19.24)  Event move "out of space L2"
DwnL3FromU                  AT %gPlc19.25;   *(gPlc19.25)  Event move "out of space L3"
Ack_Mfun_UsrM_GestCpCy      AT %gPlc19.26;   *(gPlc19.26)  Unlock M-Functions 
Up_V1                       AT %gPlc19.27;   *(gPlc19.27)  Move Up V1
Down_V1                     AT %gPlc19.28;   *(gPlc19.28)  Move out of space V1
DwnV1FromU                  AT %gPlc19.29;   *(gPlc19.29)  Event move "out of space V1"
mem_GEAR_C_C1               AT %gPlc19.30;   *(gPlc19.30)  mem of gearing C and C1
mem_GEAR_C_C2               AT %gPlc19.31;   *(gPlc19.31)  mem of gearing C and C2


SpurgoGas           AT %gPlc23.0;  * (gPlc23.0) Spurgo gas in corso
HoldXpalletAxisUP   AT %gPlc23.1;  * (gPlc23.1) Blocco avanzamento asse X da pallet 1
HoldXpalletAxisDwn  AT %gPlc23.2;  * (gPlc23.2) Blocco avanzamento asse X da pallet 2
Xaxis52Selected     AT %gPlc23.3;  * (gPlc23.3) Asse X pallet selezionato
Hold_req            AT %gPlc23.4;  * (gPlc23.4) request hold          
plc_EnblLockDoor    AT %gPlc23.5;  * GUI Enable pushbutton Lock Operator Door  
plc_EnblUnlockDoor  AT %gPlc23.6;  * GUI Enable pushbutton Unlock Operator Door
plc_DoorLocked_OK   AT %gPlc23.7;  * PLC Confirmation for Operator Door locked 
HoldButtonPressed   AT %gPlc23.30; *(gPlc19.30)
StartButtonPressed  AT %gPlc23.31; *(gPlc19.31)

ResumeProgEnab      AT %gPlc25.0;  *(gPlc25.0) Data avaiable for resume program

Res_Axes            AT %gPlc25.1;  *(gPlc25.1) reset axes disable electric shaft
Res_Ch3             AT %gPlc25.2;  *(gPlc25.2) reset channel 3
Res_Ch0             AT %gPlc25.3;  *(gPlc25.3) reset channel 0
ResetMach           AT %gPlc25.4;  *(gPlc25.4) reset macchina

IncDecStepCutFocal  AT %gPlc26;     *(gPlc26) Inc dec step of cut focale parameter (LASER)
IncDecStepCutDist   AT %gPlc27;     *(gPlc27) Inc dec step of cut distance (standoff) parameter (LASER)
            
ManSelectorPOW_out  AT %gPlc30;     *(gPlc30)Selettore manuale POTENZA: ultimo valore impostato (valore di uscita per LsrGest)
ManSelectorDUTY_out AT %gPlc31;     *(gPlc30)Selettore manuale DUTY: ultimo valore impostato (valore di uscita per LsrGest)
ManSelectorFREQ_out AT %gPlc32;     *(gPlc30)Selettore manuale FREQUENCY: ultimo valore impostato (valore di uscita per LsrGest)
            
InterpolatResidual  AT %gPlc40;  
Sel_Z_Ch8           AT %gPlc41.0;  
Sel_Y_Ch8           AT %gPlc41.1;
Sel_U_Ch8           AT %gPlc41.2;
Sel_V_Ch8           AT %gPlc41.3;
Sel_W_Ch8           AT %gPlc41.4;     
Sel_Y_Ch9           AT %gPlc41.5;
Sel_U_Ch9           AT %gPlc41.6;
Sel_V_Ch9           AT %gPlc41.7;
Sel_Z_Ch9           AT %gPlc41.8;   

Sel_X_Ch6           AT %gPlc41.9;   
Sel_Y_Ch6           AT %gPlc41.10;   
Sel_Z_Ch6           AT %gPlc41.11;   
Sel_U_Ch6           AT %gPlc41.12;   
Sel_V_Ch6           AT %gPlc41.13;   
Sel_W_Ch6           AT %gPlc41.14;   

Sel_Z_Ch7           AT %gPlc41.15; 
Sel_U_Ch7           AT %gPlc41.16;   
Sel_V_Ch7           AT %gPlc41.17;   
Sel_W_Ch7           AT %gPlc41.18;   
TON_ABIL_PON        AT %gPlc41.19;  
                       
b_TipTouch          AT %LsPlc47.7;  ** (LsPlc47.7) TipTouch Attivo da MainFast

exeaux_for_mancut   AT %LsPlc47.8;  ** (LsPlc47.8) Manual cut request
RetrPlusReq         AT %LsPlc47.9;  ** (LsPlc47.9) Request of positive retrace
RetrMinusReq        AT %LsPlc47.10; ** (LsPlc47.10) Request of negative retrace
RetraceCommBack     AT %LsPlc47.11; ** (LsPlc47.11) Retrace command after hold
NozzleLosePlate     AT %LsPlc47.13; ** (LsPlc47.13) Nozzle is far more then threshold
M4001_REQ           AT %LsPlc47.14; ** (LsPlc47.14) M4001 request to enable height threshold detection
ExeApaRequest       AT %LsPlc47.15; ** (LsPlc47.15) Apa request from user
BreakPosReq         AT %LsPlc47.17; ** (LsPlc47.17) Request break point positioning
Hold_from_pallet_usr  AT %LsPlc47.18;  *(LsPlc47.18) HOLD request form pallet exch.
TurnOffLaser        AT %LsPlc47.19;   * (LsPlc47.19) Turn off laser from microjoint algorithm
AxesAreAbsolute     AT %LsPlc47.20;  * (LsPlc47.19) All axes are absolute

UsrGrantPlcMoveX    AT %LsPlc47.21;  * (LsPlc47.21) 
UsrGrantPlcMoveX1   AT %LsPlc47.22;  * (LsPlc47.22) 
UsrGrantPlcMoveX3   AT %LsPlc47.23;  * (LsPlc47.23) 


WaitVRTC_OK         AT %LsPlc47.30;  * (LsPlc47.30) Wait Regulation ok 
VRTCIncSetPReached  AT %LsPlc47.31;  * (LsPlc47.31) Vrtc  has reached the setpoint for the inc movement


*Remote controller*******************
X_AXIS_POSITION     AT %LsPlc77;
Y_AXIS_POSITION     AT %LsPlc76;
Z_AXIS_POSITION     AT %LsPlc75;
*********************************
APA_X_Sample        AT %LsPlc59;     * (LsPlc59) X coordinate sample for APA calculation during apa 
APA_Y_Sample        AT %LsPlc60;     * (LsPlc60) Y coordinate sample for APA calculation during apa


***********************************
* Plc RETAIN (Generali macchina)
*rPlc1
TecUsedGui          AT %rPlc1;    * (rPlc1)Variabile passata a Monitor.ini per lancio Sinottico (0=Pls, 1=Thc, 2=Oxi, 3=Laser, 4=Wtj, 5=pls_am)
TecUsedIso          AT %rPlc2;    * (rPlc2)Variabile a ISO(3333.pgm) per selezione tecnologia(1=Pls, 2=Oxi, 3=Wtj, 4=Laser)

AxisUsaved          AT %rPlc5;    * (rPlc5)Quota asse U per calcolo tubo in macchina


* --------------------------------------------------------------

****************************************************************************************************
*                              (Gui Generali di macchina)
*                  Variabili scritte da GUI per scambio dati con ISO e PLC
* ------------------------------------------------------------------------------------------------ *
****************************************************************************************************
*gGui0
*(VAR_IN_OUT) used   AT %gGui0.0;    *(gGui0.0)Abilita Setup GAntry ed "Escludi allineamento a fine taratura"
AcqusizQtaCorrez    AT %gGui0.1;     *(gGui0.1)Acquisiz. correz. Quota di taratura asse slave Gantry
Preset_Allin        AT %gGui0.2;     *(gGui0.2)Richiesta Preset/Allineamento quota slave a master
PresetErrReq        AT %gGui0.3;     *(gGui0.3)Richiesta Preset/Allineamento quota slave a master per errore allin.
PresetTubo          AT %gGui0.4;     *(gGui0.4)Richiesta Preset asse Tubo
OffRegolPls         AT %gGui0.5;     *(gGui0.5)Richiesta congelamento regolazione da Operatore
LaserEnable         AT %gGui0.11;    *(gGui0.11) Laser point enable by operator
WLensplus           AT %gGui0.12;    *(gGui0.12)Comando W_Lens +
WLensmimus          AT %gGui0.13;    *(gGui0.13)Comando W_Lens -
SelManAxe_Lens      AT %gGui0.14;    *(gGui0.14)Selezione W_Lens -

guiRef_Y            AT %gGui0.20;    *(gGui0.20) GUI button REF Y
guiRef_Z            AT %gGui0.21;    *(gGui0.21) GUI button REF Z
guiJOG_Min          AT %gGui0.22;    *(gGui0.22) GUI button JOG to Minus
guiJOG_Max          AT %gGui0.23;    *(gGui0.23) GUI button JOG to Maximum

*gGui1
OrigMinus           AT %gGui1.0;     *(gGui1.0)Origine -
OrigPlus            AT %gGui1.1;     *(gGui1.1)Origine +
AcquisOrig          AT %gGui1.2;     *(gGui1.2)Acquisisci origine
ImpostaOrig         AT %gGui1.3;     *(gGui1.3)Imposta origine
PosOrigCorr         AT %gGui1.4;     *(gGui1.4)Posiziona X,Y su origine corrente
SelY1Head           AT %gGui1.5;     *(gGui1.5)Selezione Testa Y1 (UI)
EnabJogTuning       AT %gGui1.6;     *(gGui1.6)Abilitazione Jog tuning RTCP (UI)
Repos               AT %gGui1.7;     *(gGui1.7)Attivazione Repos (UI)
JogEnable           AT %gGui1.8;     *(gGui1.8)Attivazione Jog Enable (UI)
SwitchTech          AT %gGui1.9;     *(gGui1.9)Commutazione Tecnologia in uso (UI)
ParkReq             AT %gGui1.10;    *(gGui1.10)Richiesta parcheggio testa
SelXorig            AT %gGui1.11;    *(gGui1.11)Selezione asse X da pagina origini
SelYorig            AT %gGui1.12;    *(gGui1.12)Selezione asse X da pagina origini
SelZorig            AT %gGui1.13;    *(gGui1.13)Selezione asse X da pagina origini
AbilBypass          AT %gGui1.14;    *(gGui1.14)Abilitazione bypass degli inputs
LubrifTest          AT %gGui1.16;    *(gGui1.16)Lubrification test
LubrifReset         AT %gGui1.17;    *(gGui1.17)Lubrification error reset
PuliziaUgello       AT %gGui1.18;    *(gGui1.18)Puizia Ugello Su Spazzola
ResRot              AT %gGui1.19;    *(gGui1.19)Reset rotazione origine
SetAng              AT %gGui1.20;    *(gGui1.20)Set angle
CapacitveTest       AT %gGui1.21;    *(gGui1.21)Capacitive test
ServicePosition     AT %gGui1.25;    *(gGui1.25) GUI Keysoft for Axes to Service Position
ImpostaOrigRot      AT %gGui1.26;    *(gGui1.26) Set only rotate origin

XQtaForceMan        AT %gGui3;       *(gGui3)Quota asse X per "Imposta Origine"(Pag.Origini)
YQtaForceMan        AT %gGui4;       *(gGui4)Quota asse Y per "Imposta Origine"(Pag.Origini)
RotForceMan         AT %gGui5;       *(gGui5)Angolo Rotaz.per "Imposta Origine"(Pag.Origini)

ByPassIn1           AT %gGui6;       *(gGui6)Bypass inputs 1
ByPassIn2           AT %gGui7;       *(gGui7)Bypass inputs 2
ByPassIn3           AT %gGui8;       *(gGui8)Bypass inputs 3
ByEnabIn1           AT %gGui9;       *(gGui9)Enab Bypass inputs 1
ByEnabIn2           AT %gGui10;      *(gGui10)Enab Bypass inputs 2
ByEnabIn3           AT %gGui11;      *(gGui11)Enab Bypass inputs 3

guiTAB_Laser        AT %gGui20.0;   *(gGui20.0)GUI TAB Laser selected
guiTAB_Pallet       AT %gGui20.1;   *(gGui20.1)GUI TAB Pallet selected

guiOnOffGantryPall  AT %gGui21.0;   *(gGui21.0)ON OFF gantry pallet

guiTabDynPiercing   AT %gGui22.0;   *(gGui22.0) Per vis. dati piercing dinamico
guiManualReq        AT %gGui22.1;   *(gGui22.1) Manual cut request
guiAcqXstart        AT %gGui22.2;   *(gGui22.2) Manual cut: acquire X start quota
guiAcqXend          AT %gGui22.3;   *(gGui22.3) Manual cut: acquire X end quota
guiAcqYstart        AT %gGui22.4;   *(gGui22.4) Manual cut: acquire Y start quota
guiAcqYend          AT %gGui22.5;   *(gGui22.5) Manual cut: acquire Y end quota

ButtClearAlarm      AT %gGui23.0;   *(gGui23.0) Clear alarm interface push-button
ButtGuiShootLsr     AT %gGui23.1;
ButtGuiPurgeN2      AT %gGui23.2;
ButtGuiPurgeO2      AT %gGui23.3;
ButtGuiPurgeAIR     AT %gGui23.4;

***********************************
* Gui RETAIN (Generali macchina)
* rGui0
*OrigSel0         AT %rGui0.0;        *(rGui0.0) Origine Selezionata 0
*OrigSel1         AT %rGui0.1;        *(rGui0.1) Origine Selezionata 1
*OrigSel2         AT %rGui0.2;        *(rGui0.2) Origine Selezionata 2
*OrigSel3         AT %rGui0.3;        *(rGui0.3) Origine Selezionata 3
*OrigSel4         AT %rGui0.4;        *(rGui0.4) Origine Selezionata 4
*OrigSel5         AT %rGui0.5;        *(rGui0.5) Origine Selezionata 5
*OrigSel6         AT %rGui0.6;        *(rGui0.6) Origine Selezionata 6
*OrigSel7         AT %rGui0.7;        *(rGui0.7) Origine Selezionata 7
*OrigSel8         AT %rGui0.8;        *(rGui0.8) Origine Selezionata 8
*OrigSel9         AT %rGui0.9;        *(rGui0.9) Origine Selezionata 9
*OrigSel10        AT %rGui0.10;       *(rGui0.10) Origine Selezionata 10
*OrigSel11        AT %rGui0.11;       *(rGui0.11) Origine Selezionata 11
*OrigSel12        AT %rGui0.12;       *(rGui0.12) Origine Selezionata 12
*OrigSel13        AT %rGui0.13;       *(rGui0.13) Origine Selezionata 13
*OrigSel14        AT %rGui0.14;       *(rGui0.14) Origine Selezionata 14
*OrigSel15        AT %rGui0.15;       *(rGui0.15) Origine Selezionata 15
*OrigSel16        AT %rGui0.16;       *(rGui0.16) Origine Selezionata 16
*OrigSel17        AT %rGui0.17;       *(rGui0.17) Origine Selezionata 17
*OrigSel18        AT %rGui0.18;       *(rGui0.18) Origine Selezionata 18
OrigSel19         AT %rGui0.19;       *(rGui0.19) Origine Selezionata 19
GUI_BUTT_NoTube   AT %rGui0.24;       *(rGui0.24) GUI button NO TUBE IN MACHINE

UpArrow          AT %rGui2;          *(gGui2)Up arrow management
LeftArrow        AT %rGui3;          *(gGui3)Left arrow management
DownArrow        AT %rGui4;          *(gGui4)Down arrow management
RightArrow       AT %rGui5;          *(gGui5)Right arrow management

ManSelector      AT %rGui6;          *(rgGui6)Selettore manuale potenza/duty/frequenza  
ManSelectorPOW   AT %rGui7;          *(rgGui7)Selettore manuale POTENZA: ultimo valore impostato
ManSelectorDUTY  AT %rGui8;          *(rgGui8)Selettore manuale DUTY: ultimo valore impostato
ManSelectorFREQ  AT %rGui9;          *(rgGui9)Selettore manuale FREQUENCY: ultimo valore impostato


* %rGui10 declared in Var in Out

* TODO Da inserire queste due variabili nella struttura LSR_GEST (defcn.app)
ParIntSpeedFilt_duty    AT %rGui20;     *(rGui20)Filtro FIR sulla velocit� interpolata calcolata [msec] PER DUTY in funz(vel)
ParIntSpeedFilt_freq    AT %rGui21;     *(rGui21)Filtro FIR sulla velocit� interpolata calcolata [msec] PER FREQ in funz(vel)

LinearFun_tim_Y1        AT %rGui22;     *(rGui22)Funzione lineare: Y1
LinearFun_tim_Y2        AT %rGui23;     *(rGui23)Funzione lineare: Y1
LinearFun_freq_X1       AT %rGui24;     *(rGui24)Funzione lineare: X1
LinearFun_freq_X2       AT %rGui25;     *(rGui25)Funzione lineare: X4
LinearFun_freq_REF      AT %rGui26;     *(rGui26)Funzione lineare: frequenza campione utilizzata per settare tempo FIR sincronismo DUTY



****************************************************************************************************
*                              (Iso Generali di macchina)
*                  Variabili scritte da ISO per scambio dati con ISO e PLC
* ------------------------------------------------------------------------------------------------ *
*
* %gIso[0] = gTechType   ;Sritto da cicli fissi 5 assi per definire la tecnologia in uso.
*


**************************************
* VARIABILI PLC TECNOLOGIA LASER     *
**************************************
* LsPlc;       ** Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA LASER)
* rLsPlc;      ** Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA LASER RETAIN)

UpLoad_DB_Laser     AT %LsPlc0.0;    *(LsPlc0.0 )Comando Caricamento DataBase Laser
RegolLaserActive    AT %LsPlc0.1;    *(LsPlc0.1 )Regolazione Laser Attiva
UseDbUser           AT %LsPlc0.5;    *(LsPlc0.5) User db is used
UseDbUser_Offline   AT %LsPlc0.6;    *(LsPlc0.6) User db is used

LsrVrtcFcSoftware   AT %LsPlc0.31;   *(LsPlc0.31)Intervento Fc Sofware in lavorazione Laser

*DWORD "LsPlc1" riservato per dialogo "LaserPage" / Fonte Laser
D5_ChillWarning     AT %LsPlc1.9;    *(LsPlc1.9)Chiller Warning    [Da Laser a "GUI"]
D6_ChillError       AT %LsPlc1.10;   *(LsPlc1.10)Chiller Error     [Da Laser a "GUI"]
LensType0           AT %LsPlc1.11;   *(LsPlc1.11)Lens Type 0                   [Da Precitec a "GUI"]
LensType1           AT %LsPlc1.12;   *(LsPlc1.12)Lens Type 1                   [Da Precitec a "GUI"]
Error               AT %LsPlc1.13;   *(LsPlc1.13)Error                         [Da Precitec a "GUI"]
PositReached        AT %LsPlc1.14;   *(LsPlc1.14)Position Reached              [Da Precitec a "GUI"]
SW1                 AT %LsPlc1.15;   *(LsPlc1.15)Exch.cartrige avaiable        [Da Precitec a "GUI"]
SW4                 AT %LsPlc1.16;   *(LsPlc1.16)Prot.window cartrige avaiable [Da Precitec a "GUI"]
ReferenceRun        AT %LsPlc1.17;   *(LsPlc1.17)Reference in corso
AutoState           AT %LsPlc1.18;   *(LsPlc1.18)Stato di automatico

LaserOn             AT %LsPlc1.25;    *(LsPlc1.25) Per GUI  LaserOn
PresitecReady       AT %LsPlc1.26;    *(LsPlc1.26) For GUI  Presitec Ready
PresitecFAR         AT %LsPlc1.27;    *(LsPlc1.27) For GUI  Precitec FAR
OutpSelect          AT %LsPlc1.28;    *(LsPlc1.28) For GUI  Precitec OUTP.SELECT
YLRpowersupplyON    AT %LsPlc1.29;    *(LsPlc1.29) YLR power supply active
YLRremotestartPB    AT %LsPlc1.30;    *(LsPlc1.39) YLR remote start PB

Err1Precitec        AT %LsPlc3.0;       *(LsPlc3.0) Protective window cartridge not present 19.375
Err2Precitec        AT %LsPlc3.1;       *(LsPlc3.1) Protective window not present 18.75
Err3Precitec        AT %LsPlc3.2;       *(LsPlc3.2) Protective window: max. temperature exceeded 18.125
Err4Precitec        AT %LsPlc3.3;       *(LsPlc3.3) Drive malfunction: overload 17.5
Err5Precitec        AT %LsPlc3.4;       *(LsPlc3.4) Drive malfunction: no power supply 16.875
Err6Precitec        AT %LsPlc3.5;       *(LsPlc3.5) Drive malfunction: supply voltage too low 16.25
Err7Precitec        AT %LsPlc3.6;       *(LsPlc3.6) Drive: no reference travel/reference travel aborted 15.625
Err8Precitec        AT %LsPlc3.7;       *(LsPlc3.7) Collimating lens: max. temperature exceeded 15.0
Err9Precitec        AT %LsPlc3.8;       *(LsPlc3.8) Focusing lens: max. temperature exceeded 14.375
Err10Precitec       AT %LsPlc3.9;       *(LsPlc3.9) Laser head: max. temperature exceeded 13.75
Err11Precitec       AT %LsPlc3.10;      *(LsPlc3.10) Diffused laser light: max. level exceeded 13.125
Err12Precitec       AT %LsPlc3.11;      *(LsPlc3.11) Pressure - Cutting gas: max. pressure exceeded 12.5
Err13Precitec       AT %LsPlc3.12;      *(LsPlc3.12) Pressure inside the laser head: max. pressure exceeded 11.875

Tp_ValDerProbe      AT %LsPlc4;      * Test Point Derivata lettura alalogica per "APA"

PipeInsideLenght    AT %LsPlc6;      *Lunghezza tubo in macchina

VelInterpCN0        AT %LsPlc10;     *(LsPlc10) A_GUI per visualizzare velicit� interpolatore CN0

ExeCalibraz         AT %LsPlc11.0;   *(LsPlc11.0) Richiesta esecuzione Calibrazione
IPG_YLR_SPARO_Laser AT %LsPlc11.1;   *(LsPlc11.1) IPG_YLR_Sparo laser in corso
GasForLaserShot     AT %LsPlc11.2;   *(LsPlc11.2) Gas enable for laser shot
CondHoldPlcExt      AT %LsPlc11.3;   *(LsPlc11.3) Hold condition from external plc(pallet change)
LastGasAir          AT %LsPlc11.4;   *(LsPlc11.4) Keep last gas open valve air
LastGasOxy          AT %LsPlc11.5;   *(LsPlc11.5) Keep last gas open valve oxygen
LastGasNit          AT %LsPlc11.6;   *(LsPlc11.6) Keep last gas open valve nitrogen
Exe_SELF_Calibraz   AT %LsPlc11.7;   *(LsPlc11.7) Richiesta esecuzione Calibrazione
TipTouchNotFiltered AT %LsPlc11.8;   *(LsPlc11.8) TipTouch signal not filtered
GasTestLED          AT %LsPlc11.10;  *(LsPlc11.10) LED Comando TEST Valvola prop.
GasYV22LED          AT %LsPlc11.11;  *(LsPlc11.11) LED aux valve
GasYV23LED          AT %LsPlc11.12;  *(LsPlc11.12) LED nozzle air
GasYV24LED          AT %LsPlc11.13;  *(LsPlc11.13) LED cut air assistance
GasYV0LED           AT %LsPlc11.14;  *(LsPlc11.14) LED Comando Valvola Aria Taglio
GasYV2LED           AT %LsPlc11.15;  *(LsPlc11.15) LED Comando Valvola Ossigeno
GasYV3LED           AT %LsPlc11.16;  *(LsPlc11.16) LED Comando Valvola Azoto
AspOnOffLED         AT %LsPlc11.17;  *(LsPlc11.17) LED Comando ON\OFF Aspiratore
AspAutoLED          AT %LsPlc11.18;  *(LsPlc11.18) LED Abilitazione Auto Aspiratore
AspTestEvLED        AT %LsPlc11.19;  *(LsPlc11.19) LED Abilitazione Test EV settori Aspirazione
AspTypeLED          AT %LsPlc11.20;  *(LsPlc11.20) LED Tipo comando ON\OFF Aspiratore


Up_PalletIn         AT %LsPlc26.0;   *(LsPlc26.0) Up pallet in machine 
Dwn_PalletIn        AT %LsPlc26.1;   *(LsPlc26.1) Dwn pallet in machine
HoldSafePipeX       AT %LsPlc26.2;   *(LsPlc26.2) Hold moving for X axis, not allow manual movement in pipe area
OrderMoveXPosit     AT %LsPlc26.3;   *(LsPlc26.3) Order of movement positive dir for X
OrderMoveXNegat     AT %LsPlc26.4;   *(LsPlc26.4) Order of movement negative dir for X
OrderMoveYPosit     AT %LsPlc26.5;   *(LsPlc26.5) Order of movement positive dir for Y
OrderMoveYNegat     AT %LsPlc26.6;   *(LsPlc26.6) Order of movement negative dir for Y
HoldSafePipeY       AT %LsPlc26.7;   *(LsPlc26.7) Hold moving for Y axis, not allow manual movement in pipe area
UnlockLaserComm     AT %LsPlc26.8;   *(LsPlc26.8) Unlock iso execution for Tstp15 command

ECAT_OK             AT %LsPlc26.9;

num_pallet_machine  AT %LsPlc27;     *(LsPlc27) Number of pallet in machine (1=up,2=down) gui
PowerLaserOut       AT %LsPlc30;     *(LsPlc30) Valore potenza laser in Uscita
GasPressPgmSet      AT %LsPlc31;     *(LsPlc31) Gas pressure setpoint by program (mbar)
DutyLaserOut        AT %LsPlc32;     *(LsPlc32) Valore Duty Cycle laser in Uscita
FreqLaserOut        AT %LsPlc33;     *(LsPlc33) Valore Frequenza laser in Uscita
CurPressPSI         AT %LsPlc34;     *(LsPlc34) Current gas pressure (PSI*10)
PressCorrente       AT %LsPlc35;     *(LsPlc35) Pressione Corrente (mbar)
SetOutPress         AT %LsPlc38;     *(LsPlc38) Setpoint output pressure (mbar)
SetPressPSI         AT %LsPlc39;     *(LsPlc39) Setpoint output pressure (PSI*10)
BitInValvProp       AT %rLsPlc39;    *(rLsPlc39) Numero bit x feedback valvola prop.
BitOutValvProp      AT %rLsPlc42;    *(rLsPlc42) Numero bit x uscita valvola prop.

Count_sec_day       AT %rLsPlc40;    *(rLsPlc40) Counter for second, last switch on
Count_day_day       AT %rLsPlc41;    *(rLsPlc41) Counter for day, last switch on 
Count_day_tot       AT %rLsPlc44;    *(rLsPlc44) Counter for day, total time 
Count_year_tot      AT %rLsPlc45;    *(rLsPlc45) Counter for years, total time
Count_sec_tot       AT %rLsPlc46;    *(rLsPlc46) Counter for second, total time

Count_sec_lson      AT %rLsPlc47;    *(rLsPlc47) Counter for second, Laser on
Count_day_lason     AT %rLsPlc48;    *(rLsPlc48) Counter for day, laser on

LubricType          AT %rLsPlc53;     *(LsPlc53) Tipology of lubriation
DoorSafety          AT %rLsPlc55;     *(LsPlc55) Tipology of safety door

CurrentWPage        AT %rLsPlc56;
CypToWPage          AT %rLsPlc57;

**************************************


plc_Lubrication_Clk     AT %LsPlc45.0;      *(LsPlc45.0) PLC Enable Lubrication cicle from GUI
plc_Pallet_AUT          AT %LsPlc45.1;      *(LsPlc45.1) PLC Enable for Pallet moving in AUTO
plc_Enable_Menu         AT %LsPlc45.2;      *(LsPlc45.2) PLC Enable for main ESA MENU
plc_Enable_ClosePage    AT %LsPlc45.3;      *(LsPlc45.3) PLC Enable for keysoft Close Page UIPL
plc_Enable_LubClk       AT %LsPlc45.4;      *(LsPlc45.4) PLC Enable for Lubrication cycle


QtaPosizIncr            AT %LsPlc46;        *(LsPlc46) Quota Posizionam. Z incr.

JogCommandTipTouch      AT %LsPlc47.0;      *(LsPlc47.0) Automatic jog command for tip touch
Up_Z_RetraceBack        AT %LsPlc47.1;      *(LsPlc47.1) Comando Salita Z e Retrace indietro (Dopo TipTouch)
StartAftTipTouch        AT %LsPlc47.2;      *(LsPlc47.2) Start after tip touch
EnPosiz                 AT %LsPlc47.3;      *(LsPlc47.3) Abilitaz. posiz. Asse Z
Posiz_Z_OK              AT %LsPlc47.4;      *(LsPlc47.4) Posizionam. Z Eseguito
StartSeqHold            AT %LsPlc47.5;      *(LsPlc47.5) Avvio seq.Salita Z dopo hold (Senza Autorestart)
StartAftTipTouch_2      AT %LsPlc47.6;      *(LsPlc47.6) Start after tip touch (Enable error amplifier) 

***** %LsPlc48 declared VAR_IN_OUT
***** %LsPlc50....69 Reserved for IO simulations

*  LsPlc78 to LsPlc99 used in capacit.plc

Perc_erro10             AT %LsPlc90;         * (LsPlc90) Percentual Value error 10 mm
Perc_erro9              AT %LsPlc91;         * (LsPlc91) Percentual Value error 9 mm
Perc_erro8              AT %LsPlc92;         * (LsPlc92) Percentual Value error 8 mm
Perc_erro7              AT %LsPlc93;         * (LsPlc93) Percentual Value error 7 mm
Perc_erro6              AT %LsPlc94;         * (LsPlc94) Percentual Value error 6 mm
Perc_erro5              AT %LsPlc95;         * (LsPlc95) Percentual Value error 5 mm
Perc_erro4              AT %LsPlc96;         * (LsPlc96) Percentual Value error 4 mm
Perc_erro3              AT %LsPlc97;         * (LsPlc97) Percentual Value error 3 mm
Perc_erro2              AT %LsPlc98;         * (LsPlc98) Percentual Value error 2 mm
Perc_erro1              AT %LsPlc99;         * (LsPlc99) Percentual Value error 1 mm


Memo_C1_LASER_SWITCH		AT  %rLsPlc0.0; *(rLsPlc0.0) Memoria stato comando C1_LASER_SWITCH
*                           AT  %rLsPlc0.1; *(rLsPlc0.1) N.U.
LastClampState              AT  %rLsPlc0.2;    *(rLsPlc0.2) Memory pipe clamp state
LastClampStateF             AT  %rLsPlc0.3;    *(rLsPlc0.3) Memory pipe clamp state
LastUnLoadSupState1         AT  %rLsPlc0.4;    *(rLsPlc0.4) Memory pipe clamp state
LastUnLoadSupState2         AT  %rLsPlc0.5;    *(rLsPlc0.5) Memory pipe clamp state
LastUnLoadSupState3         AT  %rLsPlc0.6;    *(rLsPlc0.6) Memory pipe clamp state
LastUnLoadSupState4         AT  %rLsPlc0.7;    *(rLsPlc0.7) Memory pipe clamp state
LastUnLoadSupState5         AT  %rLsPlc0.8;    *(rLsPlc0.8) Memory pipe clamp state
LastUnLoadSupState6         AT  %rLsPlc0.9;    *(rLsPlc0.9) Memory pipe clamp state
LastUnLoadSupState7         AT  %rLsPlc0.10;   *(rLsPlc0.10) Memory pipe clamp state
* .............................
Memo_CMD_SBL_LAM_ASSE_W	AT  %rLsPlc0.24; *(rLsPlc0.24) Memoria stato Comando Sbloccaggio lamiera asse W (Pistone Alto)
Memo_CMD_BL_LAM_ASSE_W	AT  %rLsPlc0.25; *(rLsPlc0.25) Memoria stato Comando Bloccaggio lamiera asse W (Pistone Basso)
Memo_CMD_SBL_LAM_TENDITORE	AT  %rLsPlc0.26; *(rLsPlc0.26) Memoria stato Comando Sbloccaggio lamiera Tensionatore (Pistone Alto)
Memo_CMD_BL_LAM_TENDITORE	AT  %rLsPlc0.27; *(rLsPlc0.27) Memoria stato Comando Bloccaggio lamiera Tensionatore (Pistone Basso)

retainFeedOvr               AT  %rLsPlc12;      * (rLsPlc12) Retain value for Feed Override
Time_pressure_check         AT  %rLsPlc13;      * (rLsPlc13) Attesa controllo pressione pompa
Focal_Type                  AT  %rLsPlc14;      * (rLsPlc14) Tipologia focale

second                     AT %rLsPlc21;     * (rLsPlc21) timer for lubrication cycle second
minute                     AT %rLsPlc22;     * (rLsPlc22) timer for lubrication cycle minute
EnabAlgori                 AT %rLsPlc78.0;
TimeToStart                AT %rLsPlc79;
PeriodTotal                AT %rLsPlc80;
PercRed                    AT %rLsPlc81;


**************************************
* VARIABILI GUI TECNOLOGIA LASER     *
**************************************
***********************************************************************************************
* (LsGui) ===>>> ---------------------- Riservati in scrittura a GUI (GLOBALI TECNOLOGIA LASER)
***********************************************************************************************

*** VAR_IN_OUT in FUNCTION "Coil.plc"  ===>>>    AT %LsGui0.0;    *(LsGui0.0)
ON_OFF_CABINA               AT %LsGui0.1;    *(LsGui0.1)
ON_OFF_TESTA_LASER          AT %LsGui0.2;    *(LsGui0.2)
PLS_HMI_SCARICO             AT %LsGui0.3;    *(LsGui0.3)  Pulsante scarico lamiera da macchina su nastro
PLS_HMI_CARICO              AT %LsGui0.4;    *(LsGui0.4)  Pulsante carico lamiera in macchina da nastro
Refer_Travel                AT %LsGui0.7;    *(LsGui0.7)  Precitec: Reference Travel da pagina video [LaserPage]
Sblocco_Porte			          AT %LsGui0.8;    *(LsGui0.8)  Pulsante ON\OFF blocco porte
Calibrazione                AT %LsGui0.9;    *(LsGui0.9)  Richiesta Calibrazione [LaserPage]
SparoMan_On                 AT %LsGui0.10;   *(LsGui0.10) Sparo Manuale Start
SparoMan_Stop               AT %LsGui0.11;   *(LsGui0.11) Sparo Manuale Stop
ZupAfterHold                AT %LsGui0.14;   *(LsGui0.14) Salita asse Z dopo Hold
Pls_AspTestEv               AT %LsGui0.15;    *(LsGui0.15) Pulsante Abilitazione Test EV settori Aspirazione
Self_Calibrazione           AT %LsGui0.16;    *(LsGui0.16)  self-capacitive calibration
Door1_Up_Gui                 AT %LsGui0.17;    *(LsGui0.17)  self-capacitive calibration
Door1_Down_Gui               AT %LsGui0.18;    *(LsGui0.18)  self-capacitive calibration
Door2_Up_Gui                 AT %LsGui0.19;    *(LsGui0.19)  self-capacitive calibration
Door2_Down_Gui               AT %LsGui0.20;    *(LsGui0.20)  self-capacitive calibration

Manual_alignment            AT %LsGui0.30;   *(LsGui0.30)Manual alignment for C C1 axis
Pls_ON_OFF_Aspiratore       AT %LsGui0.31;   *(LsGui0.31)Pulsante ON\OFF accensione Aspiratore

FeedbackDBloaded            AT %LsGui1.0;           * (LsGui1.0) upload DB request
PlsOnOffVerify              AT %LsGui1.4;           * (LsGui1.4)Pulsante Verify
PulsMoveFocal               AT %LsGui1.5;           * (LsGui1.5) Focal command to move to set point
PalletInCharge              AT %LsGui2.0;           * (LsGui2.0)Pallet in charging

CopyParamComm               AT %LsGui3.0;           * (LsGui3.0) Copy common parameter X-Y 
EnabManCutHole              AT %LsGui3.1;           * (LsGui3.1) Enable manual cut on material empty zones 
ManualAPAReq                AT %LsGui3.2;           * (LsGui3.2) manual APA request          
SwitchLaserGui              AT %LsGui3.3;           * (LsGui3.3) Switch on laser resonator
BtnJogEnable                AT %LsGui3.4;           * (LsGui3.4) Gui jog enable button

CloseClampC                 AT %LsGui3.4;           * (LsGui3.4) Close clamp C
OpenClumpC                  AT %LsGui3.5;           * (LsGui3.5) Open clamp C 
CloseClampC1                AT %LsGui3.6;           * (LsGui3.6) Close clamp C1
OpenClampC1                 AT %LsGui3.7;           * (LsGui3.7) Open clamp C1
Open_C_C1                   AT %LsGui3.8;           * (LsGui3.8) Open C C1 
Close_C_C1                  AT %LsGui3.9;           * (LsGui3.9) Close C C1
ForceFeedSlow               AT %LsGui3.10;          * (LsGui3.10)  force feed slow
                         
RefRaytFocal                AT %LsGui3.20;           * (LsGui3.20) Reference comand from gui
                         
NumRecordSave               AT %LsGui10;     *(LsGui10)Numero Record su cui salvare pagina Ricetta Tabella

TabMachineSettings          AT %LsGui11;            * (LsGui11) Tab machine settings

* Gas managment pages
P_TestAir                 AT %LsGui18.3;   *(LsGui18.3) Gas air test from valve page
P_TestOxi                 AT %LsGui18.4;   *(LsGui18.4) Gas oxigen test from valve page
P_TestNit                 AT %LsGui18.5;   *(LsGui18.5) Gas nitrogen test from valve page
P_ValTestAir              AT %LsGui18.6;   *(LsGui18.6) Valve air test from setting page
P_ValTestOxy              AT %LsGui18.7;   *(LsGui18.7) Valve oxygen test from setting page
P_ValTestNit              AT %LsGui18.8;   *(LsGui18.8) Valve nitrogen test from setting page
P_ValTestProp             AT %LsGui18.9;   *(LsGui18.9) Proportional valve test from setting page

FocByUser                   AT %LsGui74;       *(LsGui74) Set-point for focal movement by user

* rLsGui;      ** Riservati in scrittura a GUI (GLOBALI TECNOLOGIA LASER RETAIN)

*[LaserPage]
PotSparoMan         AT %rLsGui1;      *(rLsGui1) A_GUI Potenza Sparo In Manuale
DutySparoMan        AT %rLsGui2;      *(rLsGui2) A_GUI Duty Sparo In Manuale
FreqSparoMan        AT %rLsGui3;      *(rLsGui3) A_GUI Frequenza Sparo In Manuale
TimeSparoMan        AT %rLsGui4;      *(rLsGui4) A_GUI Tempo Sparo In Manuale

*[SetMachine]
TimeRetraceInd      AT %rLsGui5;      *(rLsGui5) Tempo Retrace Indietro per Autorestart dopo TipTouch

*[GasPage]
PressMaxEvProp      AT %rLsGui11;     *(rLsGui11)Pressione massima valvola proporzionale
PressMaxOssigeno    AT %rLsGui12;     *(rLsGui12)Pressione massima Ossigeno
PressMaxAzoto       AT %rLsGui13;     *(rLsGui13)Pressione massima Azoto
PressMaxAria        AT %rLsGui14;     *(rLsGui14)Pressione massima Aria
PressTestGas        AT %rLsGui15;     *(rLsGui15)Pressione test Gas
PressSpurgoGas      AT %rLsGui16;     *(rLsGui16)Pressione spurgo Gas
PotenzaMaxLaser     AT %rLsGui17;     *(rLsGui17)Potenza massima Laser
FinestraAllPress    AT %rLsGui18;     *(rLsGui18)Finestra allarme pressione da pagina "Gas" in %
TimeMaxAllPress     AT %rLsGui19;     *(rLsGui19)Tempo Ritardo allarme pressione da pagina "Gas"
Delay_Press_Check   AT %rLsGui21;     *(rLsGui21) Delay_Press_Check
Diff_Limit_Pres     AT %rLsGui22;     *(rLsGui22) Input limit difference pressure
ManualAPAX          AT %rLsGui23;     *(rLsGui23) Lenght X for manual cut
ManualAPAY          AT %rLsGui24;     *(rLsGui24) Lenght Y for manulal cut

MicroJointDist      AT %rLsGui25;     *(rLsGui25) Distance for microjoint
FilterForMicroJ     AT %rLsGui26;     *(rLsGui26) Microjoint FIR time (NOT USED)

EnabGasOutByGraph   AT %rLsGui27.0;   **(rLsGui27.0) PLC option for enabling the graph (Pressure-voltage)
EnabPowerOutByGraph AT %rLsGui27.1;   **(rLsGui27.1) PLC option for enabling the graph (Power-voltage)
EnabExtraValve      AT %rLsGui27.2;   **(rLsGui27.2) Enable extra valve after proporzional valve
ParClosePropV       AT %rLsGui27.3;   **(rLsGui27.3) Close only proportional valve when close gases in program running
GasSimulated        AT %rLsGui27.4;
OptLsrCutAirPurge   AT %rLsGui27.5;

manualcutreqX       AT %rLsGui29.0;    *(LsGui29.0) Manual cut request along X
manualcutreqY       AT %rLsGui29.1;    *(LsGui29.1) Manual cut request along Y

EnBlowDryRun        AT %rLsGui29.2;    *(LsGui29.2) Enable gas blowing in dry run
EnabCutEdge         AT %rLsGui29.3;    *(LsGui29.3) Enable start to cut from edge
ForceFeedSlow1      AT %rLsGui29.4;    *(rLsGui29.4)  force feed slow 1

SelGasPagLaser      AT %rLsGui40;     *(rLsGui40) Gas selection on laser page (for shoot test)
TipTouchFilter      AT %rLsGui41;     *(rLsGui41) tip touch filter (msec)
Cutting_Line        AT %rLsGui42;     *(rLsGui42) Cutting line for manual cut

ApaXDistOffset      AT %rLsGui43;     *(rLsGui43) Apa X safe offset after origin acquire
ApaYDistOffset      AT %rLsGui44;     *(rLsGui44) Apa Y safe offset after origin acquire

EnabApaTransl       AT %rLsGui45.0;   * (rLsGui45.0) Enable Apa origin translation   
EnabResAngle        AT %rLsGui45.1;   * (rLsGui45.1) Enable reset when program end whith M30           
WaitAfterApa        AT %rLsGui45.2;   * (rLsGui45.2) Wait after apa execution

GuideLaserRecord    AT %rLsGui46;     * (rLsGui46) Trumpf guide laser record

ZoomValGui          AT %rLsGui47;     * (rLsGui47) Zoom value from HMI

VrtcUpLimitNear     AT %rLsGui60;     * (rLsGui60) Up velocity limit for Near phase
VrtcDownLimitNear   AT %rLsGui61;     * (rLsGui61) Down velocity limit for Near phase

MaxErrCalibControl  AT %rLsGui62;     * (rLsGui62) Maximum error allowed during test calibration [%]

WeldingCfound       AT %rLsGui63;     * (rLsGui63) Welding: C position found
WeldingCoffset      AT %rLsGui64;     * (rLsGui64) Welding: C offset 
HeadingOffset       AT %rLsGui65;     * (rLsGui65) Heading: origin offset
WeldingTubeInc      AT %rLsGui66;     * (rLsGui66) Welding: Incremental C for round tube

ErrFoundRtcpTest    AT %rLsGui70;     * (rLsGui70) Error fouded in the RTCP test [0.001mm]
CaxisTestRTCP_Cap   AT %rLsGui71;     * (rLsGui71) C axis position in RTCP test and Capacitive test [0.001mm]


**************************************
* VARIABILI ISO TECNOLOGIA LASER     *
**************************************
* LsIso;       ** Riservati in scrittura per ISO (GLOBALI TECNOLOGIA LASER)
* rLsIso;      ** Riservati in scrittura per ISO (GLOBALI TECNOLOGIA LASER RETAIN)

CodeDB              AT %LsIso0;      *(LsIso0)Numero Codice DataBase da caricare

**** LsIso10 declared VAR_IN_OUT

iso_Debug_Code      AT %LsIso20;     *(LsIso20)Debug ISO code for user cycles
PiercOutRange       AT %LsIso40;     *(LsIso40) execute piercing out of range 

ActualPosX          AT %LsIso41;     * Initial position for jump X
ActulPosY           AT %LsIso42;     * Initial position for jump Y
NextPosX            AT %LsIso43;     * Next position for jump X
NextPosY            AT %LsIso44;     * Next position for jump Y
Dist                AT %LsIso45;
ActualRegSet        AT %LsIso46;     * Actual setpoint before jump     
ActualPosC          AT %LsIso48;     * Initial position for jump X
ActualPosZ          AT %LsIso49;     * Initial position for jump Z

NextPosC            AT %LsIso51;     * Next position for jump C   
NextSetpoint        AT %LsIso52;     * Next setpoint in no piercing mode (cutting distance)

NextPosZ            AT %LsIso56;     * Next position for jump Z  
**************************************

Logger1             AT %LsIso81;     * Logger variable
**************************************

EnabDutyMan         AT %LsIso83;     *(LsIso83) Enable Duty management by speed
EnabPowerMan        AT %LsIso84;     *(LsIso84) Enable Power management by speed
EnabFreqMan         AT %LsIso85;     *(LsIso85) Enable Frequency management by speed


***********************************************************************************
* ASSOCIAZIONE AI REGISTRI DI PERSONALIZZAZIONE CICLICHE UTENTE    (C50 - C126)   *
***********************************************************************************
TimeDelayBl         AT %C55;        *(C55) Delay during piercing to allow gas blowing

WaitGasFlow         AT %C56.0;      *(C56.0) M function iso wait

Capacit_Anal        AT %C102;       * (C102)Analogica Capacitivo (punti ADc)
Capacit_Anal_Mm     AT %C103;       * (C103)Analogica Capacitivo (mm)

LogResourceVRTC1    AT %C121;       * (C121) Logical resource VRTC1 (surface)
LogResourceVRTC2    AT %C131;       * (C131) Logical resource VRTC0 (tool)

OUT_isFlyCutModeON  AT %C126.0;     ** application flyCut mode is ON
INP_laserON         AT %C126.1;     ** application turn laser ON in non-flyCut mode (in C126 xche Scritto da PLC)

CapRange            AT %C151;       * (C151) Capacitive range
tcp_Z_head1_fast    AT %C152;        * (C152) Z tcp of head 1
tcp_X_head1_fast    AT %C157;        * (C157) X tcp of head 1
tcp_Y_head1_fast    AT %C158;        * (C158) Y tcp of head 1

bevAngleHead1_calc1 AT %C153;        * (C153) head 1 bevel angle 1 (feedback VRTC tools)
bevAngleHead1_calc2 AT %C154;        * (C154) head 1 bevel angle 2 (setpoint VRTC surface)
bevAngleHead1_calc3 AT %C164;        * (C164) head 1 bevel angle 3 (feedback VRTC surface)

InterpSpeedFiltered AT %C155;        * (C155) 1 velocit� interpolata filtrata (PER POTENZA in funz. della velocita')
PercInterpSpeed     AT %C156;        * (C156) 1 percentuale attuale velocit� interpolata filtrata rispetto alla programmata

** in VAR_IN_OUT BevelHeadSyncG800   AT %C165.0;      * (C165.0) finestra di controllo angolo di bevel (=0, considera angolo=0)

InterpSpeedFiltered2    AT %C170;       * (C170) 2 velocit� interpolata filtrata (PER DUTY in funz. della velocita')
PercInterpSpeed2        AT %C171;       * (C171) 2 percentuale attuale velocit� interpolata filtrata rispetto alla programmata

InterpSpeedFiltered3    AT %C172;       * (C172) 2 velocit� interpolata filtrata (PER FREQUENZA in funz. della velocita')
PercInterpSpeed3        AT %C173;       * (C173) 2 percentuale attuale velocit� interpolata filtrata rispetto alla programmata


****************************************************************************************************
***
False               AT %PLCFLAGS.0;
True                AT %PLCFLAGS.1;

PAR_TaioEn          AT %cg3.23;     ** TAIO flyCut enabled
PAR_TaioAssistEn    AT %cg3.24;     ** TAIO Hardware assisted flyCut enabled
PAR_combinedMode    AT %cg3.25;     ** use combined PWM and emission


VerticalScan        AT  %RasterParam0;  
Rotated             AT  %RasterParam1;
Lead                AT  %RasterParam2;
Pitch               AT  %RasterParam3;
Gamma               AT  %RasterParam4;
Black               AT  %RasterParam5;
White               AT  %RasterParam6;
Edge                AT  %RasterParam7;
Height              AT  %RasterParam8;
Demo                AT  %RasterParam9;
Aspect              AT  %RasterParam10;
RasterLatency       AT  %RasterParam11;
***

ch0_InReset              AT %cn0.rc0.0;   * %cn0.rc0.0 CH0 en Reset
ch0_InEmerg              AT %cn0.rc0.1;   * %cn0.rc0.1 CH0 en Emergencia
ch0_InREF                AT %cn0.rc0.8;   * %cn0.rc0.8 CH0 en Modo REF
ch0_InJOG                AT %cn0.rc0.9;   * %cn0.rc0.9 CH0 en Modo JOG
ch0_InINC                AT %cn0.rc0.10;  * %cn0.rc0.10 CH0 en Modo INC
ch0_InMDI                AT %cn0.rc0.11;  * %cn0.rc0.11 CH0 en Modo MDI
ch0_InAUT                AT %cn0.rc0.12;  * %cn0.rc0.12 CH0 en Modo AUT
ch0_InSTEP               AT %cn0.rc0.13;  * %cn0.rc0.13 CH0 Debbug:Single Block execution
ch0_InSTEPIN             AT %cn0.rc0.14;  * %cn0.rc0.14 CH0 Debbug:Single Block execution and subroutines
ch0_InREPOS              AT %cn0.rc0.15;  * %cn0.rc0.15 CH0 REPOS:Recover positions after a HOLD
ch0_InBLKSING            AT %cn0.rc0.17;  * %cn0.rc0.17 CH0 Debbug:STEP mode activated
ch0_InBLKSWTCH           AT %cn0.rc0.18;  * %cn0.rc0.18 CH0 Activated execution switched block
ch0_InDRYRUN             AT %cn0.rc0.19;  * %cn0.rc0.19 CH0 DryRun activated
ch0_InTEST               AT %cn0.rc0.21;  * %cn0.rc0.21 CH0 TEST:Simulated execution
ch0_InJOGOPEN            AT %cn0.rc0.22;  * %cn0.rc0.22 CH0 JOGOPEN activated
ch0_InSTEPOUT            AT %cn0.rc0.23;  * %cn0.rc0.23 CH0 Debbug:Function excuted
ch0_InRICBLOCCO          AT %cn0.rc0.24;  * %cn0.rc0.24 CH0 Ricerca Blocco activated


ch0_InRun                AT %cn0.rc8.0;   * %cn0.rc8.0 CH0 Programa RUN activo
ch0_InHold               AT %cn0.rc8.1;   * %cn0.rc8.1 CH0 Programa RUN con HOLD activo
ch0_InReady              AT %cn0.rc8.2;   * %cn0.rc8.2 CH0 Ready
ch0_InAlarm              AT %cn0.rc8.3;   * %cn0.rc8.3 CH0 en Alarma
ch0_StrobeM              AT %cn0.rc8.4;   * %cn0.rc8.4 CH0 Strobe Funcion M desde ISO
ch0_ValFunM              AT %cn0.rc9;     * %cn0.rc9 CH0 Registro Funcion M desde ISO
ch8_ValFunM              AT %cn8.rc9;     

ch0_PrgSPEED             AT %cn0.rc25;    * %cn0.rc25 CH0 Programmed SPEED from ISO

ch2_StrobeM              AT %cn2.rc8.4;   * %cn2.rc8.4 CH2 Strobe Funcion M desde ISO
ch2_ValFunM              AT %cn2.rc9;     * %cn2.rc9 CH2 Registro Funcion M desde ISO

ax0_CancelMov           AT %ax0.ra0.16;   * %ax0.ra0.16 Cancel movement for Axis 0-Xv
ax1_CancelMov           AT %ax1.ra0.16;   * %ax1.ra0.16 Cancel movement for Axis 1-Yv
ax2_CancelMov           AT %ax2.ra0.16;   * %ax2.ra0.16 Cancel movement for Axis 2-Zv
ax27_CancelMov          AT %ax27.ra0.16;  * %ax27.ra0.16 Cancel movement for Axis 27-Xr
ax28_CancelMov          AT %ax28.ra0.16;  * %ax28.ra0.16 Cancel movement for Axis 28-Yr
ax29_CancelMov          AT %ax29.ra0.16;  * %ax29.ra0.16 Cancel movement for Axis 29-Zr

ax0_GoFordward          AT %ax0.ra3.2;    * %ax0.ra3.2 Axis 0 moving to positive
ax1_GoFordward          AT %ax1.ra3.2;    * %ax1.ra3.2 Axis 1 moving to positive
ax2_GoFordward          AT %ax2.ra3.2;    * %ax2.ra3.2 Axis 2 moving to positive
ax27_GoFordward         AT %ax27.ra3.2;   * %ax27.ra3.2 Axis 27 moving to positive
ax28_GoFordward         AT %ax28.ra3.2;   * %ax28.ra3.2 Axis 28 moving to positive
ax29_GoFordward         AT %ax29.ra3.2;   * %ax29.ra3.2 Axis 29 moving to positive

ax0_Commanded           AT %ax0.ra3.13;   * %ax0.ra3.13 Axis 0 commanded to moving
ax1_Commanded           AT %ax1.ra3.13;   * %ax1.ra3.13 Axis 1 commanded to moving
ax2_Commanded           AT %ax2.ra3.13;   * %ax2.ra3.13 Axis 2 commanded to moving
ax27_Commanded          AT %ax27.ra3.13;  * %ax27.ra3.13 Axis 27 commanded to moving
ax28_Commanded          AT %ax28.ra3.13;  * %ax28.ra3.13 Axis 28 commanded to moving
ax29_Commanded          AT %ax29.ra3.13;  * %ax29.ra3.13 Axis 29 commanded to moving

IntCopyLogg             AT %ui39.0;       * ui39.0 interface signal for log transfer from cnc to pc folder 
ResetCncInProgress      AT %ui39.1;       * ui39.1 Reset CNC in progress (setted for 2 sec)

AnalCapFeedback         AT %config_machine33;    * resolution of analog feedback for capacitive sensor

END_VAR





*******************************************************************************
*   ###VAR_IN_OUT
*******************************************************************************
VAR_IN_OUT

BevelHeadSyncG800       AT %C165.0;     * (C165.0) finestra di controllo angolo di bevel (=0, considera angolo=0)
NormalHeadingG800       AT %C165.1;     * (C165.1)
NormalCuttingG800       AT %C165.2;     * (C165.2)
BevelHeadingG800        AT %C165.3;     * (C165.3)
BevelCuttingG800        AT %C165.4;     * (C165.4)

WeldingSolderJointCoun  AT %LsPlc5;      * Solder joint counted during pipe welding

WeldingFuncOn           AT %LsIso2.0;    *(LsIso2.0)Welding function in progress
GearingC_C1Off          AT %LsIso2.1;    *(LsIso2.1)Gearing C-C1 OFF from ISO 
WeldingNoRead           AT %LsIso2.2;    *(LsIso2.2)Welding function: no read sensor
WeldingSensorPiston     AT %LsIso2.3;    *(LsIso2.3)Welding function: piston management
visCameraPiston         AT %LsIso2.4;    *(LsIso2.4)Welding seam detection: piston management
IsoCameraToDetec        AT %LsIso2.5;    *(LsIso2.5)Welding seam detection: Camera taking picture

UI_CH6_RUN_STP      AT %M1.0;       * (M1.0) START/STOP channel 6
CH6_RUN_STP         AT %M1.1;       * (M1.1) START/STOP channel 6 flag
TipFromPLCfast      AT %M1.2;       * (M1.2) used for ESA internal use
enabcopy            AT %M124.1;        * test copia

CUT_DISTANCE        AT %TabLsr0.L_CutDistance;      * Cut distance
SensorGain          AT %vrtc0.VrSensorGain;        * Capacitive sensor resolution 1mm
SensorGain_1        AT %vrtc1.VrSensorGain;        * Capacitive sensor resolution 1mm

CoefInterfCalc      AT %ui0.10;        * (ui0.10) calculate the coefficients

guiShowISO          AT %rGui1.20;       *(rGui1.20)Enable to see ISO/Graph window in Main page

EnMicroJoint        AT %rLsGui29.4;    *(%rLsGui29.4) Enable microjoint option
EnNoPiercing        AT %rLsGui29.5;    *(%rLsGui29.5) No piercing execution
EnNoLeadIn          AT %rLsGui29.6;    *(%rLsGui29.6) No lead-in execution
AllPrePiercing      AT %rLsGui29.7;    *(%rLsGui29.7) Force all pre-piercing
AutoNozzleClean     AT %rLsGui29.9;    * (%rLsGui29.9) Nozzle clean active

EnWriteBlendSwitch  AT %gIso19.0;      *(gIso19.0) Enable blendSwitch PLC writes (Lunette da ISO)
CmdBlendSwitch      AT %gIso19.1;      *(gIso19.1) =0 park, 1=follow
EnabCheckPipePos    AT %gIso19.2;      *(gIso19.2) Enable checking pipe position in order to follow the jaw

EnWriteBlendSwitch1 AT %gIso39.0;      *(gIso39.0) Enable blendSwitch PLC writes (Lunette da ISO)
CmdBlendSwitch1     AT %gIso39.1;      *(gIso39.1) =0 park, 1=follow
EnabCheckPipePos1   AT %gIso39.2;      *(gIso39.2) Enable checking pipe position in order to follow the jaw
EnWriteBlendSwitch_Hor AT %gIso39.3;    
CmdBlendSwitch_Hor     AT %gIso39.4;    

EnWriteBlendSwitch2 AT %gIso49.0;      *(gIso39.0) Enable blendSwitch PLC writes lateral (Lunette da ISO)
CmdBlendSwitch2     AT %gIso49.1;      *(gIso39.1) =0 park, 1=follow
EnabCheckPipePos2   AT %gIso49.2;      *(gIso39.2) Enable checking pipe position in order to follow the jaw

ParkUn71Sup_ISO     AT %gIso29.1;
ParkUn72Sup_ISO     AT %gIso29.2;
ParkUn73Sup_ISO     AT %gIso29.3;
ParkUn74Sup_ISO     AT %gIso29.4;
ParkUn75Sup_ISO     AT %gIso29.5;
ISO_Loader_Retrace  AT %gIso29.6;
iso_G510_Running    AT %gIso29.7;
iso_G520_V_Unload   AT %gIso29.8;
iso_G580_Running    AT %gIso29.9;
iso_G510_StartDM4   AT %gIso29.10; 
ParkUn51Sup_ISO     AT %gIso29.11;
ParkUn52Sup_ISO     AT %gIso29.12;
ParkUn53Sup_ISO     AT %gIso29.13;
ParkUn54Sup_ISO     AT %gIso29.14;
ParkUn55Sup_ISO     AT %gIso29.15;
*                   AT %gIso29.16;
iso_G520_71_Unload  AT %gIso29.17;
iso_G520_72_Unload  AT %gIso29.18;
iso_G520_73_Unload  AT %gIso29.19;
iso_G520_74_Unload  AT %gIso29.20;
iso_G520_75_Unload  AT %gIso29.21;
iso_VshapeSecFollow AT %gIso29.22;

Iso_G520HorUnload_Ax85 AT %gIso29.23;
Iso_G520HorUnload_Ax86 AT %gIso29.24;
Iso_G520HorUnload_Ax87 AT %gIso29.25;
Iso_G520HorUnload_Ax88 AT %gIso29.26;

SecFollow_51        AT %ax51.rgau1;
SecFollow_52        AT %ax52.rgau1;
SecFollow_53        AT %ax53.rgau1;
SecFollow_54        AT %ax54.rgau1;
SecFollow_71        AT %ax71.rgau1;
SecFollow_72        AT %ax72.rgau1;
SecFollow_73        AT %ax73.rgau1;
SecFollow_74        AT %ax74.rgau1;
SecFollow_75        AT %ax75.rgau1;

TubeMeasure_Y       AT %gIso23;        * Measure of pipe along Y
TubeMeasure_Z       AT %gIso24;        * Measure of pipe along Z

ProbeThreshold      AT %gIso30;        *(gIso30) probe threshold (G2292)

U_PosAfterHeading   AT %gIso31;        *(gIso31) U axis position after heading
IsoMemTimeout1      AT %gIso35;        *(gIso35) Timer da utilizzare come timeout da ISO
IsoMemTimeout2      AT %gIso36;        *(gIso36) Timer da utilizzare come timeout da ISO
PreActbld           AT %gIso37;
IN_M700_M704        AT %gIso38;

SafetyForG800       AT %C123;        * Safety for regulation active without reach the setpoint

LogVarPlc           AT  %LsPlc65;    * Used for Log

ManualLoading       AT %LsIso21.1;    *(LsIso21.1) Manual Loading tube
HoldFromIso         AT %LsIso21.0;    *(LsIso21.0) Hold Iso Request 
StartPosFocale      AT %LsIso86;      *(LsIso86) Comando Posizione Focale

StopEndTrackGeometry AT %LSRPlcOp0.7;  * (LSRPlcOp0.7) stop at the end of the track/geometry
StopEndPiece         AT %LSRPlcOp0.19; * (LSRPlcOp0.19) stop at the end of the piece
WorkListEnabled      AT %LSRPlcOp0.5;  * (LSRPlcOp0.5) Worklist mode enabled
NozzleCleanPresent   AT %LSRPlcOp1.5;  * (LSRPlcOp1.5) Nozzle clean active

LeadInStart         AT %LsIso65;    * SignaL lead in in progress
LimitSpeed          AT  %LsIso82;     * %LsIso82 Limit regualtion speed at 0 for APA
ExePipeCalib        AT  %CalPipe.General.3;      * (CalPipe.General.3) Exe Pipe Calibration
PipeCalibTyp        AT  %CalPipe.CalibrType;

***************************************
*   REFRESH AXES POINTER (GRAPHICS)
***************************************
DYNGRAPH0           AT %cn0.dyngraph2;      ** (Asse X reale) Gestione SysWide Dinamici per la grafica.
DYNGRAPH1           AT %cn0.dyngraph3;      ** (Asse Y reale) Gestione SysWide Dinamici per la grafica.
MATPRG_STS1         AT %matPrg0.sts1;       ** (Asse X reale)
MATPRG_STS2         AT %matPrg0.sts2;       ** (Asse Y reale)
MATPRG_MATC         AT %matPrg0.MatCount;   ** (MatCount)

ORIGPRGX            AT %origprgX;       * Origine programmata X
ORIGPRGY            AT %origprgY;       * Origine programmata Y


DryRunOption       AT %PlcOp0.2;            ** Macchina in modo Dry Run
PLC_SEL_REF        AT %ui16.1;              * ui16.1 Selezione Modo REF da Plc
PLC_SEL_JOG        AT %ui16.2;              * ui16.2 Selezione Modo JOG da Plc
PLC_SEL_INC        AT %ui16.3;              * ui16.3 Selezione Modo INC da Plc
PLC_SEL_MDI        AT %ui16.4;              * ui16.4 Selezione Modo MDI da Plc
PLC_SEL_AUT        AT %ui16.5;              * ui16.5 Selezione Modo AUT da Plc
PLC_SEL_DRYRUN     AT %ui16.12;             * ui16.12 Selezione esecuzione DRY RUN da Plc
PLC_SEL_SINGREF    AT %ui16.28;             * ui16.28 Selezione REF singolo
PLC_SEL_PASSTHRU   AT %ui30.0;              * ui30.0 abil. esecuzione passante da UI
PLC_APPERR         AT %APPerr;              * Errori Applicativo
INDICE_OR          AT %IndexORG;            * Indice origine selezionata
VAL_X_OR           AT %origprgX;            * (origprgX)Offset X Origine
VAL_Y_OR           AT %origprgY;            * (origprgY)Offset Y Origine
VAL_ROTEA          AT %rotEA;               * (rotEA)Rotazione origine selezionata

Invert_Open_Close       AT %rLsGui0.5;    *(rLsGui0.5)  Yang
P_Etichettatrice_enable AT %rLsGui0.6;    *(rLsGui0.6)  Da GUI Dis\Abilitazione Etichettatrice
P_Decoiler_enable       AT %rLsGui0.7;    *(rLsGui0.7)  Da GUI Dis\Abilitazione Decoiler
Automatic               AT %rLsGui0.8;    *(rLsGui0.8)  Precitec: Auto da pagina video [LaserPage]
EnTiptouchRetrace       AT %rLsGui0.9;    *(rLsGui0.9) Enable restart from tip touch using retrace
Pls_AspAuto             AT %rLsGui0.11;   *(rLsGui0.11) Pulsante Abilitazione Auto Aspiratore
Pls_AspType             AT %rLsGui0.12;   *(rLsGui0.12) Pulsante Tipo comando ON\OFF Aspiratore

SheetOriginSearc   AT %rLsGui0.10;   *(rLsGui0.10) Abilitazione Ricerca origine lamiera (APA)
OriginClrLength    AT %rLsGui0.13;   *(rLsGui0.13) Clear tube length in the machine when acquire origin
OriginAtStar       AT %rLsGui0.16;   *(rLsGui0.16) Force acquire origin 1 point when start is pushed
AutoCapOnTube      AT %rLsGui0.17;   *(rLsGui0.17) Automatical cap calibration on tube after acquire origin
AutoWeldDetec      AT %rLsGui0.18;   *(rLsGui0.18) Automatical weld seam detection
EnAutoDoor         AT %rLsGui0.19;

CALIBRATION_CHECK  AT %C26.0;        *(C26.0) Calibration control in run    
Nozzle_clean_run   AT %C26.1;        *(C26.1) Nozzle cleaning in run
CAP_LINEAR_RUN     AT %C26.2;        *(C26.2) Linearizzazione capacitivo in esecuzione
ISOTipTouchDisable AT %C26.3;        *(C26.3) Disabilita tip touch da ISO

ResAPAfunc         AT %C57.1;          * (C57.1) Reset APA function
exe1000            AT %C57.3;          * (C57.3) Exexute 1000.pgm to refresh graphic origin and angle    
                   
qCut_X_start       AT %rLsGui48;     *(rLsGui48) Manual cut: X quota start
qCut_X_end         AT %rLsGui49;     *(rLsGui49) Manual cut: X quota end
qCut_Y_start       AT %rLsGui50;     *(rLsGui50) Manual cut: Y quota start
qCut_Y_end         AT %rLsGui51;     *(rLsGui51) Manual cut: Y quota end
qBorder            AT %rLsGui52;     *(rLsGui52) Manual cut: distance from border                      
PercTimePierc      AT %rLsGui53;     *(rLsGui53)  Time percentage for?piercing?alarm

PiercCounter            AT %C71;           *(%C71) Piercing counter                 

hmiForceInputs     AT %C99.0;   * C99.0 Auxiliary to Force Inputs
hmiForceOutputs    AT %C99.4;   * C99.4 Auxiliary to Force Outputs
HOERRB_CAL_REQ     AT %C99.5;   * C99.5 HOERBIGER CALIBRATION REQUEST
Touch_enab         AT %C99.8;   * C99.8 touch enable
Regol_enab         AT %C99.9;   * C99.9 regulator enable
Aux_Req_CpyW       AT %C99.10;
STATUS_LOADER_HOLD AT %C99.11;
GESTAX36_xHoldMoving AT %C99.12;   

piercing_run       AT %LsPlc37;   * (LsPlc37) Piercing in run
PiercingInRun      AT %LsPlc70;   * (LsPlc70) Piercing in run
TipTouchDisable    AT %LsPlc71.0; * (LsPlc71.0) TipTouchDisable (ritardato)

RemoteContrShot    AT %LsPlc71.1; * (LsPlc71.1) Remote controller shoot request

EnabSetupGantry    AT %gGui0.0;             *(gGui0.0)Abilita Setup GAntry ed "Escludi allineamento a fine taratura"                
EnableSheetCont    AT %gGui0.6;             *(gGui0.6) Enable sheet control dimension

guiTAB_Sinottico   AT %rGui10;              *(rGui10)GUI TAB Sinottico for Quotes

QtaSizeSlave       AT %ax30.pa18;           * %ax30.pa18 Quota di taratura asse Slave Gantry

ax27_InSetPoint     AT %ax27.ra19.2;        * %ax27.ra19.2 Setpoint in execution for Axis 27 - Xr
ax28_InSetPoint     AT %ax28.ra19.2;        * %ax28.ra19.2 Setpoint in execution for Axis 28 - Yr
ax29_InSetPoint     AT %ax29.ra19.2;        * %ax29.ra19.2 Setpoint in execution for Axis 29 - Zr
ax30_InSetPoint     AT %ax30.ra19.2;        * %ax30.ra19.2 Setpoint in execution for Axis 30 - Ur

* User simplify parameters settings
Kp_X                  AT %ax27.pa10;           * x axis Kp parameter
Kp_Y                  AT %ax28.pa10;           * y axis Kp parameter                   
Jerk_X                AT %ax27.pa25;           * x axis Jerk parameter
Jerk_Y                AT %ax28.pa25;           * y axis Jerk parameter
Fir_1_X               AT %ax27.TRA_AuxF;       * x axis fir 1 costant 
Fir_1_Y               AT %ax28.TRA_AuxF;       * y axis fir 1 costant 
Fir_2_X               AT %ax27.TRA_AuxG;       * x axis fir 2 costant
Fir_2_Y               AT %ax28.TRA_AuxG;       * y axis fir 2 costant
Ffw_X                 AT %ax27.TRA_AuxH;       * x axis ffw 
Ffw_Y                 AT %ax28.TRA_AuxH;       * y axis ffw 
X_acc                 AT %ax27.pa1;            * x axis max acceleration
Y_acc                 AT %ax28.pa1;            * y axis max acceleration
X_dec                 AT %ax27.pa2;            * x axis max deleration  
Y_dec                 AT %ax28.pa2;            * y axis max deceleration

Kp_Gant               AT %ax30.pa10;           * U/V axis Kp parameter                  
Jerk_Gant             AT %ax30.pa25;           * U/V axis Jerk parameter
Fir_1_Gant            AT %ax30.TRA_AuxF;       * U/V axis fir 1 costant 
Fir_2_Gant            AT %ax30.TRA_AuxG;       * U/V axis fir 2 costant
Ffw_Gant              AT %ax30.TRA_AuxH;       * U/V axis ffw 
Gant_acc              AT %ax30.pa1;            * U/V axis max acceleration
Gant_dec              AT %ax30.pa2;            * U/V axis max deleration  


XY_Kp                AT %LsGui41;       *(LsGui41) Axes position gain
XY_Jerk              AT %LsGui42;       *(LsGui42) Axes jerk
XY_Acc               AT %LsGui43;       *(LsGui43) axes acceleration
XY_Dec               AT %LsGui44;       *(LsGui44) axes deceleration
XY_TimeCostant_1     AT %LsGui45;       *(LsGui45) fir 1
XY_TimeCostant_2     AT %LsGui46;       *(LsGui46) fir 2
XY_FFW               AT %LsGui47;       *(LsGui47) feed forward 

kp_Calc              AT %LsGui48;       *(LsGui48) Kp for calculation 
Rpm_Calc             AT %LsGui49;       *(LsGui48) Rpm for Kp drive Calc
KpDriveRes           AT %LsGui50;       *(LsGui50) Kp drive 

Gui_btn_CW           AT %LsGui71.0;
Gui_btn_CCW          AT %LsGui71.1;
Gui_btn_NoGear       AT %LsGui71.2;     *() Start with track not 0, without 3rd spindle
Gui_btn_Gear         AT %LsGui71.3;     *() Start with track not 0, with 3rd spindle geared
HMI_PagePop          AT %LsGui71.4;
Gui_Pul_X_M          AT %LsGui71.5;
Gui_Pul_X_P          AT %LsGui71.6;
Gui_Pul_X1_M         AT %LsGui71.7;
Gui_Pul_X1_P         AT %LsGui71.8;
Btn_Weld_Piston      AT %LsGui71.9;
Gui_Pul_X3_M         AT %LsGui71.10;
Gui_Pul_X3_P         AT %LsGui71.11;
gui_binup            AT %LsGui71.12;
Btn_Weld_En          AT %LsGui71.13;
Gui_Pul_Y_M          AT %LsGui71.14;
Gui_Pul_Y_P          AT %LsGui71.15;
Gui_Pul_Z_M          AT %LsGui71.16;
Gui_Pul_Z_P          AT %LsGui71.17;

Gui_Saw_rpm          AT %LsGui72;
Gui_Saw_DRDH         AT %LsGui73;
Gui_Saw_Kerf         AT %LsGui75;

PiercToClean         AT %LsGui81;       *(%LsGui81) Piercing number to allow nozzle cleaning
SpaceForLeadCom      AT %LsGui82;       *(%LsGui82) Costant to compensate no linear lead in (max value of compensation)
***************
NumTraccia         AT %R18;             * Numero traccia in esecuzione

SawSupWidth        AT %R24;
*                  AT %R25;
*                  AT %R26;
SawUserThick       AT %R27;
SawUserRpm         AT %R28;
SawUserVel_Z       AT %R29;
SawCutPreRead      AT %R30;
SawLaserTab        AT %R31;             * Gui Main Page
ST_TarVar65        AT %R34;
SumHeadTracks      AT %R35;             * Sum of the heading track
ST_TarVar66        AT %R36;
ST_TarVar67        AT %R37;
weldseam_offset    AT %R38;

READY_FOR_VERIFY    AT %R50.0;          * (R50.0) CH0 Program ready to verify

MainSpdLastOpen     AT %R51.0; 
MidSpdLastOpen      AT %R51.1;
SlaveSpdLastOpen    AT %R51.2; 
CorOptNoStart       AT %R51.3; 

StartTimer          AT %LsIso30;        * Start tim 
PierceCutGasSelect  AT %LsIso31;        * (LsIso31) Select piercing gas or cutting gas (0=cut gas,1=piercing gas)
P_IDax_U            AT %LsIso32;  
P_Qtaax_U           AT %LsIso33; 
P_Velax_U           AT %LsIso34;
P_IDax_C            AT %LsIso22;  
P_Qtaax_C           AT %LsIso23; 
P_Velax_C           AT %LsIso24;
P_EndPos_U          AT %LsIso35.0;
P_EndPos_C          AT %LsIso35.1;
P_RunPos_U          AT %LsIso35.2;
P_RunPos_C          AT %LsIso35.3;

M700_BLEND          AT %LsIso35.4;
CN3_REQ_VYZ_ISO     AT %LsIso35.5;

Timesec            AT %R123;                *(R123) Tempo secondi
TimeOur            AT %R124;                *(R124) Tempo ore
TimeSecProg        AT %R125;                *(R125) Conteggio Secondi Tempo Programma Taglio in corso
TimeMinProg        AT %R126;                *(R126) Conteggio Minuti Tempo Programma Taglio in corso

SelectorApp        AT %vpbSelectors0;           * Selector of vrtc parameters head 1

guiBtt_HydrPump_ON                     AT %LsGui32.0;    *(LsGui32.0) GUI Button Manual Hydraulic Pump ON
guiBtt_HydrPump_OFF                    AT %LsGui32.1;    *(LsGui32.1) GUI Button Manual Hydraulic Pump OFF
guiBtt_Vacuum_ON                       AT %LsGui32.2;    *(LsGui32.2) GUI Button Manual Vacuum System ON
guiBtt_Vacuum_OFF                      AT %LsGui32.3;    *(LsGui32.3) GUI Button Manual Vacuum System OFF
guiBtt_Lubric_ON                       AT %LsGui32.4;    *(LsGui32.4) GUI Button Manual Lubrication Cicle ON
guiBtt_Lubric_OFF                      AT %LsGui32.5;    *(LsGui32.5) GUI Button Manual Lubrication Cicle OFF
guiBtt_LockDoor                        AT %LsGui32.6;    * GUI Button Request Lock Operator Door
guiBtt_UnLockDoor                      AT %LsGui32.7;    * GUI Button Request Unlock Operator Door
                                       
BtnResetCounter                        AT %LsGui32.28;    *(LsGui32.28) Reset counter of piercing


varGESTCH0_xPPNumber        AT %LsPlc49;       *(LsPlc49) Program selected for excution in CH0

Pallet_Chng_Ext_Plc         AT %LsPlc48.10;    *(LsPlc45.10)Axes safe position and Pallet change request to external plc
gainFomFastPlc              AT %LsPlc56;       *(LsPlc56) Fast Plc gain for vrtc
LensPosEnd                  AT %LsPlc63.0;     ** (LsPlc63.0) Lens positioning finish

JumpingInProgres            AT %LsPlc63.3;     * (%LsPlc63.3) Jumping in progress

Purge_time                  AT %rLsPlc36;       *(rLsPlc36) Time for purging
rVar_Remain_Time_To_Lub     AT %rLsPlc50;       *(rLsPlc50) Retentive Var with time remaining for Lubrication

iso_exem30_fast             AT %LsIso9.0;       *(LsIso9.0) Execute only M30
iso_pipeinside_mngt         AT %LsIso9.1;       *(LsIso9.1) Sincronismo per la gestione della quota di tubo in macchina
iso_sawcut_G200             AT %LsIso9.2; 
iso_bincmd                  AT %LsIso9.3;            
iso_G520_Running            AT %LsIso9.4;
iso_LsronCutting            AT %LsIso9.5;
iso_ResetCmd                AT %LsIso9.6; 

iso_Calibration_OK          AT %LsIso10.0;     *(LsIso10.0) ISO said Sensor Calibration OK
iso_CalibContr_ok           AT %LsIso10.1;     *(LsIso10.1) Iso said calibration control done  
isoLowLeadIn                AT %LsIso10.2;     *(LsIso10.2) Enable low speed reduction in lead-in  
isoLoadRecord               AT %LsIso10.3;     *(LsIso10.3) Load database record with Block search in run
ext_change_comm             AT %LsIso10.4;     *(LsIso10.4) external plc change pallet command 
axes_safe_position          AT %LsIso10.5;     *(LsIso10.5) send axes in safe posiotion message
Pallet_change_run           AT %LsIso10.6;     *(LsIso10.6) Pallet change ext plc in run   
EnSpeedPiercing             AT %LsIso10.7;     *(LsIso10.7) Enable speed for piercing in Near phase
ProgCounterAtZero           AT %LsIso10.8;     *(LsIso10.8) Program counter is at 0   
CalibOK                     AT %LsIso10.9;     *(LsIso10.9) Calibration control OK


CurrentLineExecute          AT %LsIso11;        *(LsIso11) Current cutting line parameter to execute 
SpeedForPiercing            AT %LsIso12;        *(LsIso12) Speed for Z axi during piercing
TruLaserRecord              AT %LsIso13;        *(LsIso13) Selection of recod in trumpf laser resonator
ZoomValueIso                AT %LsIso14;        *(LsIso14) Zoom value from Iso
DisabFir_CH0_axes           AT %LsIso15;        *(LsIso15) Disab FIR for all CH0 axes
DisabFir_CH3_axes           AT %LsIso16;        *(LsIso16) Disab FIR for all CH0 axes
DisabTipTouch               AT %LsIso17.0;      *(LsIso17.0) Disable tip touch
gG800GStatus                AT %LsIso18;        *(LsIso18) G800 Gxx Status

req_graph_refr              AT %LsPlc48.18;    *(LsPlc48.18) Req graph refresh 

SafePipeZone                AT %LSRCostK9;     * (%LSRCostK9) Safety distance from axis limit in pipe management
TipTouchRetraceTim          AT %LSRCostK12;    * (LSRCostK12) Tempo retrace tip touch con gestione autorestart
TipTouchNumTentativi        AT %LSRCostK13;    * (LSRCostK13) Numero tentativi per tip touch con gestione autorestart
TipTouchHoldQtaZup          AT %LSRCostK14;    * (LSRCostK14) Quota inc di svincolo Z dopo hold o tip touch
userdistance                AT %LSRCostK54;    * (LSRCostK54) User distance retrace
CheckDimensSpeed            AT %LSRCostK55;    * (LSRCostK55) Speed for sheet control

OpzXmaster         AT %ax0.pa3.25;          *(ax0.pa3.25) Selects X axis as Gantry Master
OpzYmaster         AT %ax1.pa3.25;          *(ax1.pa3.25) Selects Y axis as Gantry Master
U_GANTRY_SEL       AT %cn0.cc1.3;           *(cn0.cc1.3) Selects U axis as Gantry Slave on Channel 0
V_GANTRY_SEL       AT %cn0.cc1.4;           *(cn0.cc1.4) Selects Y axis as Gantry Slave on Channel 0
A_AXE_SEL          AT %cn0.cc1.6;           *(cn0.cc1.6) Selects A axis (Pipe) on Channel 0
B_AXE_SEL          AT %cn0.cc1.7;           *(cn0.cc1.7) Selects B axis (Pipe) on Channel 0
RotazSuAxeY_1      AT %twi1.htwi.oarh.dEB;  *(twi1.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI1
RotazSuAxeY_2      AT %twi2.htwi.oarh.dEB;  *(twi2.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI2
RotazSuAxeY_3      AT %twi3.htwi.oarh.dEB;  *(twi3.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI3
RotazSuAxeY_4      AT %twi4.htwi.oarh.dEB;  *(twi4.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI4
RotazSuAxeY_5      AT %twi5.htwi.oarh.dEB;  *(twi5.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI5
RotazSuAxeY_6      AT %twi6.htwi.oarh.dEB;  *(twi6.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI6
RotazSuAxeY_7      AT %twi7.htwi.oarh.dEB;  *(twi7.htwi.oarh.dEB)Parametro Rotazione intorno all asse Y TWI7

ISO_O_CALIB_REQ         AT %C30.7;          * (C30.7) Calibration request from ISO
ISO_O_CALIB_STROBE      AT %C30.8;          * (C30.8) Calibration strobe from ISO

INP_flyCutModeON         AT %C125.0;        ** application turn flyCut mode ON
OUT_isRasterModeON       AT %C125.2;        ** application raster mode is ON

INP_rasterModeON         AT %C124.2;     ** application turn raster mode ON

EnValOut1_G834           AT %TstP11.14;     *(TstP11.14) Abilita Attuazione Valori Uscite (G834/G835)
EnValOut2_G834           AT %TstP11.15;     *(TstP11.15) Abilita Attuazione Valori Uscite (G834/G835)

EnValOut1                AT %TstP14.14;     *(TstP14.14) Abilita Attuazione Valori Uscite
EnValOut2                AT %TstP14.15;     *(TstP14.15) Abilita Attuazione Valori Uscite

VerNozzlelost            AT %TstP15.0;      *(TstP15.0) Verifica NOZZLE LOST
EnCommutValvole          AT %TstP15.1;      *(TstP15.1) Abilita Commutazione Valvole
LaserOFF_REQ             AT %TstP15.2;      *(TstP15.2) (LASER_OFF) Turn off laser from Iso
LaserOn_REQ              AT %TstP15.3;      *(TstP15.3) (LASER_ON)

* VarInOut pd_spline[] management

nSpan               AT %pd_nSpans;
XSTART              AT %pds_xs;
XEND                AT %pds_xe;
YSTART              AT %pds_ys;
YEND                AT %pds_ye;
COEFICIENT_0        AT %pds_a0;
COEFICIENT_1        AT %pds_a1;
COEFICIENT_2        AT %pds_a2;
COEFICIENT_3        AT %pds_a3;
COEFICIENT_4        AT %pds_a4;
COEFICIENT_5        AT %pds_a5;
SPAN_NUM            AT %pds_ps; 

RegCommand1              AT %TstP10;        *(TstP10) Disabble the regulation and simulation of the sensor

Val_Focale_ACT      AT %TabLsr0.L_UserInt2;         * Valore Focale da\a GUI
AzzeraFocGui        AT %LsGui0.7;                   * comando azzeramento da pagina video

Btn_Cpy_W           AT %gGui79.2; 
tmpartOffset        AT %tmParts.TP_feedOffset;
END_VAR

