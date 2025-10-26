//-------------------------------------------------------------------------
//                      DEFCN.APP
//-------------------------------------------------------------------------
#include <defcnAPP.h>
SECTION Application

typedef struct {
    dword ofX;                  // Origine X del campo
    dword ofY;                  // Origine Y del campo
    dword ofZ;                  // Origine Z del campo
    dword ofU;                  // Origine U del campo (non ancora gestito)
    dword ofV;                  // Origine V del campo (non ancora gestito)
    dword ofW;                  // Origine W del campo (non ancora gestito)
} ORIGINI;

typedef struct {
  byte          Enab;
  double        origprgX;    // SHF[X] origine programma
  double        origprgY;    // SHF[Y] origine programma
  double        origprgPipe; // SHF[Pipe] origine programma per asse Tubo
  double        shfX;        // SHF[X]
  double        shfY;        // SHF[Y]
  double        rotEA;       // ROT[EA]
  dword         xprg;        // G92 X Y
  dword         yprg;        // G92 X Y
  dword         nSub;        // numero ciclo fisso da eseguire
} OFFSET;

typedef struct{
  string  L_Name[80];
  dword   L_WorkType;
  dword   L_Code;
  string  L_Material[16];
  dword   L_Tickness;
  dword   L_BevelAngle;
  double  L_PierceHeight;
  dword   L_PierceTime;
  double  L_CutDistance;
  dword   L_FeedA;
  dword   L_FeedB;
  dword   L_FeedC;
  dword   L_FeedMarking;
  dword   L_FeedDecoating;
  double  L_KerfA;
  double  L_KerfB;
  double  L_KerfC;
  double  L_SafeDistance;

  dword   L_PiercePower;
  dword   L_PierceFreq;
  dword   L_PierceDuty;
  dword   L_PierceGas;
  dword   L_PiercePressure;
  double  L_PierceFocal;

  dword   L_CutPower;
  dword   L_CutFreq;
  dword   L_CutDuty;
  dword   L_CutGas;
  dword   L_CutPressure;
  double  L_CutFocal;

  dword   L_PiercingMode;

  dword   L_UserInt[20];  // 0 - Piercing Standoff
                          // 1 - Piercing Aria Assist.
                          // 2 - Cut Standoff
                          // 3 - Cut Jerk
                          // 4 - P0
                          // 5 - V0
                          // 6 - V1
                          // 7 -
                          // 8 - Duty Min
                          // 9 - Kp lavorazione VRTC

  double  L_UserDbl[20];
  dword   L_SourceInt[20];
  double  L_SourceDbl[20];
  dword   L_Ugello;
  dword   L_HeadCalibTableNum;      // Indice tabella di calibrazione interna testa (parte da 0)
  
  // Per futuri utilizzi (dati in funzione angolo bevel)
  //word    L_Angle_DataTab;
  //word    L_Angle_DataEnab;
  //dword   L_Angle_Power;
  //dword   L_Angle_Duty;
  //dword   L_Angle_Freq;
  //dword   L_Angle_Pressure;
  //dword   L_Angle_Focal;
  //dword   L_Angle_TempA;
  //dword   L_Angle_TempB;
}TAB_LSR;

typedef struct {
    string msg[NUM_MSG_LEN];
} TBLSTR;

//Dati per ogni singola testa
typedef struct{
  double  old_td_z;
  dword   Epsilon;
  double  uxh_Thickness;             //********** TODO

  byte    touch_mode;
  byte    regol_mode;
  byte    piercing_mode;

  double  a_x;
  double  a_y;
  double  a_z;

  //Attualmente specializzati per Plasma
  dword   OffsetPiercing;
  dword   OffsetCut;
  dword   V_Arco;

  //Variabili Per TCP e VRTC: Registri da ISO Per "TCP"(Tool Center Point)
  dword   TCP[NUM_APP_TCP];

  //Attualmente specializzati per Waterjet
  dword   uxh_WtIso[10];
  dword   uxh_WtPlc[10];

  //Attualmente specializzati per Plasma
  dword   uxh_PlIso[10];
  dword   uxh_PlPlc[10];

  //Attualmente specializzati per Laser
  dword   uxh_LsIso[10];
  dword   uxh_LsPlc[10];

  //
  // DATI PER OGNI TESTA
  //
  // XXX NOTA:
  // XXX 1) Nel caso serva sapere quale testa e' attiva sull'unita' usare gTravT.
  // XXX 2) Con ossitaglio sono attive piu' unita' simultaneamente, ma non ci
  // XXX    sono piu' routine che girano con diversi numeri di istanza,
  // XXX    attualmente:  iterare uxh[] ed attivare i cannelli con gTravT != 0.
  //
  byte    uxh_gTravT;
  byte    uxh_gTravH;
  byte    uxh_gTravD;
  double  uxh_gTravS;
  byte    uxh_gTravE;
  //assi a predisposizione con multiutensile
  double  uxh_SpacingX;
  double  uxh_SpacingY;
  //assi a predisposizione
  double  uxh_BvlAngC;
  double  uxh_BvlAngA;
  //taper
  double  uxh_Lead;
  double  uxh_Tilt;
  //flags per G806
  dword   uxh_Flags;
}UNIT_WORK_HEAD;

typedef struct{
  UNIT_WORK_HEAD uxh[12];

  //DATI TESTA PRINCIPALE
  byte    gTravT;
  byte    gTravH;
  byte    gTravD;
  double  gTravS;
  byte    gTravE;
  byte    gKerfDescr;
  double  gKerfSvr;
  //assi a predisposizione
  double  BvlAngC;
  double  BvlAngA;
}UNIT_WORK;

typedef struct{
  dword  UL_gTravTech;
  //
  // Unità associata
  // (0=usa gTravT; 1,2,... usa unità 1,2,...; 256=usa unita' 0)
  //
  dword  UL_gTravUnit;
}UNIT_LUN;


//*******************************************************************
//  REFERENCE SWITCH PROGRAMMING
//*******************************************************************
typedef struct {
    dword lsrefmode;            // Axes ref mode (0=use ref switch NO, 1=use ref switch NC,2=use MAX limit switch, 3=use MIN limit switch)                    
    dword lsreflevel;           // ref switch (1=NC, 0=NO)
} AX_LSREF;


//*******************************************************************
//  ASPIRATOR
//*******************************************************************
typedef struct {
    dword GV_ParDW[10];         // Parametri generici DWORD
    dword GV_ParB[10];          // Parametri generici BYTE
} GEN_VASCHE;

typedef struct {
    dword QStart;               // Quota inizio X bocca di aspirazione
    dword QEnd;                 // Quota fine X bocca di aspirazione
    dword QStartY;              // Quota inizio Y bocca di aspirazione
    dword QEndY;                // Quota fine Y bocca di aspirazione
} BOCCA;


//*******************************************************************
//  FIR FILTER
//*******************************************************************
typedef struct {
    dword ff_fifo[100];         // fifo campioni
    dword ff_index;             // indice
    dword ff_out;               // uscita
} S_FIR;


//*****************************************************************************
//  CAPACITIVE
//*****************************************************************************
typedef struct{
    word cDigIn[2];             // Digital IN
    word cDigOut[2];            // Digital OUT
    word cAnaIn[2];             // Analog IN
    word cButton[2];            // Buttons
    word cLeds[2];              // Leds
    RETAIN dword cPar[10];      // Parameters
    RETAIN dword cOpt[2];       // Options
} CAPACITIVE;

typedef struct {  
            dword    Far_signal;
            dword    Nozzle_lost;  
            dword    tip_touch;  
            dword    calibrareq;	
            dword 	 cable_cut;			
            dword    curcapval;   
   RETAIN   dword    FARcapval_Ang;				
   RETAIN   dword    maxcapval;
   RETAIN   dword    mincapval;				
   RETAIN   dword    capval[30];     
} EM_CAP;  

//*****************************************************************************
//  RESONATOR
//*****************************************************************************
typedef struct{
    word DigIn[2];              // Digital IN
    word DigOut[2];             // Digital OUT
    word rAnaIn[5];
    word rAnaOut[2];
    word Button[2];
    word Leds[2];
    RETAIN dword Par[10];
} RESONATOR;

//*****************************************************************************
//  FOCAL: with PWM
//*****************************************************************************
typedef struct{
    dword   pwDutyCalcuSec;     // duty (uSec)
    dword   pwSetPosMAN;        // Position setting in manual mode
    RETAIN dword pwPar[10];
} FOCAL_PWM;

//*****************************************************************************
//  LASER MANAGEMENT
//*****************************************************************************
typedef struct{
    word    AnaOut[2];          // Analog OUT
    dword   VisPowerOut;
    dword   VisDutyOut;
    dword   VisFreqOut;
    word    lButton[2];
    RETAIN dword   ParMaxPower;
    RETAIN dword   ParIntSpeedFilt;
    RETAIN dword   ParDACres;
    RETAIN dword   ParOptions;
    RETAIN dword   ShootPower;
    RETAIN dword   ShootFreq;
    RETAIN dword   ShootDuty;
    RETAIN dword   ShootTime;
    RETAIN dword   ShootGasType;
    RETAIN dword   ShootGasPress;
} LSR_GEST;

//*****************************************************************************
//  LASER MECH HEAD
//*****************************************************************************
typedef struct{
    byte    OutA;               // Interface ISO-PLC
    byte    CalibRun;           // Calibration running
    byte    CalibTmout;         // Calibration timeout
    byte    TouchErr;           // Touch error
    byte    FaultErr;           // Fault error
    dword   FullScale;          // Full scale (Volt)
    dword   ADCres;             // ADC resolution (4096,...)
    dword   IWnum;              // Analog input number of capacitive
    dword   CapacitiveVal;      // Capacitive value (ADC points)
    dword   CapacitiveVolt;     // Capacitive value (XX.XX volt)
    dword   lmPar[5];           // Generic parameters
                                // lmPar[0] = Z axis speed for search TOUCH input (mm/min)
} LSRMECH_MNG;

//*****************************************************************************
//  CAPACITIVE
//  LASER CAPACITIVE MEMORIES
//*****************************************************************************
typedef struct{

          byte    TipTouch;
          byte    Ready;
          byte    Error;
          byte    Charac_Curve;
          byte    Lineariz;
          byte    Strobe;                 // used for handshake
          byte    Acknowledge;            // used for handshake
          byte    Far;
          byte    NozzleError;
          byte    BodyTouch;
          byte    CalibStart;
          byte    Adjustment;
    
          dword   GeneralPurpose[4];
  RETAIN  dword   rGeneralPurpose[4];

} CAPACITIVE_TYPE;

//*****************************************************************************
//  HEADS
//*****************************************************************************
//typedef struct {
//         dword  Input[2];
//         dword  Output[2];
          
//         dword  GeneralPurpose[10];
//RETAIN   dword  rGeneralPurpose[10];

//} HEAD_TYPE;

//*****************************************************************************
//  HEADS
//*****************************************************************************
typedef struct {

    word fDigIn[2];             // Digital IN             
    word fDigOut[2];             // Digital OUT            
    dword fAnaIn[2];             // Analog IN      
    dword fAnaOut[2];            // Analog Out            
    word fButton[2];            // Buttons                
    word fLeds[2];              // Leds                   
    RETAIN dword fPar[5];      // Parameters             
    RETAIN dword fOpt[2];       // Options                                                            

} HEAD_TYPE;


typedef struct {

         dword  CalibrType;           // 0 Table frame, 1 machine frame
 RETAIN  dword  BackupTable;          // Backup table for restore parameters
 RETAIN  dword  MaxErrTouchVert;      // Maximum error during vertical probing
 RETAIN  dword  MaxErrTouchHoriz;     // Maximum error during horizontal probing
 RETAIN  dword  CoordXCalib;          // Coordinate X for calibration
 RETAIN  dword  CoordYCalib;          // Coordinate Y for calibration
 RETAIN  dword  BorderDist;           // Border parameter for calibration validation
 RETAIN  dword  MinSide;              // Minimum side lenght for calibration validation
         
 RETAIN  dword  DimXPipe[2];          // Shape Information X
 RETAIN  dword  DimYPipe[2];          // Shape Information Y
 RETAIN  dword  DimZPipe[2];          // Shape Information Z
 RETAIN  dword  DimUPipe[2];          // Shape Information U
 RETAIN  dword  DimVPipe[2];          // Shape Information V
 RETAIN  dword  DimWPipe[2];          // Shape Information W
 RETAIN  dword  Round[2];             // Shape Information junction;          

 RETAIN  dword  Lenght[2];            // Lenght of the Pipe
 RETAIN  dword  Radius;               // Radius      
 RETAIN  dword  Side[4];              // Side 
 RETAIN  dword  RoundPar[2];          // round from gui
                          
 RETAIN  dword  General[4];           // GeneralUse                         
 
 // Output variable for the cycles        
 dword  Size_X;                       // Dimension of Pipe along X direction
 dword  Size_Y;                       // Dimension of Pipe along X direction
 dword  Size_Z;                       // Dimension of Pipe along X direction
 
 // User Correction Parameters
 RETAIN dword UsrPara[5];  
         
} PIPE_CAL;

//*****************************************************************************
//  OVERRIDE MANAGEMENT
//*****************************************************************************
typedef struct {
    dword Type;
    dword MaxPercent;
    dword MinPercent;
    dword SlowFeed;
    dword IncStep;
    byte  PulPlus;
    byte  PulMinus;
    dword Resolution;
    dword O_value;
} OVERRIDE;

//*****************************************************************************
//  Wireless Remote Controller
//*****************************************************************************
typedef struct {                                      
	word    Xhc_LineIdx[4];                             
    dword  	Xhc_Puls;                                 
} S_XHC_CONFIG;                                       
                                                                
//*****************************************************************************
//  LISTE DI LAVORO
//*****************************************************************************
// Gestione PLC/ISO
typedef struct{
    byte            wl_ListRunning;     // Lista in esecuzione
    byte            wl_EditLock;        // Editing lista bloccata
    dword           wl_parztime;        // Tempo di esecuzione taglio
    dword           wl_cmd;             // ----------------------------
                                        // bit0 = handshake STB trasmissione PROG al CN
    dword           wl_answer;          // ----------------------------
                                        // bit0 = handshake ACK trasmissione PROG al CN
    dword           wl_flag;            // Varie
                                        // bit0 = Codice DB passato dalla lista
    RETAIN dword    wl_Index;           // Indice linea lista attuale (parte da zero)
    RETAIN string   wl_Curret[80];      // Nome lista corrente caricata/esecuzione
    dword           wl_mngdwpar[20];    // Varie
} WLIST_MNG;

// Area dati
typedef struct{
    string  wl_parA[80];
    string  wl_parB[80];
    string  wl_parC[80];
    string  wl_parD[80];
    dword   wl_dwpar[20];                // Dati
    dword   wl_time;                     // Tempo di esecuzione taglio (sec)
    byte    wl_endl;                     // programma eseguito/escluso
} WLIST;

//generic spline (i.e. power delivery rate) polynomial span
typedef struct {
    double  pds_xs;     //curve initial X coordinate
    double  pds_xe;     //curve final   X coordinate
    double  pds_ys;     //curve initial Y coordinate
    double  pds_ye;     //curve final   Y coordinate
    double  pds_a[6];   //n-th degree coefficient, only [2..5] used!
    double  pds_ps;     //parameter span

} POWDEL_SPAN;

//generic spline (i.e. power delivery rate)
typedef struct {
    CHKSUM  dword       pd_nSpans;
    CHKSUM  POWDEL_SPAN pd_spans[MAX_POWDEL_SPANS];

    //User information
    CHKSUM dword    pd_userInfo[4];

    //Curve editor helper: parameters
    CHKSUM double   pdh_Xmin;
    CHKSUM double   pdh_Xmax;
    CHKSUM double   pdh_Ymin;
    CHKSUM double   pdh_Ymax;

    //Curve editor helper: component input data
    CHKSUM double   pdh_X[MAX_POWDEL_POINTS];
    CHKSUM double   pdh_Y[MAX_POWDEL_POINTS];
    CHKSUM word     pdh_nPoints;
    CHKSUM byte     pdh_interpType;

} POWDEL_SPLINE;


//====================================================================
// CALIBRATION management (when you use internal head table )
//====================================================================
typedef struct {
    RETAIN dword targ_z;
           dword user_d;
} TS_CALIB_DATA;


typedef struct {
    RETAIN byte             enab;
    RETAIN dword            targ_a_axis;
    RETAIN dword            targ_b_axis;
           TS_CALIB_DATA    data[20];
           dword            user_g[20];
} TS_CALIB;


//====================================================================
// VRTC parameters block (see PLC for by-parameter mapping)
//====================================================================
typedef struct {
    dword vpbp[30];
} VRTC_PARAM_BLOCK;

//====================================================================
// VRTC Surface Tool Management
//====================================================================

typedef struct {
    dword       uvhToolCommand[8];      //Head Tool VRTC user command
    dword       uvhToolStatus[8];       //Head Tool VRTC user status
    dword       uvhSurfaceCommand[8];   //Head Surface VRTC user command
    dword       uvhSurfaceStatus[8];    //Head Surface VRTC user status

    dword       uvhCommand[8];          //Head command for all VRTCs
    dword       uvhStatus[8];           //Head status for all VRTCs

} UserVrtcHead;


// RTCPCALIB
typedef struct {
           dword RCgPlc[100];
           dword RCWtPlc[100];
    RETAIN dword RCrWtPlc[100];
           dword RCWtGui[100];
    RETAIN dword RCrWtGui[100];
           dword RCgIso[100];
           dword RCrIso[50];
           dword RCTCP[100];
} RTCP_CALIB;

//====================================================================
//PLC real-time table based interpolation                             
typedef struct {                                                      
    CHKSUM  string  ptbiszDescription[20];                            
                                                                      
    CHKSUM  dword   ptbiNdivs[3];                                     
    CHKSUM  dword   ptbiInMin[3];                                     
    CHKSUM  dword   ptbiInMax[3];                                     
    CHKSUM  dword   ptbiIpType[3];                                    
                                                                      
    CHKSUM  dword   ptbiTabSiz;                                       
    CHKSUM  dword   ptbiOut[NUM_PLC_TBI_DIVS];                        
                                                                      
} PLC_TBI;                                                            
//====================================================================


//====================================================================
// Area Dati protocollo modbus 
//====================================================================

typedef struct {
    dword  RxModbus[16];
    string RxStringModus[16];
    dword  TxModbus[16];
    string TxStringModus[16];
} MODBUSTCP;


typedef struct {
    // CNC --> CAM
    // Cycle Info
    dword   ModeProcess;
              // bit 0: Manual Data Input 
              // bit 1: Automatic
              // bit 2: Step 
              // bit 3: Manual Jog
              // bit 4: Incremental Jog
              // bit 5: Profile return
              // bit 6: Home

    dword   StsProcess;   
              // bit 0: Idle 
              // bit 1: Cycle
              // bit 2: Hold
              // bit 3: Runh
              // bit 4: Hrun
              // bit 5: Error
              // bit 6: Wait
              // bit 7: Reset
              // bit 8: Emergency
              // bit 9: Input  
         
    // Programma Attivo
    string  ActiveProgram[128];
    dword   DurationSec;
    dword   DurationMin;
    dword   DurationHour;

    dword   StartingSec;
    dword   StartingMin;
    dword   StartingHour;	

    dword   ElapsedSec;		
    dword   ElapsedMin;		
    dword   ElapsedHour;		

    // Maintenance Info
    dword   NextService;
    dword   ConsumableAge;		

    // Parameter Snapshot;
    string  MaterialName[16];
    dword   MaterialCode;
    dword   Thickness;
        
    // CNC --> CAM
    // Part Program Load
    string  LoadProgram[16];
		
    // WorkList Load
    string  WorkList[16];
			
    // Cycle Command	
    dword   CtrlMode;
            // bit 0: Manual Data Input 
            // bit 1: Automatic
            // bit 2: Step 
            // bit 3: Manual Jog
            // bit 4: Incremental Jog
            // bit 5: Profile return
            // bit 6: Home
    			  
    dword   CtrlProcess;   
            // bit 0: Avvio Ciclo 
            // bit 1: Hold
            // bit 2: Reset
} IOT;

//*******************************************************************
//  IO TEST
//*******************************************************************
typedef struct {
            word    w_state[30];
    PERSIST word    w_set[30];
    PERSIST word    w_mode[30];
} TS_IO_TEST;


// tube polygon descriptor for generalized machining aid algorythm
typedef struct {
    double      gma_u;
    double      gma_v;
    double      gma_r;

} GMAvertex;

typedef struct {
    dword       gmanVertices;
    GMAvertex   gmaVertices[16];

} GMApolygon;


//*******************************************************************
//  GESTIONE TABELLA GENERICA
//  Per laser tubo: utilizzata per gestire dinamica assi in base
//                  al peso del materiale
//*******************************************************************
typedef struct {
    PERSIST dword   gt_par_in;
    PERSIST dword   gt_par_outA;
    PERSIST dword   gt_par_outB;
    PERSIST dword   gt_par_outC;
    PERSIST dword   gt_par_outD;
    PERSIST dword   gt_par_outE;
    PERSIST dword   gt_par_outF;
} TS_GEN_TABLE;

//*******************************************************************
//  LASER
//  GESTIONE DATI DI TAGLIO CALCOLATI LINEARMENTE TRA UNA
//  LINEA DI TAGLIO E L'ALTRA.
//*******************************************************************
typedef struct {
            dword CDL_Power;
            dword CDL_Duty;
            dword CDL_Freq;
            dword CDL_Focal;
            dword CDL_Pressure;
            dword CDL_CutHeight;
            dword CDL_Feed;
            dword CDL_FeedPerc;
            dword CDL_FirstLine;
            dword CDL_SecondLine;
            dword CDL_SimOnPsw;  //(psw=2021)
            dword CDL_SimFirstLine;
            dword CDL_SimAngle;
            dword CDL_par[10];
    PERSIST dword CDL_rpar[10];
} TS_CUT_DATA_LINEAR;



//#############################################################################

TS_GEN_TABLE        TsGenTable[10];
TS_CUT_DATA_LINEAR  TsCutDataLinear;

GMApolygon gmaPolygons[4];


RETAIN AX_LSREF ax_lsref[NUM_ASSI]; // Axes ref mode

TS_IO_TEST  TsIoTest;

TS_CALIB    TsCalib[8];

MODBUSTCP   ModBusTcp;
IOT         ModBusIoT;

RTCP_CALIB RtcpCalib[2];

FOCAL_PWM       FocalPwm[2];
CAPACITIVE      Capacitive[2];
RESONATOR       Resonator[2];
LSR_GEST        LsrGest[2];

dword ServiceMillePGM;      // 1000.PGM services
word  LaserShootOn;         // Shoot state


UserVrtcHead uvHeads[2];

// VRTC parameter blocks for all channels (see PLC for by-channel mapping)
RETAIN VRTC_PARAM_BLOCK vpbs[11];

// VRTC parameters block selector by channel
dword vpbSelectors[2];

S_FIR ff_fir[2];                        // Filtri FIR da PLC

RETAIN byte WriteCNCPar;  // If we have CNC values overwrite

//====================================================================
//    Variabili riservate IN scrittura scrittura a "PLC" "GUI" "ISO"
//====================================================================
// Riservati scrittura PLC >>> GUI, ISO //
dword gPlc[NUM_APP_gplc];               // Riservati in scrittura per il PLC (GENERICI DI MACCHINA)
RETAIN dword rPlc[NUM_APP_rplc];        // Riservati in scrittura per il PLC (GENERICI DI MACCHINA RETAIN)
dword PlPlc[NUM_APP_plplc];             // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA PLASMA)
dword LsPlc[NUM_APP_lsplc];             // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA LASER)
RETAIN dword rPlPlc[NUM_APP_rplplc];    // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA PLASMA RETAIN)
RETAIN dword rLsPlc[NUM_APP_rlsplc];    // Riservati in scrittura per il PLC (GLOBALI TECNOLOGIA LASER RETAIN)

// Riservati scrittura GUI >>> PLC, ISO //
dword gGui[NUM_APP_ggui];               // Riservati in scrittura a GUI (GENERICI DI MACCHINA)
RETAIN dword rGui[NUM_APP_rgui];        // Riservati in scrittura a GUI (GENERICI DI MACCHINA RETAIN)

dword PlGui[NUM_APP_plgui];             // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA PLASMA)
dword LsGui[NUM_APP_lsgui];             // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA LASER)
RETAIN dword rPlGui[NUM_APP_rplgui];    // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA PLASMA RETAIN)
RETAIN dword rLsGui[NUM_APP_rlsgui];    // Riservati in scrittura a GUI (GLOBALI TECNOLOGIA LASER RETAIN)

// Riservati scrittura ISO >>> PLC, GUI //
dword gIso[NUM_APP_giso];               // Riservati in scrittura per ISO (GENERICI DI MACCHINA)
dword rIso[NUM_APP_riso];               // Riservati in scrittura per ISO (GENERICI DI MACCHINA RETAIN)

dword LsIso[NUM_APP_lsiso];             // Riservati in scrittura per ISO (GLOBALI TECNOLOGIA LASER)
RETAIN dword rLsIso[NUM_APP_rlsiso];    // Riservati in scrittura per ISO (GLOBALI TECNOLOGIA LASER RETAIN)
// ************************************ //

TBLSTR     strtab[NUM_STR_TAB];         // String TABLE di 128 messaggi da 64 char
RETAIN TBLSTR     Rstrtab[NUM_STR_TAB_RTN]; // String TABLE di 128 messaggi da 64 char TAMPONATA

PERSIST BOCCA       Bocca[64];          // Parametri per ciascuna vasca di aspirazione
PERSIST GEN_VASCHE  GenVasche;          // Vasche di aspirazione GENERALI
dword   DigOutAspirator[2];             // Digital out for aspiration management

RETAIN double EBK[NUM_APP_EBK];         // Registri a doppia precisione TAMPONATI (Per VRTC)

PERSIST OFFSET funz[22];                // Registri per gestione origini lavorazioni (21 e 22 x LISTE LAVORO)
PERSIST dword  IndexORG;                // Indice struttura Origini
PERSIST MAT    matPezzo;                // Struttura Dati Matrici Locali Pezzo (Plasma)
PERSIST MAT    matPrg;                  // Struttura Dati Matrici Locali Programma (Plasma)
dword   Kerf;                           // Kerf corrente
dword   TimTagCurr;                     // Tempo di taglio programma corrente

//-------------------- Registri di servizio per ogni canale ---------------
RETAIN dword   numpp[NUM_CANALI];       // numero PP da GUI per PLC da girare su C_NUMPPGM
dword   stato[NUM_CANALI];              // Stato comandato da GUI per il PLC
ORIGINI Or[NUM_CANALI];                 // Origini pezzo per ogni canale

//-------------------------------------------------------------------------

RETAIN word FEED;

word   VER_VERSION;                     // Versione software applicativo
word   REL_VERSION;                     // Release software applicativo
word   MODA_VERSION;                    // Modificatore 1 software applicativo
word   MODB_VERSION;                    // Modificatore 2 software applicativo

dword   SoftBtn[16];                    // Pulsanti a video
dword   VisIcon[16];                    // Stato icone
dword   TstP[16];                       // TestPoints

UNIT_WORK UnitWork;                     // Variabili per G800-G840 (inizio/fine lavorazioni)
byte      FunzRequest;                  // Funzione richiesta per unità (1=StartWork, 2=EndWork)

RETAIN TAB_LSR TabLsr[20];              // Tabelle di lavoro Laser

RETAIN dword DB_Find_Material;
RETAIN dword DB_Find_Tickness;
RETAIN dword DB_Find_CutGas;

RETAIN dword  Lsr_UserInt[15];    		// Dati globali Laser
	                                    // 0 - Indice tabella selezionata	                                    // 1 - Indice tabella attual
RETAIN double Lsr_UserDbl[15];    		// Dati globali Laser

RETAIN dword PlcOp[NUM_PLCOP];          // Opzioni del PLC
RETAIN dword CostK[NUM_COSTK];          // Costanti K
RETAIN dword LSRPlcOp[NUM_LSRPLCOP];    // Laser PLC Options
RETAIN dword LSRCostK[NUM_LSRCOSTK];    // Laser K Constants
RETAIN dword LSRPlcOpUSR[NUM_LSRPLCOP]; // Laser PLC Options USR
RETAIN dword LSRCostKUSR[NUM_LSRCOSTK]; // Laser K Constants USR

RETAIN UNIT_LUN UnitLun[12];             // Configurazione Unità logiche per G806 T""

dword   CompData[10];                   // Dati per gestione canale computazionale (GuiCad)


LSRMECH_MNG     LsrMech;                // Laser (Laser Mech)

//HEAD_TYPE  Head[10];                    // Laser Heads

HEAD_TYPE  Head_Laser[10];                    // Laser Heads

PIPE_CAL  CalPipe;                      //  Calibration pipe cycle

// Gestione LISTE di LAVORO
RETAIN WLIST    Wlist[40];              // Liste di lavoro (Area DATI)
WLIST_MNG       WlistMngt;              // Liste di lavoro (gestione PLC/ISO)

// Gestione PEZZI/GEOMETRIA e contatori tempi di taglio
RETAIN dword   PEZZO;                   // Numero pezzo (parte da 1)
RETAIN dword   GEOMETRIA;               // Numero geometria (parte da 1)

RETAIN dword   TimeActualProg;          // LASER: Tempo di esecuzione programma attuale (n. scansioni PLC lento)
RETAIN dword   TimePrevProg;            // LASER: Tempo di esecuzione programma precedente (n. scansioni PLC lento)

POWDEL_SPLINE   pd_splines[MAX_POWDEL_SPLINES];

RETAIN S_XHC_CONFIG XhcApp;             // Pulsantiera 

CHKSUM dword    RasterParam[20];       // Raster print function variables 

RETAIN OVERRIDE FeedRate[2];

// for tcplink
dword           EDK[NUM_APP_EDK];               // Registri da CN a PLC (E10000)
RETAIN dword    EOK[NUM_APP_EOK];        // Registri da CN a PLC (E80000)
//

PERSIST dword   User_Option[2];         // Available to define  Machine Builder Generic Options
PERSIST dword   Cost_User[32];          // Available to define Machine Builder specific variables
RETAIN  dword   numblck[16];            // numero blocco canale corrente
PERSIST dword   config_machine[100];     // Words used to define Machine type

double dSpeedCurve[10];
double dPowerCurve[10];
word wPointsPowerCurve;
byte byTypeInterpCurve;

EM_CAP EmConfig;

//Modbus Events
dword ModbusTCPEvent;
dword ModbusTCPStat;
dword ModbusTCPCmd;

CHKSUM PLC_TBI ptbiTrans[NUM_PLC_TBI];

//*************** Sezione Applicativo ******************
ENDSECTION Application


// User
#include <defcn.USR>
