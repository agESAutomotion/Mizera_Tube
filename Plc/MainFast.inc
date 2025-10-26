*******************************************************************************
*   MAIN FAST.INC
*   ESAutomotion
*   LASER machine
*******************************************************************************

TempCapacitiveSensorPoints:     DWORD;
SogliaFineLamieraPoints:        DWORD;    
MaxVolt_x100:                   DWORD;    
MaxADCPoints:                   DWORD;    

RT_PRGRUN:                      R_TRIG;             * Trigger PROGRUN
temp_bevAnglecalc:  DWORD;
fir_time:           DWORD;
error1:             DWORD;
appoggio:           BOOL;
appoggio1:          BOOL;
appoggio2:          BOOL;
appoggio3:          BOOL;
V_Capacitivo:       DWORD;      * Lettura tensione capacitivo (in centesimi di V) 
StandOff:           DWORD;      * Valore distanza taglio in centesimi
TON_TIPTOUCH:       TON;        * Tip touch ritardato di 500 msec

* Hold retrace memories
dist1HoldPush:      DWORD;      * Distance of the interpolator when Stop button is pushed
dist2Hold:          DWORD;      * Distance of the interpolator when axes are not moving any more
RTRIG_STOP:         R_TRIG;  
DeltaDist:          DWORD;      * Delta distance for hold
TON_RETRCOM:        TON;
ch0stopusr:         BOOL;       * Channel in stop from user request
FTRIG_RetrDone:     F_TRIG;
appoggio4:          BOOL;
*

TmpHeightCheck:     DWORD;      * threshold for capacitive sensor


*** SIMULATION
cont:               DWORD;
cont2               AT %R46;
step                AT %R47;
dist:               DWORD;
RTRIG_PRG:          R_TRIG;        
encoderRegister:    DWORD;
prevEncoderRegister: DWORD;
PosEnc:             DWORD;    
inc:                DWORD; 
encoderPosition:    DWORD;   
position            AT  %rLsPlc77; 
AxisPos:            DWORD;
RTRIG_OFFSET:       R_TRIG;
*** 

d:                 DWORD;    * external encoder pulses
m:                 DWORD;    * mm for 1 external encoder revolution

* Wrong Working Counter

synchTime           AT %ecat_ring.can_msync;      * Measured cycle time(min,max)

ActualRxError       AT %ecat_ring.CanRingDiag1;  * Actual Receive error value        
ActualWCounter      AT %ecat_ring.CanRingDiag2;  * Actual Working Counter value
ActualDCerror       AT %ecat_ring.CanRingDiag6;  * Actual DC errors
EcatRun             AT %ecat_ring.ec_status.31;  * Actual Working Counter value

TP_WCOUNT:                 TP;
TON_WCERROR:              TON;
TON_ECAT_OK:              TON;
R_TRIG_WC:             R_TRIG;
WCerror:                 BOOL;         * working counter error
ErrorWCounter:           BOOL;         * ethercat working counter is increasing
ErrorReceived:           BOOL;         * Error recieved on ethercat ring
ErrorDCoscillat:         BOOL;         * Error DC is oscillating too much
firstRXError:           DWORD;         * Initial RX error count
firstWCounterVal:       DWORD;         * Wrong counter after statup of the ring
firstDCerrVal:          DWORD;         * DC error first value
timeforAlarm:           DWORD;         * time for alarm
synchTimeEthercat:      DWORD;         * synch time for ethercat

MeasuredCycleMin:       DWORD;         * min ethercat synch time
MeasuredCycleMax:       DWORD;         * max ethercat synch time
DeltaSynch:             DWORD;         * Delta Ethercat Synch
*

RTRIG_WELDING_COUNT:    R_TRIG;
RTRIG_INIT:             R_TRIG;
loadconfig:              DWORD;
calcBeam:                DWORD;  
tcp_z:                   DWORD;
TP_TIME:                 TP;
InitTCP:                 BOOL;

