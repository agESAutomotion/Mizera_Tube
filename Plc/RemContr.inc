**********************************************************************
* RemContr.inc
* Remote controller input button
**********************************************************************

RemContrAuto               AT %XhcApp.Xhc_Puls.0;          * Xhc_Puls.0  Sel auto mode      *   
RemContrJog                AT %XhcApp.Xhc_Puls.1;          * Xhc_Puls.1  Sel jog mode       *
RemContrRef                AT %XhcApp.Xhc_Puls.2;          * Xhc_Puls.2  Sel ref mode       *
RemContrStart              AT %XhcApp.Xhc_Puls.3;          * Xhc_Puls.3  Start push-button  *
RemContrStop               AT %XhcApp.Xhc_Puls.4;          * Xhc_Puls.4  Stop push button   *
RemContrReset              AT %XhcApp.Xhc_Puls.5;          * Xhc_Puls.5  Reset Push Button  *
RemContrSheetCheck         AT %XhcApp.Xhc_Puls.6;          * Xhc_Puls.6  Sheet dimension control *
RemContrBreakPoint         AT %XhcApp.Xhc_Puls.7;          * Xhc_Puls.7  Break point positioning request*
RemContrDryRun             AT %XhcApp.Xhc_Puls.8;          * Xhc_Puls.8  Select Dry Run          *
RemContrFeedPlus           AT %XhcApp.Xhc_Puls.9;          * Xhc_Puls.9  Feed plus               *
RemContrBackOrig           AT %XhcApp.Xhc_Puls.10;         * Xhc_Puls.10 Start axes to origin    *
RemContrManAPA             AT %XhcApp.Xhc_Puls.11;         * Xhc_Puls.11 Manual Apa              *
RemContrCalib              AT %XhcApp.Xhc_Puls.12;         * Xhc_Puls.12 Execute calibration     *
RemContrFeedMinus          AT %XhcApp.Xhc_Puls.13;         * Xhc_Puls.13 Feed minus              *
RemContrOxyON              AT %XhcApp.Xhc_Puls.14;         * Xhc_Puls.14 Oxy On
RemContrAirOn              AT %XhcApp.Xhc_Puls.15;         * Xhc_Puls.15 Air On
RemContrN2On               AT %XhcApp.Xhc_Puls.16;         * Xhc_Puls.16 N2 on       
RemContrLaserOn            AT %XhcApp.Xhc_Puls.17;         * Xhc_Puls.17 Laser shut           
RemContrCMinus             AT %XhcApp.Xhc_Puls.18;         * Xhc_Puls.18 Jog C axis minus     
RemContrYPlus              AT %XhcApp.Xhc_Puls.19;         * Xhc_Puls.19 Jog U axis minus         *     
RemContrZPlus              AT %XhcApp.Xhc_Puls.20;         * Xhc_Puls.20 Jog Z axis plus         * 
RemContrGuideLas           AT %XhcApp.Xhc_Puls.21;         * Xhc_Puls.21 Enable guide laser      * 
RemContrUPlus              AT %XhcApp.Xhc_Puls.22;         * Xhc_Puls.22 Jog Y axis plus
RemContrJogFast            AT %XhcApp.Xhc_Puls.23;         * Xhc_Puls.23 Jog Fast                *
RemContrUMinus             AT %XhcApp.Xhc_Puls.24;         * Xhc_Puls.24 Jog Y axis minus
RemContrShoot              AT %XhcApp.Xhc_Puls.25;         * Xhc_Puls.25 Enable laser resonator           
RemContrCPlus              AT %XhcApp.Xhc_Puls.26;         * Xhc_Puls.26 Jog C axis plus   
RemContrYMinus             AT %XhcApp.Xhc_Puls.27;         * Xhc_Puls.27 Jog U axis plus         *
RemContrZMinus             AT %XhcApp.Xhc_Puls.28;         * Xhc_Puls.28 Jog Z axis minus        *
*RemContrSetP               AT %XhcApp.Xhc_Puls.29;         * Xhc_Puls.29 Sel INC mode            *
Rem_Combination            AT %XhcApp.Xhc_Puls.29;         * Xhc_Puls.29 Sel INC mode            *
RemContrFoc0               AT %XhcApp.Xhc_Puls.30;         * Xhc_Puls.30 Move focal at 0 mm      *
RemContrCancAlarm          AT %XhcApp.Xhc_Puls.31;         * Xhc_Puls.31 Clear alarm             *

RemContrXPlus              AT %PLCFLAGS.0;         * Xhc_Puls.24 Jog X axis plus         *
RemContrXMinus             AT %PLCFLAGS.0;         * Xhc_Puls.22 Jog X axis minus        *

*// Screen Lines
LineFeed                   AT %XhcApp.Xhc_LineIdx0;        * XhcApp.Xhc_LineIdx0 Z axis quote      
LineXaxis                  AT %XhcApp.Xhc_LineIdx1;        * XhcApp.Xhc_LineIdx1 X axis quote      
LineYaxis                  AT %XhcApp.Xhc_LineIdx2;        * XhcApp.Xhc_LineIdx2 Y axis quote      
LineZaxis                  AT %XhcApp.Xhc_LineIdx3;        * XhcApp.Xhc_LineIdx3 Feedrate and mode 