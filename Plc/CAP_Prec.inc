*******************************************************************************
*   CAP_PREC.INC
*   ESAutomotion
*   LASER machine
*******************************************************************************

VAR_IN_OUT

I_FAR_A10                   AT %cDigIn0.0;      * Far away
I_TIP_BODY_TOUCH_A11        AT %cDigIn0.1;      * Tip\Body Touch or Nozzle_Lost (TIP TOUCH:A11=1,A14=0,A15=0)      
I_CABLE_CUT_A12             AT %cDigIn0.2;      * Cable cut
I_READY_A13                 AT %cDigIn0.3;      * Ready                                                            
I_BODY_TOUCH_A14            AT %cDigIn0.4;      * Body touch (A11=1,A14=1,A15=0)                                   
I_NOZZLE_LOST_A15           AT %cDigIn0.5;      * Nozzle Lost/Pos reached (NOZZLE LOST:A11=1,A14=0,A15=1)

O_CALIBRA_REQ_A3            AT %cDigOut0.0;     * Calibration request 
O_STROBE_A7                 AT %cDigOut0.1;     * Strobe
O_SELCHAR_BIT0_A4           AT %cDigOut0.2;     * Selecting the calibrated characteristic curve (1-8)
O_SELCHAR_BIT1_A5           AT %cDigOut0.3;     * Selecting the calibrated characteristic curve (1-8)
O_SELCHAR_BIT2_A6           AT %cDigOut0.4;     * Selecting the calibrated characteristic curve (1-8)

END_VAR
