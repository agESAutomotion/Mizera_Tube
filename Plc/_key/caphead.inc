I_I_PREC_POSITION_REACHED   AT %fDigIn.0;       * Position Reached 
I_I_PREC_ERROR              AT %fDigIn.1;       * precitec in error
O_O_PREC_OUTPSELEC          AT %fDigOut.0;      * Outp precitec for multi function selection
O_O_PREC_AUTOMATIC          AT %fDigOut.1;      * Automatic request fo precitec head
O_O_PREC_REFERENCE          AT %fDigOut.2;      * Reference signal for precitec head
I_FAR_A10                   AT %cDigIn0.0;      * Far away
I_TIP_BODY_TOUCH_A11        AT %cDigIn0.1;      * Tip\Body Touch or Nozzle_Lost (TIP TOUCH:A11=1,A14=0,A15=0)      
I_CABLE_CUT_A12             AT %cDigIn0.2;      * Cable cut
I_READY_A13                 AT %cDigIn0.3;      * Ready                                                            
I_BODY_TOUCH_A14            AT %cDigIn0.4;      * Body touch (A11=1,A14=1,A15=0)                                   
I_NOZZLE_LOST_A15           AT %cDigIn0.5;      * Nozzle Lost/Pos reached (NOZZLE LOST:A11=1,A14=0,A15=1)
O_CALIBRA_REQ_A3            AT %cDigOut0.0;     * Calibration request 
O_STROBE_A7                 AT %cDigOut0.1;     * Strobe      