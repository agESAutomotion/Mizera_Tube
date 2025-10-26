
VAR_IN_OUT

I_I_PREC_POSITION_REACHED   AT %fDigIn.0;       * Position Reached 
I_I_PREC_ERROR              AT %fDigIn.1;       * precitec in error
I_I_PREC_SW1                AT %PLCFLAGS.1;
I_I_PREC_SW4                AT %PLCFLAGS.1;
	   
O_O_PREC_OUTPSELEC          AT %fDigOut.0;      * Outp precitec for multi function selection
O_O_PREC_AUTOMATIC          AT %fDigOut.1;      * Automatic request fo precitec head
O_O_PREC_REFERENCE          AT %fDigOut.2;      * Reference signal for precitec head            
                            
OA_FOCALE                   AT %fAnaOut0; * Focal analog out

ForceRefGui                 AT %fButton.0;
FocType                     AT %fPar0;

MinFocal                    AT %pwPar2;         * 
MaxFocal                    AT %pwPar3;
aValueFocal                 AT %pwPar4;
bValueFocal                 AT %pwPar5;

CntrlVoltage                AT %LsPlc2;         *(LsPlc2) A_GUI Volt Focale

END_VAR