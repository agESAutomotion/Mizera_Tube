VAR_IN_OUT

****************************************************************************************************
* NLIGHT SOURCE I/O
****************************************************************************************************
I_I_FW_READY_nLight               AT %DigIn.0;      * NLIGHT: nLight ready to recieve input       
I_I_EMISS_nLight                  AT %DigIn.1;      * NLIGHT: emiss nlight enable
I_I_ERR_nLight                    AT %DigIn.2;      * NLIGHT: nlight error
I_I_RDY_nLight                    AT %DigIn.3;      * NLIGHT: laser nlight ready                                
I_I_ExtCtr_Ready_nLight           AT %DigIn.4;      * NLIGHT: nLight external control ready       

O_O_EXTEN_IN_nLight               AT %DigOut.0;     * NLIGHT: nlight exten in 
O_O_GUIDE_nLight                  AT %DigOut.1;     * NLIGHT: Guide laser nlight 
O_O_SYSTEM_ON_nLight              AT %DigOut.2;     * NLIGHT: nlight system on
O_O_RESET_nLight                  AT %DigOut.3;     * NLIGHT: reset error nlight
O_O_ExtCtr_ENABLE_nLight          AT %DigOut.4;     * NLIGHT: Enable external control of nlight

IA_BEAM_DUMP                    AT %rAnaIn0;      * NLIGHT: Beam dump monitor
IA_AUXILIARY                    AT %rAnaIn1;      * NLIGHT: Auxiliary monitor
IA_BACK_REFL                    AT %rAnaIn2;      * NLIGHT: Back reflection monitor
IA_OPT_POWER                    AT %rAnaIn3;      * NLIGHT: Optical power monitor

Pul_Guide_Laser                 AT %Button.0;
Pul_Sys_On                      AT %Button.1;
Pul_Error_Reset                 AT %Button.2;

guiSystemOn                     AT %Leds.0;       * nLight System On
guiExTenIn                      AT %Leds.1;       * nLight ExTenIn
guiLaserOn                      AT %Leds.2;       * nLight Laser On
guiReady                        AT %Leds.3;       * nLight Laser ready
guiErr                          AT %Leds.4;       * nLight Error
guiEmiss                        AT %Leds.5;       * nLight Emiss
guiResetError                   AT %Leds.6;       * nLight reset error
guiGuideLaser                   AT %Leds.7;       * nLight guide laser

END_VAR