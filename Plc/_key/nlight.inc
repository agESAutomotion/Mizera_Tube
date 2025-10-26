I_FW_READY_nLight               AT %Resonator1.DigIn.0;      * NLIGHT: nLight ready to recieve input       
I_EMISS_nLight                  AT %Resonator1.DigIn.1;      * NLIGHT: emiss nlight enable
I_ERR_nLight                    AT %Resonator1.DigIn.2;      * NLIGHT: nlight error
I_RDY_nLight                    AT %Resonator1.DigIn.3;      * NLIGHT: laser nlight ready                                
I_ExtCtr_Ready_nLight           AT %Resonator1.DigIn.4;      * NLIGHT: nLight external control ready       
O_EXTEN_IN_nLight               AT %Resonator1.DigOut.0;     * NLIGHT: nlight exten in 
O_GUIDE_nLight                  AT %Resonator1.DigOut.1;     * NLIGHT: Guide laser nlight 
O_SYSTEM_ON_nLight              AT %Resonator1.DigOut.2;     * NLIGHT: nlight system on
O_RESET_nLight                  AT %Resonator1.DigOut.3;     * NLIGHT: reset error nlight
O_ExtCtr_ENABLE_nLight          AT %Resonator1.DigOut.4;     * NLIGHT: Enable external control of nlight
IA_BEAM_DUMP                    AT %Resonator1.rAnaIn0;      * NLIGHT: Beam dump monitor
IA_AUXILIARY                    AT %Resonator1.rAnaIn1;      * NLIGHT: Auxiliary monitor
IA_BACK_REFL                    AT %Resonator1.rAnaIn2;      * NLIGHT: Back reflection monitor
IA_OPT_POWER                    AT %Resonator1.rAnaIn3;      * NLIGHT: Optical power monitor
Pul_Guide_Laser                 AT %Resonator1.Button.0;
Pul_Sys_On                      AT %Resonator1.Button.1;
Pul_Error_Reset                 AT %Resonator1.Button.2;
guiSystemOn                     AT %Resonator1.Leds.0;       * nLight System On
guiExTenIn                      AT %Resonator1.Leds.1;       * nLight ExTenIn
guiLaserOn                      AT %Resonator1.Leds.2;       * nLight Laser On
guiReady                        AT %Resonator1.Leds.3;       * nLight Laser ready
guiErr                          AT %Resonator1.Leds.4;       * nLight Error
guiEmiss                        AT %Resonator1.Leds.5;       * nLight Emiss
guiResetError                   AT %Resonator1.Leds.6;       * nLight reset error
guiGuideLaser                   AT %Resonator1.Leds.7;       * nLight guide laser