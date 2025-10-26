*******************************************************************************
*   RE_IPG.INC
*   ESAutomotion
*   LASER machine
*******************************************************************************
VAR_IN_OUT

*** I/O

I_B1_LASER_READY                AT %DigIn0.0;   * IPG Laser internal main power supply ON (Aux Power ON)
I_B13_AVVERTENZA                AT %DigIn0.1;   * IPG WARNING OUT
I_B4_LASER_ERROR                AT %DigIn0.2;   * IPG YLS YLR Laser in Alarm status
I_B5_LASER_GUIDE                AT %DigIn0.3;   * IPG Laser guida ON
I_B2_EMISSION_ON                AT %DigIn0.4;   * IPG Emission ON
I_B7_LASER_ASSIGNED             AT %DigIn0.5;   * IPG Laser ASSIGNED
I_B8_POWER_SUPPLY               AT %DigIn0.6;   * IPG power supply status
I_B3_INT_CONTROL                AT %DigIn0.7;   * IPG Controllo interno abilitato
I_B6_ANALOG_CONTROL             AT %DigIn0.8;   * IPG Controllo analogico ON

O_C1_LASER_SWITCH               AT %DigOut0.0;  * IPG Laser Switch ON
O_A4_LASER_RESET                AT %DigOut0.1;  * IPG Resetta errori
O_A1_LASER_REQ                  AT %DigOut0.2;  * IPG Richiesta Laser
O_A5_GUIDE_LASER                AT %DigOut0.3;  * IPG Laser spot guide ON
O_A3_INT_CONTROL                AT %DigOut0.4;  * IPG Abilitazione Controllo interno
O_A6_ANALOG_CONTROL             AT %DigOut0.5;  * IPG Controllo analogico ON
O_SAFETY_CIRCUIT_RESET          AT %DigOut0.6;  * IPG (reset dopo safety)
O_A2_LASER_PROG_RUN             AT %DigOut0.7;  * IPG (START PROGRAM)


*** BUTTONS

BTN_LASER_REQUEST               AT %Button0.0;  ** BTN: laser request ON
BTN_POWER_ON                    AT %Button0.1;  ** BTN: Power ON
BTN_GUIDE_ON                    AT %Button0.2;  ** BTN: Guide ON
BTN_ANALOG_ON                   AT %Button0.3;  ** BTN: Analog control ON
BTN_INT_ON                      AT %Button0.4;  ** BTN: Interal control ON
BTN_RESET                       AT %Button0.5;  ** BTN: reset


END_VAR
