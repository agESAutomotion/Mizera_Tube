I_I_PUL_START               AT %M51.0;          * Start
I_I_PUL_STOP                AT %M51.1;          * Stop
I_I_PUL_RESET               AT %M51.2;          * Reset
I_I_HW_START                AT %M51.3;          * Start (handwheel)
I_I_HW_STOP                 AT %M51.4;          * Stop (handwheel)
I_I_HW_RESET                AT %M51.5;          * Reset (handwheel)
I_I_HW_JOGPLUS              AT %M51.6;          * Jog+ (handwheel)
I_I_HW_JOGMINUS             AT %M51.7;          * jog- (handwheel)
I_I_JOY_X_P                 AT %M51.8;          * X+ (Joystick)
I_I_JOY_Y_P                 AT %M51.9;          * Y+ (Joystick)
I_I_JOY_X_M                 AT %M51.10;         * X- (Joystick)
I_I_JOY_Y_M                 AT %M51.11;         * Y- (Joystick)
I_I_PUL_TORCH_UP            AT %M51.12;         * Jog+ pushbutton (Torch Up)
I_I_PUL_TORCH_DW            AT %M51.13;         * Jog- pushbutton (Torch Down)
I_I_SEL_AUT                 AT %M51.18;         * Sel AUT state
I_I_SEL_JOG                 AT %M51.19;         * Sel JOG state
I_I_SEL_INC                 AT %M51.20;         * Sel INC state
I_I_SEL_MDI                 AT %M51.21;         * Sel MDI state
I_I_DRIVES_OK               AT %M52.0;          * All drives are ok
I_I_READY_DRIVE_X           AT %M52.1;          * Ready signal drive X
I_I_READY_DRIVE_Y           AT %M52.2;          * Ready signal drive Y
I_I_READY_DRIVE_Z           AT %M52.3;          * Ready signal drive Z
I_I_READY_DRIVE_UV          AT %M52.4;          * Ready signal drive U/V
I_I_READY_DRIVE_W           AT %M52.5;          * Ready signal drive W (external focal)
I_I_READY_DRIVE_C           AT %M52.6;          * Ready signal drive C
I_I_READY_DRIVE_C1          AT %M52.7;          * Ready signal drive C1
I_I_READY_DRIVE_A           AT %M52.8;          * Ready signal drive A
I_I_READY_DRIVE_B           AT %M52.9;          * Ready signal drive B
I_I_READY_DRIVE_Z_FOLL      AT %M52.10;         * Ready signal drive Z_follower
I_I_PRESSIONE_ARIA_IMPIANTO AT %M55.0;          * Pressione aria Impianto OK
I_I_PRESSIONE_N2            AT %M55.1;          * Pressione Azoto OK
I_I_PRESSIONE_O2            AT %M55.2;          * Pressione Ossigeno OK
I_I_PRESSIONE_ARIA          AT %M55.3;          * Pressione Aria di taglio OK
I_I_PRESSIONE_ARIA_BANCO    AT %M55.4;          * Pressione aria Banco OK
I_I_HOERBIGER_READY         AT %M55.8;          * Hoerbiger ready on bus ecat
O_O_LUCE_CABINA             AT %M60.0;          * machine light
O_O_LUCE_SU_TESTA_LASER     AT %M60.1;          * head light
O_O_LAMPADA_ROSSA           AT %M60.2;          * Red lamp
O_O_LAMPADA_ARANCIONE       AT %M60.3;          * Orange lamp
O_O_LAMPADA_VERDE           AT %M60.4;          * Green lamp
O_O_CNC_OK                  AT %M60.5;          * CNC OK
O_O_BUZZER_START_CIC        AT %M60.6;          * Star cycle buzzer
O_O_LUB                     AT %M60.7;          * Lubrication
O_O_WELDING_PISTON          AT %M60.8;          * Piston to enable Sensor for welding search
O_O_EV_ARIA_UGELLO          AT %M60.15;         * Elettrovalvola aria ugello
O_O_EV_ASSISTENZA_TAGLIO    AT %M60.16;         * Elettrovalvola assistenza taglio
O_O_EV_OPEN_N2              AT %M60.17;         * Elettrovalvola apertura Azoto
O_O_EV_OPEN_O2              AT %M60.18;         * Elettrovalvola apertura Ossigeno
O_O_EV_OPEN_ARIA            AT %M60.19;         * Elettrovalvola apertura Aria taglio
O_O_EV_AUX_GAS              AT %M60.20;         * Elettrovalvola ausiliaria (a valle della proporzionale)
O_O_ENAB_VALVE_HOERB        AT %M60.21;         * Enable proportional valve for oxygen (Hoerbiger) 
O_O_PILOT_HOERB             AT %M60.22;         * Hoerbiger: gas command ev
O_O_CALIB_REQ_HOERB         AT %M60.23;         * Hoerbiger: calibration request
O_O_EV_OPEN_N2_HOERB        AT %M60.24;         * Hoerbiger: N2 ev
O_O_EV_OPEN_O2_HOERB        AT %M60.25;         * Hoerbiger: O2 ev
O_O_EV_OPEN_ARIA_HOERB      AT %M60.26;         * Hoerbiger: air ev
O_O_GAS_ECAT_TYPE0_ENAB     AT %M60.27;         * Enable proportional valve on bus ecat