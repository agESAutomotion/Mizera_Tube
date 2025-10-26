*******************************************************************************
*   LSRGEST.INC
*   ESAutomotion
*   LASER machine
*******************************************************************************
VAR

***************************************
*   PWM BOARD n.1
***************************************
HW_PWM_DUTY             AT %QW908;          ** Laser Duty Cycle
HW_FREQ                 AT %QW909;          ** Laser frequency 
HW_PWM_RES              AT %QW910;          ** Laser PWM resolution setting
HW_PWM_CFG              AT %QW911;          ** Laser I/F board PWM_CFG register
HW_PWM_CFG_EN           AT %QW911.0;        ** Laser I/F board PWM_CFG register: EN
HW_PWM_CFG_FC           AT %QW911.8;        ** Laser I/F board PWM_CFG register: FC
HW_PWM_CFG_C_OE         AT %QW911.9;        ** Laser I/F board PWM_CFG register: C_OE
HW_PWM_CFG_C_EF         AT %QW911.12;       ** Laser I/F board PWM_CFG register: C_EF
HW_PWM_CFG_C_FR         AT %QW911.13;       ** Laser I/F board PWM_CFG register: C_FR
HW_PWM_POWER            AT %QW913;          ** Laser power


***************************************
*   PWM BOARD n.2
***************************************
HW2_PWM_DUTY            AT %QW918;          ** Laser Duty Cycle
HW2_FREQ                AT %QW919;          ** Laser frequency 
HW2_PWM_RES             AT %QW920;          ** Laser PWM resolution setting
HW2_PWM_CFG             AT %QW921;          ** Laser I/F board PWM_CFG register
HW2_PWM_CFG_EN          AT %QW921.0;        ** Laser I/F board PWM_CFG register: EN
HW2_PWM_CFG_FC          AT %QW921.8;        ** Laser I/F board PWM_CFG register: FC
HW2_PWM_CFG_C_OE        AT %QW921.9;        ** Laser I/F board PWM_CFG register: C_OE
HW2_PWM_CFG_C_EF        AT %QW921.12;       ** Laser I/F board PWM_CFG register: C_EF
HW2_PWM_CFG_C_FR        AT %QW921.13;       ** Laser I/F board PWM_CFG register: C_FR
HW2_PWM_POWER           AT %QW923;          ** Laser power

END_VAR


VAR_IN_OUT

OA_POTENZA_LASER        AT %AnaOut0;            ** Power
OA2_POTENZA_LASER       AT %AnaOut1;            ** Power

LASER_SIM_PAR           AT %ParOptions.0;       ** Laser in simulation
LASER_POWER_ONBUS_PAR   AT %ParOptions.1;       ** Power become from BUS
LASER_POWER_LINEAR_PAR  AT %ParOptions.2;       ** Power linearization
LASER_USE_PWM_POWER     AT %ParOptions.3;       ** Use out PWM power
LASER_MAX_POWER_PAR     AT %ParMaxPower;        ** Max power
LASER_DAC_RES_PAR       AT %ParDACres;          ** DAC resolution
LASER_FILTER_PAR        AT %ParIntSpeedFilt;    ** Filter on interpolation speed

END_VAR
