
VAR_IN_OUT 

* Cutbox to PLC
**
Ec_CutBoxST_ONE                  AT %EcMDPIn0;      ** CutBox status bits (1)
Ec_CutBoxST_TWO                  AT %EcMDPIn1;      ** CutBox status bits (2)
Ec_CutHeadST_ONE                 AT %EcMDPIn2;      ** Cutting head status bits (1)
Ec_CutHeadST_TWO                 AT %EcMDPIn3;      ** Cutting head status bits (2)
Ec_CutHeadSerST_ONE              AT %EcMDPIn4;      ** Cutting head servo controller status bits (1)
Ec_CutHeadSerST_TWO              AT %EcMDPIn5;      ** Cutting head servo controller status bits (2)

* ProCutter 2 settings

Ec_ProCutterType                AT %EcMDPIn10;      ** 1: ProCutter 2.0 -- 2: ProCutter 2.0 Zoom
Ec_StraylSensoProtWindGain      AT %EcMDPIn11;      ** 1: Low -- 2: Medium -- 3: High -- Read gain factor for protective window process stray light (top and bottom)
Ec_ActualImageRatio             AT %EcMDPIn20;      ** PC zoom only: current image ratio. For the assignment of variable value to image ratio
Ec_CurrentFocalPosition         AT %EcMDPIn51;      ** Current focal position [10μm]
Ec_CustomerFocusOffset          AT %EcMDPIn52;      ** Read customer offset [10 μm]

** Distance functions

Ec_WorkingDistance              AT %EcMDPIn18;      ** Distance z [1 μm]
Ec_NextCalibPosition            AT %EcMDPIn19;      ** Next reference point position in 16-point calibration [μm]
Ec_PIDSpeedSetpoint             AT %EcMDPIn53;      ** Speed setpoint value of the PID sensor controller, value range from min to max [-27648 to 27648]

** Sensor values	

Ec_TempProtectiveWindow             AT %EcMDPIn21;  ** Process protective window temperature [0.01 °C]
Ec_TemperatureFocusingLens          AT %EcMDPIn22;  ** Focusing lens temperature [0.01 °C]
Ec_TemperatureCollimatingLens       AT %EcMDPIn23;  ** Collimating lens temperature [0.01 °C]
Ec_TemperatureCuttingHead           AT %EcMDPIn24;  ** Head temperature [0.01 °C]
Ec_TemperatureDistanceSensor        AT %EcMDPIn25;  ** Temperature output distance sensor [0.01 °C] (0 - 100°C)
Ec_StraylightBeamPath               AT %EcMDPIn26;  ** Beam path stray light [0.01 %]
Ec_StraylightProtectWindowProc      AT %EcMDPIn27;  ** Stray light, protective window [0.01 %]
Ec_StraylightProtectWindowCollima   AT %EcMDPIn28;  ** Stray light, protective window COL [0.01 %]
Ec_PresLeakProtectWindowProcess     AT %EcMDPIn29;  ** Process protective window pressure [10 mbar]
Ec_ActualCuttingGasPres             AT %EcMDPIn30;  ** Cutting gas pressure [10 mbar]
Ec_HumidityInCuttingHead            AT %EcMDPIn31;  ** Humidity inside head [0.01%]
Ec_Plasma                           AT %EcMDPIn32;  ** PlasmaTec value (0 … 1023)
**

* Plc to Cutbox              
**

Ec_CutBoxControl_ONE                AT %EcMDPOut0;  ** CutBox control bits 1
Ec_CutBoxControl_TWO                AT %EcMDPOut1;  ** CutBox control bits 2
Ec_CuttHeadContr_ONE                AT %EcMDPOut2;  ** Cutting head control bits 1
Ec_CuttHeadContr_TWO		        AT %EcMDPOut3;  ** Cutting head control bits 2


** ProCutter 2.0 control settings 
 
Ec_StraylightGainFactor             AT %EcMDPOut7;  ** 1: too low -- 2: medium -- 3: high - Default value: 1 -- Gain factor for writing the two protective window stray light sensors
Ec_MaxStraylightColLens             AT %EcMDPOut20; ** Max. stray light, protective window collimating lens [0.01%] (top) Default value: 80%
Ec_MaxStraylightProtWindowProcess   AT %EcMDPOut21; ** Max. stray light, process protectiv window [0.01%] (bottom) Default value: 80%
Ec_MaxHumidityCuttingHead           AT %EcMDPOut22; ** Max. humidity [0.01%] Default value: 100%
Ec_MaxTempProtWindowProcess         AT %EcMDPOut23; ** Max. protective window temperature [0.01°C] Default value: 90°C
Ec_MaxTempCollimatingLens           AT %EcMDPOut24; ** Max. collimating lens temperature [0.01°C] Default value: 100°C
* Ec_MaxTempFocalLens               AT %EcMDPOut25; ** Max. focusing lens temperature [0.01°C] Default value: 80°C
Ec_MaxTempCuttingHead               AT %EcMDPOut26; ** Max. laser head temperature [0.01 °C] Default value: 55°C
Ec_MaxPressureAssistGas             AT %EcMDPOut27; ** Max. cutting gas pressure [0,01 bar] Default value: 25Bar
Ec_MaxPresLeakProtWProc             AT %EcMDPOut28; ** Max. protective window pressure loss [0.01bar] Default value: 3000mBar
Ec_MaxStrayLCutHead                 AT %EcMDPOut29; ** Max. stray light [0.01 %]
Ec_ImageRatio                       AT %EcMDPOut32; ** Adjust the image ratio (can only be used with zoom heads).
Ec_FocalPosition                    AT %EcMDPOut48; ** Focal position setpoint [10μm] Note: The focal position setpoint will only be accepted if 1027_CB_Rd_FocusNewSetpoint is enabled. Default value: 0.00mm

** Distance function control settings
 
Ec_SelectCharacteristicCurve        AT %EcMDPOut6;  ** Characteristic curve selection Default value: 0
Ec_DistanceFilterWindowSize         AT %EcMDPOut8;  ** Mean value time window (1 - 255) Default value: 6
Ec_P_Parameter                      AT %EcMDPOut9;  ** PID controller P parameter (1 - 255) Default value: 150
Ec_I_Parameter                      AT %EcMDPOut10; ** PID controller I parameter (1 - 255) Default-value: 0
Ec_D_Parameter                      AT %EcMDPOut11; ** PID controller D parameter (1 - 255) Default value: 0
Ec_DistanceStandoffValue            AT %EcMDPOut30; ** Sensor control: working distance between nozzle and workpiece [1 μm] (0.03 - 0.97 x measuring range limit)
Ec_SpatterfilterTime                AT %EcMDPOut31; ** Sensor control: Spatter compensation time [ms] (0 - 1020) Default value: 20 ms
Ec_TipTouchLevel                    AT %EcMDPOut33; ** TipTouch [μm] (0-1000μm) Default value: 100 μm
Ec_MeasuringRangeLimit              AT %EcMDPOut34; ** Measuring range end value [μm] (2000-40000 μm) Default value: 30,000 mm
Ec_CollisionDelay                   AT %EcMDPOut35; ** Collision delay [ms] to suppress ‘tip touch’ and ‘collision’ (0 - 1020 ms) Default value: 20 ms
Ec_SetPointBand                     AT %EcMDPOut36; ** Setpoint range of the sensor controller for the signal ‘position reached’ [μm] (100 - 5000)
Ec_ControlDynamicBand               AT %EcMDPOut37; ** Control voltage dynamics [mV/ms] (100 - 9999) to limit linear drive acceleration
Ec_MaxTemperatureSensorUnit         AT %EcMDPOut38; ** Power Characteristic Curve Required for old firmware versions of specific cutting heads

**


* Cutbox to PLC

PLC_CutBoxST_ONE                    AT %CutBoxST_ONE;       ** CutBox status bits (1)
PLC_CutBoxST_TWO                    AT %CutBoxST_TWO;       ** CutBox status bits (2)
PLC_CutHeadST_ONE                   AT %CutHeadST_ONE;      ** Cutting head status bits (1)
PLC_CutHeadST_TWO                   AT %CutHeadST_TWO;      ** Cutting head status bits (2)
PLC_CutHeadSerST_ONE                AT %CutHeadSerST_ONE;   ** Cutting head servo controller status bits (1)
PLC_CutHeadSerST_TWO                AT %CutHeadSerST_TWO;   ** Cutting head servo controller status bits (2)
		 
DistanceControlReady                AT %CutBoxST_ONE.0;     ** Status CutBox distance system (1 = ready for operation 0 = not ready for operation)
DistancePositionReached             AT %CutBoxST_ONE.1;     ** Setpoint position within the setpoint range (Zn)
AlarmDistContrIntError              AT %CutBoxST_ONE.2;     ** Collective fault
FarDistance                         AT %CutBoxST_ONE.3;     ** Distance sensor outside the measurement range
AlarmTipTouch                       AT %CutBoxST_ONE.4;     ** Nozzle touches the workpiece
AlarmBodyTouch                      AT %CutBoxST_ONE.5;     ** Short circuit between sensor and workpiece
AlarmCableCut                       AT %CutBoxST_ONE.6;     ** Distance sensor cable disconnected
AlarmVoltageSupply                  AT %CutBoxST_ONE.7;     ** 

AlarmTempDistSensorUni              AT %CutBoxST_TWO.0;     ** Temperature of the sensor body > 55° C
AlarmNozzleLost                     AT %CutBoxST_TWO.1;     ** Cutting head nozzle missing
FieldbusSynced                      AT %CutBoxST_TWO.2;     ** If fieldbus is synchronized = 1 (true)
    *                                   AT %CutBoxST_TWO.3;     ** Free
AlarmDistCalibGainError             AT %CutBoxST_TWO.4;     ** Sensor out of the possible measuring range
AlarmDistCalibDeltaError            AT %CutBoxST_TWO.5;     ** Faulty supporting point position during calibration
AlarmCalibrationCanceled            AT %CutBoxST_TWO.6;     ** ???
    *                                   AT %CutBoxST_TWO.6;     ** Free
*

SensorizedCuttingHeadDetected       AT %CutHeadST_ONE.0;    ** Laser head with sensor detected
AlarmStrayLProteWColLens            AT %CutHeadST_ONE.1;    ** Protective window COL: stray light limit value exceeded
AlarmGlobalErrorCuttingHead         AT %CutHeadST_ONE.2;    ** Collective error = cutting head error
AlarmMissingCartridgeProcess        AT %CutHeadST_ONE.3;    ** Protective window cartridge missing
AlarmMissProtWindowProc             AT %CutHeadST_ONE.4;    ** Protective window missing
AlarmTempProteWindowProc            AT %CutHeadST_ONE.5;    ** Protective window temperature exceeded
AlarmTempCollimatingLens            AT %CutHeadST_ONE.6;    ** Collimating lens: temperature exceeded
AlarmTempFocusingLens               AT %CutHeadST_ONE.7;    ** Focusing lens: temperature exceeded

AlarmTempCuttingHead                AT %CutHeadST_TWO.0;    ** Cutting head: temperature exceeded
AlarmStrayLBeamPath                 AT %CutHeadST_TWO.1;    ** Stray light: level exceeded
AlarmAssistGasPressTooHigh          AT %CutHeadST_TWO.2;    ** Cutting gas: pressure exceeded
AlarmPresLeakageProtWProc           AT %CutHeadST_TWO.3;    ** Protective window cartridge, pressure loss
AlarmCutHeadError                   AT %CutHeadST_TWO.4;    ** System errors
AlarmMissingCartrColLens            AT %CutHeadST_TWO.5;    ** Protective window cartridge COL missing
AlarmHumidityCuttingHead            AT %CutHeadST_TWO.6;    ** Humidity: level exceeded
AlarmStrayLProtWindowProc           AT %CutHeadST_TWO.7;    ** Protective window: stray light limit exceeded


DriveDetected                       AT %CutHeadSerST_ONE.0; ** Servo detected
AlarmCuttHeadComLost                AT %CutHeadSerST_ONE.1; ** Communication between cutting head and CutBox disturbed
AlarmDriveError                     AT %CutHeadSerST_ONE.2; ** Collective error = drive error
AlarmEdgeTec                        AT %CutHeadSerST_ONE.3; ** ???????
DriveBusy                           AT %CutHeadSerST_ONE.4; ** Drive busy
*                                   AT %CutHeadSerST_ONE.5; ** Free
FocusUpperLimitReached              AT %CutHeadSerST_ONE.6; ** Upper focus limit reached
FocusLowerLimitedReached            AT %CutHeadSerST_ONE.7; ** Lower focus limit reached

AlarmDriveOverload                  AT %CutHeadSerST_TWO.0; ** Drive fault: overload
*                                   AT %CutHeadSerST_TWO.1; ** 
FocusSetpointReached                AT %CutHeadSerST_TWO.2; ** Focal position reached
CuttingHeadDriveReady               AT %CutHeadSerST_TWO.3; ** Cutting head drive ready
ReferenceTravelActive               AT %CutHeadSerST_TWO.4; ** Reference travel active
AlarmCommunicationDrive             AT %CutHeadSerST_TWO.5; ** Drive fault: cannot communicate with servo motor
AlarmReferenceTravelDrive           AT %CutHeadSerST_TWO.6; ** Drive fault: no reference travel
AlarmPSVoltageDrive                 AT %CutHeadSerST_TWO.7; ** Drive fault: supply voltage too low

***  PLC TO CUTBOX
 
PLC_CutBoxControl_ONE               AT %CutBoxControl_ONE;      ** CutBox control bits 1
PLC_CutBoxControl_TWO               AT %CutBoxControl_TWO;      ** CutBox control bits 2
PLC_CuttHeadContr_ONE               AT %CuttHeadContr_ONE;      ** Cutting head control bits 1
PLC_CuttHeadContr_TWO               AT %CuttHeadContr_TWO;	    ** Cutting head control bits 2

DistanceCalibrateCharCurve          AT %CutBoxControl_ONE.0;    ** Reference point calibration
SensorController                    AT %CutBoxControl_ONE.1;    ** Activates the operating mode (sensor controller = 1 / distance measuring system = 0)
*                                   AT %CutBoxControl_ONE.2;    ** Free
NozzelLostCalibration               AT %CutBoxControl_ONE.3;    ** Single nozzle lost calibration
DistanceStrobeSignal                AT %CutBoxControl_ONE.4;    ** Signal to be used for calibration
DistanceResetError                  AT %CutBoxControl_ONE.5;    ** ???????
ManualMode                          AT %CutBoxControl_ONE.6;    ** Activates manual CutBox mode on GUI
SyncToFieldbus                      AT %CutBoxControl_ONE.7;    ** Synchronization of the CutBox cycle with the fieldbus (yes = 1)
	
CuttingHeadPower                    AT %CutBoxControl_TWO.0;    ** Switches the cutting head power supply on/off 0 = enabled 1 = disabled
    *                                   AT %CutBoxControl_TWO.1;    ** Free
    *                                   AT %CutBoxControl_TWO.2;    ** Free
    *                                   AT %CutBoxControl_TWO.3;    ** Free
    *                                   AT %CutBoxControl_TWO.4;    ** Free
    *                                   AT %CutBoxControl_TWO.5;    ** Free
    *                                   AT %CutBoxControl_TWO.6;    ** Free
SaveOutputData	                    AT %CutBoxControl_TWO.7;    ** Stores the values of all acyclic and cyclic output PLC data in a non-volatile memory for loading after a power failure (save once=1).
	
DistanceReferenceTravel             AT %CuttHeadContr_ONE.0;    ** Perform reference travel
    *                                   AT %CuttHeadContr_ONE.1;    ** Free
    *                                   AT %CuttHeadContr_ONE.2;    ** Free
    *                                   AT %CuttHeadContr_ONE.3;    ** Free
    *                                   AT %CuttHeadContr_ONE.4;    ** Free
FocusNewSetpoint                    AT %CuttHeadContr_ONE.5;    ** Approach focal position
    *                                   AT %CuttHeadContr_ONE.6;    ** Free
ResetPbus                           AT %CuttHeadContr_ONE.7;    ** Reset communicaiton between cutting head and CutBox

ProcessDataSend                     AT %CuttHeadContr_TWO.0;    ** ?????
    *                                   AT %CuttHeadContr_TWO.1;    ** Free
    *                                   AT %CuttHeadContr_TWO.2;    ** Free
    *                                   AT %CuttHeadContr_TWO.3;    ** Free
    *                                   AT %CuttHeadContr_TWO.4;    ** Free
    *                                   AT %CuttHeadContr_TWO.5;    ** Free
    *                                   AT %CuttHeadContr_TWO.6;    ** Free
    *                                   AT %CuttHeadContr_TWO.7;    ** Free
*
 

* ProCutter 2 settings
 
PLC_ProCutterType               AT %ProCutterType;          ** 1: ProCutter 2.0 -- 2: ProCutter 2.0 Zoom
PLC_StraylSensoProtWindGain     AT %StraylSProtWGain;       ** 1: Low -- 2: Medium -- 3: High -- Read gain factor for protective window process stray light (top and bottom)
PLC_ActualImageRatio            AT %ActualImageRatio;       ** PC zoom only: current image ratio. For the assignment of variable value to image ratio
PLC_CurrentFocalPosition        AT %CurFocalPosition;       ** Current focal position [10μm] 
PLC_CustomerFocusOffset         AT %CustFocusOffset;        ** Read customer offset [10 μm]
		
* Distance functions
		 
PLC_WorkingDistance             AT %WorkingDistance;        ** Distance z [1 μm]
PLC_NextCalibPosition           AT %NextCalibPosition;      ** Next reference point position in 16-point calibration [μm]
PLC_PIDSpeedSetpoint            AT %PIDSpeedSetpoint;       ** Speed setpoint value of the PID sensor controller, value range from min to max [-27648 to 27648]
		
* Sensor values		
		
PLC_TempProtWindow              AT %TempProtWindow;         ** Process protective window temperature [0.01 °C]
PLC_TempFocusingLens            AT %TempFocusingLens;       ** Focusing lens temperature [0.01 °C]
PLC_TempCollimatingLens         AT %TempCollimatingLens;    ** Collimating lens temperature [0.01 °C]
PLC_TempCuttingHead             AT %TempCuttingHead;        ** Head temperature [0.01 °C]
PLC_TempDistanceSensor          AT %TempDistanceSensor;     ** Temperature output distance sensor [0.01 °C] (0 - 100°C)
PLC_StraylightBeamPath          AT %StraylightBeamPath;     ** Beam path stray light [0.01 %]
PLC_StraylProtectWindowProc     AT %StrayLProtectWProc;     ** Stray light, protective window [0.01 %]
PLC_StrayLProtectWindowCol      AT %StrayLProtectWCol;      ** Stray light, protective window COL [0.01 %]
PLC_PresLeakProtectWindowProc   AT %PresLeakPrWinProc;      ** Process protective window pressure [10 mbar]
PLC_ActualCuttingGasPres        AT %ActualCutGasPres;       ** Cutting gas pressure [10 mbar]
PLC_HumidityInCuttingHead       AT %HumInCutHead;	        ** Humidity inside head [0.01%]
PLC_Plasma                      AT %Plasma;                 ** PlasmaTec value (0 … 1023)
		 

		 
* ProCutter 2.0 control settings 
 
PLC_StraylightGainFactor        AT %StrayLGainFactor;       ** 1: too low -- 2: medium -- 3: high - Default value: 1 -- Gain factor for writing the two protective window stray light sensors
PLC_MaxStrayLColLens            AT %MaxStrayLColLens;       ** Max. stray light, protective window collimating lens [0.01%] (top) Default value: 80%
PLC_MaxStrayLProtWindowProc     AT %MaxStrayLProtWProc;     ** Max. stray light, process protectiv window [0.01%] (bottom) Default value: 80%
PLC_MaxHumidityCuttingHead      AT %MaxHumCuttingHead;      ** Max. humidity [0.01%] Default value: 100%
PLC_MaxTempProtWindowProcess    AT %MaxTempProtWProc;       ** Max. protective window temperature [0.01°C] Default value: 90°C
PLC_MaxTempCollimatingLens      AT %MaxTempColLens;         ** Max. collimating lens temperature [0.01°C] Default value: 100°C
PLC_MaxTempCuttingHead          AT %MaxTempCuttingHead;     ** Max. focusing lens temperature [0.01°C] Default value: 80°C 
                                                            ** Max. laser head temperature [0.01 °C] Default value: 55°C
PLC_MaxPresAssistGas            AT %MaxPresAssistGas;       ** Max. cutting gas pressure [0,01 bar] Default value: 25Bar
PLC_MaxPresLeakProtWProc        AT %MaxPrLeakProtWProc;     ** Max. protective window pressure loss [0.01bar] Default value: 3000mBar
PLC_MaxStrayLCutHead            AT %MaxStrayLCutHead;       ** Max. stray light [0.01 %]
PLC_ImageRatio                  AT %ImageRatio;             ** Adjust the image ratio (can only be used with zoom heads).
PLC_FocalPosition               AT %FocalPosition;          ** Focal position setpoint [10μm] Note: The focal position setpoint will only be accepted if 1027_CB_Rd_FocusNewSetpoint is enabled. Default value: 0.00mm
		
* Distance function control settings
 
PLC_SelectCharacteristicCurve   AT %SelectCharCurve;        ** Characteristic curve selection Default value: 0
PLC_DistanceFilterWindowSize    AT %DistFiltWindSize;       ** Mean value time window (1 - 255) Default value: 6
PLC_P_Parameter                 AT %P_Parameter;            ** PID controller P parameter (1 - 255) Default value: 150
PLC_I_Parameter                 AT %I_Parameter;            ** PID controller I parameter (1 - 255) Default-value: 0
PLC_D_Parameter                 AT %D_Parameter;            ** PID controller D parameter (1 - 255) Default value: 0
PLC_DistanceStandoffValue       AT %DistStandoffValue;      ** Sensor control: working distance between nozzle and workpiece [1 μm] (0.03 - 0.97 x measuring range limit)
PLC_SpatterfilterTime           AT %SpatterfilterTime;      ** Sensor control: Spatter compensation time [ms] (0 - 1020) Default value: 20 ms
PLC_TipTouchLevel               AT %TipTouchLevel;          ** TipTouch [μm] (0-1000μm) Default value: 100 μm
PLC_MeasuringRangeLimit         AT %MeasRangeLimit;         ** Measuring range end value [μm] (2000-40000 μm) Default value: 30,000 mm
PLC_CollisionDelay              AT %CollisionDelay;         ** Collision delay [ms] to suppress ‘tip touch’ and ‘collision’ (0 - 1020 ms) Default value: 20 ms
PLC_SetPointBand                AT %SetPointBand;           ** Setpoint range of the sensor controller for the signal ‘position reached’ [μm] (100 - 5000)
PLC_ControlDynamicBand          AT %ControlDynBand;         ** Control voltage dynamics [mV/ms] (100 - 9999) to limit linear drive acceleration
PLC_MaxTemperatureSensorUnit    AT %MaxTempSensorUnit;      ** Power Characteristic Curve Required for old firmware versions of specific cutting heads

  
END_VAR 