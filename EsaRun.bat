ECHO OFF
ping 1.1.1.1 -n 1 -w 1000 > NUL
cd exe
cmd /c start .\kvisogui.exe
ping 1.1.1.1 -n 3 -w 1000 > NUL
@REM start tkskey.exe ..\plc\_key\vrtc5.key
@REM start tkskey.exe ..\plc\_key\ESAtest.key
@REM start tkskey.exe ..\plc\_key\nlightIO.key
start tkskey.exe ..\plc\_key\usrLogicIO.key
start tkskey.exe ..\plc\_key\LogicIO.key
start tkskey.exe ..\plc\_key\caphead.key