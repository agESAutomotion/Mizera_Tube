ECHO OFF
ping 1.1.1.1 -n 1 -w 1000 > NUL
cd C:\Kvara\exe
cmd /c start c:\kvara\exe\kvisogui.exe
ping 1.1.1.1 -n 3 -w 1000 > NUL
//cmd /c start tkskey.exe c:\kvara\plc\_key\vrtc5.key
//cmd /c start tkskey.exe c:\kvara\plc\_key\ESAtest.key
//cmd /c start tkskey.exe c:\kvara\plc\_key\nlightIO.key
cmd /c start tkskey.exe c:\kvara\plc\_key\usrLogicIO.key
cmd /c start tkskey.exe c:\kvara\plc\_key\LogicIO.key
//cmd /c start tkskey.exe c:\kvara\plc\_key\caphead.key