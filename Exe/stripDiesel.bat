@echo off
del *.pdb
for %%a in (*d.dll) do if /I "%%a" NEQ "errorhand.dll" if /I "%%a" NEQ "vshared.dll" del "%%a"
for %%a in (*d.exe) do if /I "%%a" NEQ "guicad.exe"    if /I "%%a" NEQ "kvsend.exe"  del "%%a"
