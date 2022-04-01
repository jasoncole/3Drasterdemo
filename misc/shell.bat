@echo off
where /q cl
IF %ERRORLEVEL% == 0 (EXIT /b)

call C:\"Program Files (x86)\Microsoft Visual Studio 12.0"\VC\vcvarsall.bat x64
REM set path=w:\bouncer\misc;%path%
