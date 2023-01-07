@echo off
where /q cl
IF %ERRORLEVEL% == 0 (EXIT /b)

call C:\"Program Files\Microsoft Visual Studio"\2022\Community\VC\Auxiliary\Build\vcvarsall.bat x64
REM set path=w:\bouncer\misc;%path%
