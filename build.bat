@echo off

REM (allen): quit early if we already have cl
where /q cl
IF not %ERRORLEVEL% == 0 (call ..\misc\shell.bat)

IF NOT EXIST ..\..\build mkdir ..\..\build
pushd ..\..\build

cl -Zi -DGAME_SLOW=1 W:\game\code\win32_game.cpp user32.lib Gdi32.lib Winmm.lib
popd