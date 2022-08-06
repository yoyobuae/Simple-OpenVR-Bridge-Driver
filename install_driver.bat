@ECHO OFF
SETLOCAL EnableExtensions EnableDelayedExpansion

SET "DRIVER_NAME=apriltagtrackers"

SET "DRIVER_PATH=%cd%\%DRIVER_NAME%"

REM first, find the path to the vrpathreg.exe

set /a count = 1
set /a line = 0

REM the path can be found in openvrpaths.vrpath, which should always be in %localappdata%

REM the file is in json, and the path is under the "runtime" field. We first find the line number of the "runtime" field

for /F %%A in (%localappdata%\openvr\openvrpaths.vrpath) do (
 if %%A=="runtime" set /a line = count+2
 set /a count += 1
)

set /a count = 1

REM then, we parse whole lines and save the path. It should be 2 fields under "runtime"

for /F "tokens=*" %%A in (%localappdata%\openvr\openvrpaths.vrpath) do (
 if !count!==!line! set VRPATH=%%A
 set /a count += 1
)

set VRPATH=%VRPATH:"=%

set VRPATHREG_EXE=!VRPATH!\\bin\\win64\\vrpathreg.exe

IF "%1"=="help" (
    ECHO Usage: install_driver.bat ^[^<driver path^>^] ^[^<path to vrpathreg.exe^>^ ^[^<path to steamvr.vrsettings^>^]
    ECHO ^<driver path^> defaults to "%DRIVER_PATH%"
    ECHO ^<path to vrpathreg.exe^> defaults to "%VRPATHREG_EXE%"
    GOTO end
)

IF NOT "%1"=="" (
    SET "DRIVER_PATH=%1"
)

IF NOT "%2"=="" (
    SET "VRPATHREG_EXE=%2"
)

IF NOT EXIST "%DRIVER_PATH%" (
    ECHO Driver not found: "%DRIVER_PATH%"
    echo(
    echo Check that you downloaded the right files and make sure to unzip the folder before running this script^^!
    echo(
    GOTO end
)

IF NOT EXIST "%VRPATHREG_EXE%" (
    ECHO vrpathreg.exe not found: "%VRPATHREG_EXE%"
    echo(
    echo This usualy means an error with your SteamVR installation, or if you have multiple installations of SteamVR.
    echo You can also try to locate the vrpathreg.exe file yourself and input it below. The file is inside SteamVR\bin\win64.
    echo(
    
    set /p "VRPATHREG_EXE=Enter full path to vrpathreg.exe: "
)

REM remove driver from older versions

IF EXIST "!VRPATH!\\drivers\\apriltagtrackers" (
    ECHO Found old driver install at "!VRPATH!\\drivers\\apriltagtrackers, removing

    RMDIR /S /Q "!VRPATH!\\drivers\\apriltagtrackers"	
)

REM clean up our driver installs
CALL "%VRPATHREG_EXE%" removedriver "%DRIVER_PATH%"
IF NOT "%errorlevel%"=="0" GOTO end

CALL "%VRPATHREG_EXE%" removedriverswithname "%DRIVER_NAME%"
IF NOT "%errorlevel%"=="0" GOTO end

REM add the new driver
CALL "%VRPATHREG_EXE%" adddriver "%DRIVER_PATH%"
IF NOT "%errorlevel%"=="0" GOTO end

REM display the current configuration, with the newly added driver
CALL "%VRPATHREG_EXE%" show

echo(
echo The driver has been installed successfully^^!
echo(

echo(
echo Enabling multiple drivers in SteamVR config...
echo(

REM the path can be found in openvrpaths.vrpath, which should always be in %localappdata%

REM the file is in json, and the path is under the "config" field. We first find the line number of the "config" field

set /a count = 1
set /a line = 0

for /F %%A in (%localappdata%\openvr\openvrpaths.vrpath) do (
 if %%A=="config" set /a line = count+2
 set /a count += 1
)

set /a count = 1

REM then, we parse whole lines and save the path. It should be 2 fields under "config"

for /F "tokens=*" %%A in (%localappdata%\openvr\openvrpaths.vrpath) do (
 if !count!==!line! set VRCONFIG=%%A
 set /a count += 1
)

set VRCONFIG=%VRCONFIG:"=%

set VRSETTINGS=!VRCONFIG!\\steamvr.vrsettings

IF NOT "%3"=="" (
    SET "VRSETTINGS=%3"
)

IF NOT EXIST "%VRSETTINGS%" (
    ECHO steamvr.vrsettings not found: "%VRSETTINGS%"
    echo(
    echo This usualy means an error with your SteamVR installation, or if you have multiple installations of SteamVR.
    echo You can also try to locate the steamvr.vrsettings file yourself and input it below. The file is inside Steam/config.
    echo(
    
    set /p "VRSETTINGS=Enter full path to steamvr.vrsettings:"
)

REM we search for the steamvr section, and add the line to it

    set search="steamvr" : {
    set replace="steamvr" : { "activateMultipleDrivers" : true,

    set "textFile=%VRSETTINGS%"

    for /f "delims=" %%i in ('type "%textFile%" ^& break ^> "%textFile%" ') do (
        set "line=%%i"
        setlocal enabledelayedexpansion
        >>"%textFile%" echo(!line:%search%=%replace%!
        endlocal
    )

REM if activateMultipleDrivers is already set to false, set it to true. SteamVR will fix up the config and remove duplicates

    set search="activateMultipleDrivers" : false,
    set replace="activateMultipleDrivers" : true,

    for /f "delims=" %%i in ('type "%textFile%" ^& break ^> "%textFile%" ') do (
        set "line=%%i"
        setlocal enabledelayedexpansion
        >>"%textFile%" echo(!line:%search%=%replace%!
        endlocal
    )

echo(
echo Finished^^!
echo(

:end
ENDLOCAL
PAUSE
REM END OF FILE
