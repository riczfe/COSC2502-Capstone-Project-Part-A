@echo off
setlocal

REM Directory paths
set HTML_PATH=.

REM File names
set HTML_FILE=web_smars.html
set GZ_FILE=web_smars.html.gz

REM Step 1: Compress the HTML file
if exist "%HTML_PATH%\%GZ_FILE%" (
    echo Removing existing %GZ_FILE%...
    del "%HTML_PATH%\%GZ_FILE%"
)

echo Compressing %HTML_FILE%...
gzip -c "%HTML_PATH%\%HTML_FILE%" > "%HTML_PATH%\%GZ_FILE%"

REM Check if gz file was created successfully
if exist "%HTML_PATH%\%GZ_FILE%" (
    echo %GZ_FILE% created successfully.
) else (
    echo Error: Failed to create %GZ_FILE%.
    exit /b 1
)

echo Compression complete!
endlocal