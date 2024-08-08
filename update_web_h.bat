@echo off
setlocal enabledelayedexpansion

REM Directory paths
set HTML_PATH=.
set SRC_PATH=.\src

REM File names
set GZ_FILE=web_smars.html.gz
set WEB_H_FILE=%SRC_PATH%\web.h

REM Check if gz file exists
if not exist "%HTML_PATH%\%GZ_FILE%" (
    echo Error: Gzipped file %GZ_FILE% not found.
    exit /b 1
)

REM Step 1: Generate C byte array from gzipped HTML file
echo Generating C byte array...
set ARRAY_CONTENT=

REM Test the Python command separately
for /f "delims=" %%A in ('python -c "import binascii; 
import sys; 
def file_to_c_array(file_path): 
    array_elements = []; 
    with open(file_path, ''rb'') as file: 
        byte = file.read(1); 
        while byte: 
            array_elements.append(''0x'' + binascii.hexlify(byte).decode(''utf-8'')); 
            byte = file.read(1); 
    print(', '.join(array_elements));" "%HTML_PATH%\%GZ_FILE%"') do (
    set ARRAY_CONTENT=%%A
)

REM Debugging output
echo Array content generated: !ARRAY_CONTENT!

REM Check if ARRAY_CONTENT is empty
if "!ARRAY_CONTENT!"=="" (
    echo Error: Failed to generate C byte array.
    exit /b 1
)

REM Step 2: Update the web.h file with the new array
echo Updating web.h file...
(
    echo const uint8_t index_html_gz[] = {
    echo !ARRAY_CONTENT!
    echo };
) > "%WEB_H_FILE%"

REM Verify if the web.h file was updated
if exist "%WEB_H_FILE%" (
    echo web.h file updated successfully.
) else (
    echo Error: Failed to update web.h file.
)

echo Update complete!
endlocal