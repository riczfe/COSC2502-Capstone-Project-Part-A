@echo off

REM Directory paths
set HTML_PATH=.
set SRC_PATH=.\src

REM File names
set HTML_FILE=web_smars.html
set GZ_FILE=web_smars.html.gz
set WEB_H_FILE=%SRC_PATH%\web.h

REM Step 1: Compress the HTML file
if exist %HTML_PATH%\%GZ_FILE% (
    echo Removing existing %GZ_FILE%...
    del %HTML_PATH%\%GZ_FILE%
)

echo Compressing %HTML_FILE%...
gzip -c %HTML_PATH%\%HTML_FILE% > %HTML_PATH%\%GZ_FILE%

REM Step 2: Generate C byte array from gzipped HTML file
echo Generating C byte array...
set ARRAY_CONTENT=
for /f "delims=" %%A in ('python -c "import binascii; import sys; \
def file_to_c_array(file_path): \
    array_elements = []; \
    with open(file_path, ''rb'') as file: \
        byte = file.read(1); \
        while byte: \
            array_elements.append(''0x'' + binascii.hexlify(byte).decode(''utf-8'')); \
            byte = file.read(1); \
    return ', '.join(array_elements); \
print(file_to_c_array('%HTML_PATH%\%GZ_FILE%'))"') do (
    set ARRAY_CONTENT=%%A
)

REM Check if ARRAY_CONTENT is empty
if not defined ARRAY_CONTENT (
    echo Error: Failed to generate C byte array.
    exit /b 1
)

REM Step 3: Update the web.h file with the new array
echo Updating web.h file...
(
    echo const uint8_t index_html_gz[] = {
    echo %ARRAY_CONTENT%
    echo };
) > %WEB_H_FILE%

echo Update complete!