#!/bin/bash

# Directory paths
HTML_PATH="."  # Path to the directory containing the HTML file
SRC_PATH="./src"  # Path to the source directory containing the web.h file

# File names
HTML_FILE="web_smars.html"
GZ_FILE="web_smars.html.gz"
WEB_H_FILE="$SRC_PATH/web.h"

# Step 1: Compress the HTML file
echo "Compressing $HTML_FILE..."
gzip -c $HTML_PATH/$HTML_FILE > $HTML_PATH/$GZ_FILE

# Step 2: Generate C byte array from gzipped HTML file
echo "Generating C byte array..."
ARRAY_CONTENT=$(python3 -c "
import binascii
def file_to_c_array(file_path):
    array_elements = []
    with open(file_path, 'rb') as file:
        byte = file.read(1)
        while byte:
            array_elements.append(f'0x{binascii.hexlify(byte).decode('utf-8')}')
            byte = file.read(1)
    return ', '.join(array_elements)
print(file_to_c_array('$HTML_PATH/$GZ_FILE'))
")

# Step 3: Update the web.h file with the new array
echo "Updating web.h file..."
cat > $WEB_H_FILE <<EOF
#ifndef WEB_H
#define WEB_H

#include <stdint.h>

const uint8_t index_html_gz[] = {
$ARRAY_CONTENT
};

EOF

echo "Update complete!"