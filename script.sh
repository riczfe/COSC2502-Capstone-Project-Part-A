#!/bin/bash

# Set variables
HTML_FILE="web_smars.html"
GZIP_FILE="web_smars.html.gz"
WEB_H_FILE="web.h"

# Compress the HTML file
gzip -c $HTML_FILE > $GZIP_FILE

# Generate C byte array from gzipped file and write it directly into web.h
echo "const uint8_t index_html_gz[] = {" > $WEB_H_FILE
hexdump -v -e '1/1 "0x%.2X, "' $GZIP_FILE >> $WEB_H_FILE
echo "};" >> $WEB_H_FILE

echo "Updated $WEB_H_FILE with new HTML content."
