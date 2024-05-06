# Step 1: Update and Compress HTML File

gzip -c web_smars.html > web_smars.html.gz


# Step 2: Convert Gzip File to C Byte Array (`convert_to_c_array.py`)

```
import binascii

def file_to_c_array(file_path, array_name):
    print(f"const uint8_t {array_name}[] = {{", end="")
    with open(file_path, 'rb') as file:
        byte = file.read(1)
        while byte:
            print(f"0x{binascii.hexlify(byte).decode('utf-8')}, ", end="")
            byte = file.read(1)
    print("};")

file_to_c_array('web_smars.html.gz', 'index_html_gz')
```

COPY CONTENT FROM 
python3 convert_to_c_array.py

Step 3: Update the web.h File
