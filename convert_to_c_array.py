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