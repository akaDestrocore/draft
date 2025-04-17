import argparse
import os
import struct

HEADER_SIZE = 0x200
IMAGE_MAGIC_OFFSET = 0x00
IMAGE_HDR_VERSION_OFFSET = 0x04
IMAGE_TYPE_OFFSET = 0x06
VERSION_MAJOR_OFFSET = 0x07
VERSION_MINOR_OFFSET = 0x08
VERSION_PATCH_OFFSET = 0x09
PADDING_OFFSET = 0x0A
VECTOR_ADDR_OFFSET = 0x0C
CRC_OFFSET = 0x10
DATA_SIZE_OFFSET = 0x14

def calculate_crc32(data):
    crc = 0xFFFFFFFF
    
    for i in range(0, len(data), 4):
        word = 0
        for j in range(min(4, len(data) - i)):
            word |= data[i + j] << (j * 8)

        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
            crc &= 0xFFFFFFFF

    return crc

def update_header(binary_data, base_addr):
    header = bytearray(binary_data[:HEADER_SIZE])
    firmware = binary_data[HEADER_SIZE:]
    
    # calculate size and CRC
    firmware_size = len(firmware)
    firmware_crc = calculate_crc32(firmware)
    vector_addr = base_addr + HEADER_SIZE
    
    # update header fields
    struct.pack_into('<I', header, VECTOR_ADDR_OFFSET, vector_addr)  # vector_addr (4 bytes)
    struct.pack_into('<I', header, CRC_OFFSET, firmware_crc)         # crc (4 bytes)
    struct.pack_into('<I', header, DATA_SIZE_OFFSET, firmware_size)  # data_size (4 bytes)
    
    print(f"  - Header updated: vector_addr=0x{vector_addr:08X}, data_size={firmware_size} bytes, CRC=0x{firmware_crc:08X}")
    
    return header + firmware

def merge_images(boot_filename, loader_filename, updater_filename, app_filename):
    # memory map offsets from base address 0x08000000
    LOADER_OFFSET = 0x4000   # 0x08004000 
    UPDATER_OFFSET = 0x8000  # 0x08008000 
    APP_OFFSET = 0x20000     # 0x08020000
    BASE_ADDR = 0x08000000

    # get file sizes
    boot_size = os.stat(boot_filename).st_size
    loader_size = os.stat(loader_filename).st_size
    updater_size = os.stat(updater_filename).st_size
    app_size = os.stat(app_filename).st_size

    # verify sizes
    if boot_size > LOADER_OFFSET:
        raise Exception(f"Bootloader is too big! Size: {boot_size}, Max: {LOADER_OFFSET}")
    if loader_size > (UPDATER_OFFSET - LOADER_OFFSET):
        raise Exception(f"Loader is too big! Size: {loader_size}, Max: {UPDATER_OFFSET - LOADER_OFFSET}")
    if updater_size > (APP_OFFSET - UPDATER_OFFSET):
        raise Exception(f"Updater is too big! Size: {updater_size}, Max: {APP_OFFSET - UPDATER_OFFSET}")

    print(f"Merging binaries:")
    print(f"Boot: {boot_filename} ({boot_size} bytes) at offset 0x{0:X}")
    print(f"Loader: {loader_filename} ({loader_size} bytes) at offset 0x{LOADER_OFFSET:X}")
    print(f"Updater: {updater_filename} ({updater_size} bytes) at offset 0x{UPDATER_OFFSET:X}")
    print(f"App: {app_filename} ({app_size} bytes) at offset 0x{APP_OFFSET:X}")

    # read all binaries
    with open(boot_filename, "rb") as f:
        boot_data = f.read()
    with open(loader_filename, "rb") as f:
        loader_data = f.read()
    with open(updater_filename, "rb") as f:
        updater_data = f.read()
    with open(app_filename, "rb") as f:
        app_data = f.read()

    # update headers
    print("\nUpdating headers:")
    print("Loader header:")
    loader_data = update_header(loader_data, BASE_ADDR + LOADER_OFFSET)
    print("Updater header:")
    updater_data = update_header(updater_data, BASE_ADDR + UPDATER_OFFSET)
    print("Application header:")
    app_data = update_header(app_data, BASE_ADDR + APP_OFFSET)

    # merged binary
    output_data = bytearray(APP_OFFSET + len(app_data))
    
    # copy components at their offsets
    output_data[0:len(boot_data)] = boot_data
    output_data[LOADER_OFFSET:LOADER_OFFSET+len(loader_data)] = loader_data
    output_data[UPDATER_OFFSET:UPDATER_OFFSET+len(updater_data)] = updater_data
    output_data[APP_OFFSET:APP_OFFSET+len(app_data)] = app_data

    with open("merged_firmware.bin", "wb") as f:
        f.write(output_data)

    print(f"\nCreated merged binary: merged_firmware.bin")
    print(f"Total size: {os.stat('merged_firmware.bin').st_size} bytes")

    # also save standalone binaries
    with open("loader.bin", "wb") as f:
        f.write(loader_data)
    with open("updater.bin", "wb") as f:
        f.write(updater_data)
    with open("application.bin", "wb") as f:
        f.write(app_data)
    
    print("Individual updated binaries saved:")
    print(f"- loader.bin ({len(loader_data)} bytes)")
    print(f"- updater.bin ({len(updater_data)} bytes)")
    print(f"- application.bin ({len(app_data)} bytes)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("boot", help="Bootloader binary")
    parser.add_argument("loader", help="Loader binary") 
    parser.add_argument("updater", help="Updater binary")
    parser.add_argument("app", help="Application binary")
    
    args = parser.parse_args()

    merge_images(args.boot, args.loader, args.updater, args.app)