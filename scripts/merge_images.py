import argparse
import os

def merge_images(boot_filename, loader_filename, updater_filename, app_filename):
    """
    Merge four binaries into one, preserving headers and positions
    """
    # Memory map offsets from base address 0x08000000
    LOADER_OFFSET = 0x4000   # 0x08004000 
    UPDATER_OFFSET = 0x8000  # 0x08008000 
    APP_OFFSET = 0x20000     # 0x08020000

    # Get file sizes
    boot_size = os.stat(boot_filename).st_size
    loader_size = os.stat(loader_filename).st_size
    updater_size = os.stat(updater_filename).st_size
    app_size = os.stat(app_filename).st_size

    # Verify sizes
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

    # Read all binaries
    with open(boot_filename, "rb") as f:
        boot_data = f.read()
    with open(loader_filename, "rb") as f:
        loader_data = f.read()
    with open(updater_filename, "rb") as f:
        updater_data = f.read()
    with open(app_filename, "rb") as f:
        app_data = f.read()

    # Create merged binary
    output_data = bytearray(APP_OFFSET + len(app_data))
    
    # Copy components at their offsets
    output_data[0:len(boot_data)] = boot_data
    output_data[LOADER_OFFSET:LOADER_OFFSET+len(loader_data)] = loader_data
    output_data[UPDATER_OFFSET:UPDATER_OFFSET+len(updater_data)] = updater_data
    output_data[APP_OFFSET:APP_OFFSET+len(app_data)] = app_data

    with open("merged_firmware.bin", "wb") as f:
        f.write(output_data)

    print(f"\nCreated merged binary: merged_firmware.bin")
    print(f"Total size: {os.stat('merged_firmware.bin').st_size} bytes")

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