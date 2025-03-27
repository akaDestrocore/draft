#!/bin/bash
set -e

echo "Building firmware components..."

BUILD_MODE="--release"
if [ "$1" == "debug" ]; then
    BUILD_MODE=""
    echo "Building in debug mode..."
else
    echo "Building in release mode..."
fi

if [ ! -d "stm32f4-pac" ]; then
    echo "Generating PAC from SVD..."
    mkdir -p stm32f4-pac
    cd stm32f4-pac
    
    cargo init --lib
    
    curl -L -O https://raw.githubusercontent.com/stm32-rs/stm32-rs/master/svd/stm32f407.svd
    
    svd2rust -i stm32f407.svd --target cortex-m -g
    
    rm -rf src/
    form -i lib.rs -o src/
    rm lib.rs
    mv generic.rs src/
    
    cat > build.rs << EOF
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
}
EOF
    
    cat > Cargo.toml << EOF
[package]
name = "stm32f4-pac"
version = "0.1.0"
edition = "2021"
description = "Low-level register access for STM32F407"

[dependencies]
bare-metal = "1.0.0"
cortex-m = "0.7.7"
cortex-m-rt = { version = "0.7.3", optional = true }
vcell = "0.1.3"

[features]
rt = ["cortex-m-rt"]
EOF
    
    cd ..
fi

echo "Building boot..."
cargo build -p boot $BUILD_MODE

echo "Building loader..."
cargo build -p loader $BUILD_MODE

echo "Building updater..."
cargo build -p updater $BUILD_MODE

echo "Building application..."
cargo build -p application $BUILD_MODE

echo "All components built successfully!"

if [ "$1" == "debug" ]; then
    echo "Skipping binary creation for debug build..."
    echo "Debug binaries are available at:"
    echo "- Boot: target/thumbv7em-none-eabihf/debug/boot"
    echo "- Loader: target/thumbv7em-none-eabihf/debug/loader"
    echo "- Updater: target/thumbv7em-none-eabihf/debug/updater"
    echo "- Application: target/thumbv7em-none-eabihf/debug/application"
    exit 0
fi

echo "Creating binary files..."

cargo objcopy --bin boot --release -- -O binary boot.bin
cargo objcopy --bin loader --release -- -O binary loader.bin
cargo objcopy --bin updater --release -- -O binary updater.bin
cargo objcopy --bin application --release -- -O binary application.bin

echo "Merging binary files..."
python3 scripts/merge_images.py boot.bin loader.bin updater.bin application.bin

echo "Firmware file created successfully!"
echo "To flash the device with the merged firmware, run:"
echo "probe-rs download --chip STM32F407VGTx --format bin --base-address 0x08000000 merged_firmware.bin"

if [ "$2" == "flash" ]; then
    echo "Flashing device with merged firmware..."
    probe-rs download --chip STM32F407VGTx --format bin --base-address 0x08000000 merged_firmware.bin
fi