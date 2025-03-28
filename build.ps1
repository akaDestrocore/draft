Write-Host "Building firmware components..."

$BUILD_MODE = "--release"
if ($args[0] -eq "debug") {
    $BUILD_MODE = ""
    Write-Host "Building in debug mode..."
} else {
    Write-Host "Building in release mode..."
}

if (-not (Test-Path -Path "stm32f407")) {
    Write-Host "Generating PAC from SVD..."
    New-Item -ItemType Directory -Path "stm32f407" | Out-Null
    Set-Location -Path "stm32f407"
    
    cargo init --lib
    if (-not $?) {
        Write-Host "Failed to initialize stm32f407 library!"
        exit 1
    }
    
    svd2rust -i stm32f407.svd --target cortex-m -g
    if (-not $?) {
        Write-Host "Failed to generate Rust files from SVD!"
        exit 1
    }
    
    Remove-Item -Path "src" -Recurse -Force -ErrorAction SilentlyContinue
    form -i lib.rs -o src/
    if (-not $?) {
        Write-Host "Failed to format generated files!"
        exit 1
    }
    
    Remove-Item -Path "lib.rs" -Force
    Move-Item -Path "generic.rs" -Destination "src/"
    
    @"
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
}
"@ | Out-File -FilePath "build.rs" -Encoding utf8
    
    @"
[package]
name = "stm32f407"
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
"@ | Out-File -FilePath "Cargo.toml" -Encoding utf8
    
    Set-Location -Path ".."
}

Write-Host "Building boot..."
cargo build -p boot $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build boot component!"
    exit 1
}

Write-Host "Building loader..."
cargo build -p loader $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build loader component!"
    exit 1
}

Write-Host "Building updater..."
cargo build -p updater $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build updater component!"
    exit 1
}

Write-Host "Building application..."
cargo build -p application $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build application component!"
    exit 1
}

Write-Host "All components built successfully!"

if ($args[0] -eq "debug") {
    Write-Host "Skipping binary creation for debug build..."
    Write-Host "Debug binaries are available at:"
    Write-Host "- Boot: target\thumbv7em-none-eabihf\debug\boot"
    Write-Host "- Loader: target\thumbv7em-none-eabihf\debug\loader"
    Write-Host "- Updater: target\thumbv7em-none-eabihf\debug\updater"
    Write-Host "- Application: target\thumbv7em-none-eabihf\debug\application"
    exit 0
}

Write-Host "Creating binary files..."
Write-Host "(Ensure cargo-binutils is installed)"

cargo objcopy --bin boot --release -- -O binary boot.bin
if (-not (Test-Path -Path "boot.bin")) {
    Write-Host "Failed to create boot.bin!"
    exit 1
}

cargo objcopy --bin loader --release -- -O binary loader.bin
if (-not (Test-Path -Path "loader.bin")) {
    Write-Host "Failed to create loader.bin!"
    exit 1
}

cargo objcopy --bin updater --release -- -O binary updater.bin
if (-not (Test-Path -Path "updater.bin")) {
    Write-Host "Failed to create updater.bin!"
    exit 1
}

cargo objcopy --bin application --release -- -O binary application.bin
if (-not (Test-Path -Path "application.bin")) {
    Write-Host "Failed to create application.bin!"
    exit 1
}

Write-Host "Merging binary files..."
python scripts\merge_images.py boot.bin loader.bin updater.bin application.bin
if (-not (Test-Path -Path "merged_firmware.bin")) {
    Write-Host "Failed to create merged firmware file!"
    exit 1
}

Write-Host "Firmware file created successfully."
Write-Host "To flash the device with the merged firmware, run:"
Write-Host "probe-rs download --chip STM32F407VGTx --format bin --base-address 0x08000000 merged_firmware.bin"

if ($args[1] -eq "flash") {
    Write-Host "Flashing device with merged firmware..."
    probe-rs download --chip STM32F407VGTx --binary-format bin --base-address 0x08000000 merged_firmware.bin
}