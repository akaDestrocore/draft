Write-Host "Building firmware components..." -ForegroundColor Cyan

$BUILD_MODE = "--release"
if ($args[0] -eq "debug") {
    $BUILD_MODE = ""
    Write-Host "Building in debug mode..." -ForegroundColor Yellow
} else {
    Write-Host "Building in release mode..." -ForegroundColor Green
}

if (-not (Test-Path -Path "stm32f4")) {
    Write-Host "stm32f4 PAC not found, generating from SVD..." -ForegroundColor Cyan
    
    $svdUrl = "https://stm32-rs.github.io/stm32-rs/stm32f407.svd.patched"
    $svdPath = "stm32f407.svd.patched"
    
    try {
        Write-Host "Downloading SVD file from URL..." -ForegroundColor Cyan
        Invoke-WebRequest -Uri $svdUrl -OutFile $svdPath
        Write-Host "SVD file downloaded successfully!" -ForegroundColor Green
    } catch {
        Write-Host "Failed to download SVD file: $_" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Creating directory structure..." -ForegroundColor Cyan
    New-Item -ItemType Directory -Path "stm32f4" -Force | Out-Null
    Set-Location -Path "stm32f4"
    
    Write-Host "Initializing Cargo library project..." -ForegroundColor Cyan
    cargo init --lib
    if (-not $?) {
        Write-Host "Failed to initialize stm32f4 library!" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Generating Rust code from SVD..." -ForegroundColor Cyan
    svd2rust -i ../stm32f407.svd.patched --target cortex-m -g
    if (-not $?) {
        Write-Host "Failed to generate Rust files from SVD!" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Reorganizing project structure..." -ForegroundColor Cyan
    Remove-Item -Path "src" -Recurse -Force -ErrorAction SilentlyContinue
    form -i lib.rs -o src/
    if (-not $?) {
        Write-Host "Failed to format generated files!" -ForegroundColor Red
        exit 1
    }
    
    Remove-Item -Path "lib.rs" -Force
    Move-Item -Path "generic.rs" -Destination "src/"
    Move-Item -Path "build.rs" -Destination "./" -ErrorAction SilentlyContinue
    
    Write-Host "Creating Cargo.toml..." -ForegroundColor Cyan
    $cargoToml = @"
[package]
name = "stm32f4"
version = "0.1.0"
edition = "2021"
description = "Low-level register access for stm32f4"

[dependencies]
bare-metal = "1.0.0"
cortex-m = "0.7.7"
cortex-m-rt = { version = "0.7.3", optional = true }
vcell = "0.1.3"

[features]
rt = ["cortex-m-rt"]
"@
    
    Set-Content -Path "Cargo.toml" -Value $cargoToml
    
    Set-Location -Path ".."
    
    Remove-Item -Path $svdPath -Force
    
    Write-Host "stm32f4 PAC initialization completed successfully!" -ForegroundColor Green
}

Write-Host "Building boot..." -ForegroundColor Cyan
cargo build -p boot $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build boot component!" -ForegroundColor Red
    exit 1
}

Write-Host "Building loader..." -ForegroundColor Cyan
cargo build -p loader $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build loader component!" -ForegroundColor Red
    exit 1
}

Write-Host "Building updater..." -ForegroundColor Cyan
cargo build -p updater $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build updater component!" -ForegroundColor Red
    exit 1
}

Write-Host "Building application..." -ForegroundColor Cyan
cargo build -p application $BUILD_MODE
if (-not $?) {
    Write-Host "Failed to build application component!" -ForegroundColor Red
    exit 1
}

Write-Host "All components built successfully!" -ForegroundColor Green

if ($args[0] -eq "debug") {
    Write-Host "Skipping binary creation for debug build..." -ForegroundColor Yellow
    Write-Host "Debug binaries are available at:" -ForegroundColor Cyan
    Write-Host "- Boot: target\thumbv7em-none-eabihf\debug\boot" -ForegroundColor Cyan
    Write-Host "- Loader: target\thumbv7em-none-eabihf\debug\loader" -ForegroundColor Cyan
    Write-Host "- Updater: target\thumbv7em-none-eabihf\debug\updater" -ForegroundColor Cyan
    Write-Host "- Application: target\thumbv7em-none-eabihf\debug\application" -ForegroundColor Cyan
    exit 0
}

Write-Host "Creating binary files..." -ForegroundColor Cyan
Write-Host "(Ensure cargo-binutils is installed)" -ForegroundColor Yellow

cargo objcopy --bin boot --release -- -O binary boot.bin
if (-not (Test-Path -Path "boot.bin")) {
    Write-Host "Failed to create boot.bin!" -ForegroundColor Red
    exit 1
}

cargo objcopy --bin loader --release -- -O binary loader.bin
if (-not (Test-Path -Path "loader.bin")) {
    Write-Host "Failed to create loader.bin!" -ForegroundColor Red
    exit 1
}

cargo objcopy --bin updater --release -- -O binary updater.bin
if (-not (Test-Path -Path "updater.bin")) {
    Write-Host "Failed to create updater.bin!" -ForegroundColor Red
    exit 1
}

cargo objcopy --bin application --release -- -O binary application.bin
if (-not (Test-Path -Path "application.bin")) {
    Write-Host "Failed to create application.bin!" -ForegroundColor Red
    exit 1
}

Write-Host "Merging binary files..." -ForegroundColor Cyan
python scripts\merge_images.py boot.bin loader.bin updater.bin application.bin
if (-not (Test-Path -Path "merged_firmware.bin")) {
    Write-Host "Failed to create merged firmware file!" -ForegroundColor Red
    exit 1
}

Write-Host "Firmware file created successfully." -ForegroundColor Green
Write-Host "To flash the device with the merged firmware, run:" -ForegroundColor Cyan
Write-Host "probe-rs download --chip STM32F407VGTx --binary-format bin --base-address 0x08000000 merged_firmware.bin" -ForegroundColor Cyan

if ($args[1] -eq "flash") {
    Write-Host "Flashing device with merged firmware..." -ForegroundColor Cyan
    probe-rs download --chip STM32F407VGTx --binary-format bin --base-address 0x08000000 merged_firmware.bin
}