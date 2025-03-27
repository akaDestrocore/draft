Write-Host "Installing Rust utilities..." -ForegroundColor Cyan
cargo install svd2rust
cargo install form
cargo install cargo-binutils
rustup component add llvm-tools-preview

Write-Host "Creating directory structure..." -ForegroundColor Cyan
New-Item -ItemType Directory -Path "stm32f4-pac" -Force | Out-Null
Set-Location -Path "stm32f4-pac"

Write-Host "Initializing Cargo library project..." -ForegroundColor Cyan
cargo init --lib

Write-Host "Generating Rust code from SVD..." -ForegroundColor Cyan
svd2rust -i stm32f407.svd --target cortex-m -g

Write-Host "Reorganizing project structure..." -ForegroundColor Cyan
Remove-Item -Path "src" -Recurse -Force -ErrorAction SilentlyContinue
form -i lib.rs -o src/
Remove-Item -Path "lib.rs" -Force
Move-Item -Path "generic.rs" -Destination "src/"
Move-Item -Path "build.rs" -Destination "./" -ErrorAction SilentlyContinue

Write-Host "Creating Cargo.toml..." -ForegroundColor Cyan
$cargoToml = @"
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
"@

Set-Content -Path "Cargo.toml" -Value $cargoToml

Set-Location -Path ".."

Write-Host "STM32F4 PAC initialization completed successfully!" -ForegroundColor Green
Write-Host "You can now build your firmware components." -ForegroundColor Green