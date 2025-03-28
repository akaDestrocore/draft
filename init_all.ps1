Write-Host "Installing Rust utilities..." -ForegroundColor Cyan
cargo install svd2rust
cargo install form
cargo install cargo-binutils
rustup component add llvm-tools-preview

Write-Host "Downloading SVD file from URL..." -ForegroundColor Cyan
$svdUrl = "https://stm32-rs.github.io/stm32-rs/stm32f407.svd.patched"
$svdPath = "stm32f407.svd.patched"

try {
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

Write-Host "Generating Rust code from SVD..." -ForegroundColor Cyan
svd2rust -i ../stm32f407.svd.patched --target cortex-m -g

Write-Host "Reorganizing project structure..." -ForegroundColor Cyan
Remove-Item -Path "src" -Recurse -Force -ErrorAction SilentlyContinue
form -i lib.rs -o src/
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
Write-Host "You can now build your firmware components." -ForegroundColor Green