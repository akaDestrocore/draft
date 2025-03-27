#!/bin/bash
# Создание PAC (Peripheral Access Crate) с svd2rust

# Настройка окружения Rust один раз для всего скрипта
export RUSTUP_HOME=~/rustup-standalone
export CARGO_HOME=~/cargo-standalone
export PATH="$CARGO_HOME/bin:$PATH"

# Установка инструментов
cargo install svd2rust
cargo install form
cargo install cargo-binutils
rustup component add llvm-tools-preview

# Создаем директорию для PAC
mkdir -p stm32f4-pac
cd stm32f4-pac

# Инициализируем пакет
cargo init --lib

# Генерируем код из SVD
svd2rust -i stm32f407.svd --target cortex-m -g

# Форматируем код
rm -rf src/
form -i lib.rs -o src/
rm lib.rs
mv generic.rs src/
mv build.rs ./

# Обновляем Cargo.toml
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