Based on cortex_m_quickstart, check the docs (first commit or https://docs.rs/cortex-m-quickstart/0/cortex_m_quickstart/)

# Installing some dependencies
cargo install xargo
cargo install svd2rust
cargo install rustfmt
cargo install cargo-board

# Generating hardware definitions from SVDs (see mcu/<target>/peripherals)
svd2rust -i mcu/stm32f20x/peripherals/STM32F439x.svd | rustfmt | tee mcu/stm32f20x/peripherals/lib.rs
svd2rust -i mcu/stm32f0xx/peripherals/STM32F0xx.svd | rustfmt | tee mcu/stm32f0xx/peripherlas/lib.rs

# Debugging an application
# It's important to do xargo operations at the workspace root (xargo.toml & .gdbinit)
cargo board netboard run -p blink

# Generating documentation for a crate
xargo board flexperiment_mini doc -p stm32f439x --open --no-deps

# TODO
- CARGO_TARGET_DIR is not quite optimal (cargo-board)
