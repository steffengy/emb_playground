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

# Overview
This should illustrate the basic outline of this tree.  
Some of the described organization might not be implemented at all (e.g. HAL).  

* apps
  * Contains the application logic  
    (e.g. a http-server which can run on board X and Y)
* boards
  * Contains a set of predefined "boards", which depend on the required hardware modules  
    (e.g. on the mcu_stm32XY crate)
* hal
  * A generic abstraction over common hardware elements  
    (e.g. GpioPin which is then implemented for the GpioPin of a specific stm32)  
    This only provides the abstractions (mostly traits). The implementation happens
    in the relevant hardwaremodule (e.g. mcu_stm32XY)
    Not much to see yet. The idea is to write board-independant code.
* mcu
  * Contains a set of supported MCUs and provides specific implementations for them.  
    e.g. `Stm32GpioPin::specific_function()` aswell as  
    the implementation for the HAL  
    e.g. `impl GpioPin for Stm32GpioPin { fn set_enabled() { ... }}`

# Module Architecture/Crate Organization
This illustrates the connections between the crates

```
[application XY] --> [board] (/board.rs)  
                        ---> [hal] (/hal)            ; use abstractions/traits in application
                        -
                        ---> [board_XY] (/boards/XY) ; board selection
                        ---> [board_ZZ] (/boards/ZZ) ; board selection
                                -
                                ---> [hal] (/hal)    ; implement general traits
                                ---> [mcu_stm32f0xx] (/mcu/stm32f2xx)
                                          ---> ... implementation details
```
