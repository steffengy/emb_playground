use cortex_m::interrupt::CriticalSection;
use stm32f439x::RCC;
use rcc::Clock;

// TODO: support bitbanding
macro_rules! special {
    ($( $name:ident = $val:expr ),*) => {
        #[allow(non_camel_case_types)]
        #[derive(Copy, Clone)]
        pub enum Special {
            $( $name ),*
        }

        impl Special {
            #[inline]
            pub fn alternate_fn(self) -> u32 {
                match self {
                    $( Special::$name => $val ),*
                }
            }
        }
    }
}

#[macro_export]
macro_rules! gpio_config {
    ( $( $gpio_reg:ident = [ $( $primary_alias:ident $(,$alias:ident)*: $pin:expr),* ] ),* ) => {
        fn setup_gpio(cs: &board::CriticalSection)
        {
            use board::imp::gpio::{Mode, Ty, Speed, Pull, InitCtl, InitVal, Special};

            // determine and write the configuration for each port
            $({
                let (mut mode, mut ty, mut speed, mut pull, mut init_ctl, mut init_val, mut special) =
                    (0u32, 0u32, 0u32, 0u32, 0u32, 0u32, 0u64);

                let mut i = 0;
                // determine the pin configuration. equal to an unrolled loop that gets optimized away by LLVM
                // in the future the MIR optimizations/constexpr should take care of this
                $({
                    mode |= ($pin.0 as u32) << (2 * i);
                    ty |= ($pin.1 as u32) << i;
                    speed |= ($pin.2 as u32) << (2 * i);
                    pull |= ($pin.3 as u32) << (2 * i);
                    init_ctl |= ($pin.4 as u32) << i;
                    init_val |= ($pin.5 as u32) << i;
                    special |= ($pin.6.alternate_fn() as u64) << (4 * i);
                    // This is unused for the last PIN, anyways optimized out
                    #[allow(unused_assignments)] {
                        i += 1;
                    }
                })*
                
                let gpior = board::imp::$gpio_reg.borrow(cs);

                Clock::$gpio_reg.enable(&cs);
                unsafe {
                    gpior.otyper.write(|w| w.bits(ty));
                    gpior.ospeedr.write(|w| w.bits(speed));
                    gpior.pupdr.write(|w| w.bits(pull));
                    gpior.bsrr.write(|w| w.bits(init_ctl << 16 | init_val));
                    gpior.afrh.write(|w| w.bits((special >> 32) as u32));
                    gpior.afrl.write(|w| w.bits(special as u32));
                    gpior.moder.write(|w| w.bits(mode));
                }
                Clock::$gpio_reg.disable(&cs);
            })*
        }

        #[allow(non_camel_case_types)]
        #[derive(Copy, Clone)]
        enum NativeGpioPin {
            $( $( $primary_alias, )* )*
        }

        #[allow(non_camel_case_types, dead_code)]
        #[derive(Copy, Clone)]
        enum GpioPin {
            $( $( $primary_alias, $( $alias, )* )* )*
        }

        // TODO: this should be a hal implementation, the native also fits good into that
        impl GpioPin {
            #[doc(hidden)]
            #[inline]
            fn native(&self) -> NativeGpioPin {
                match *self {
                    $( $( GpioPin::$primary_alias $( | GpioPin::$alias )* => NativeGpioPin::$primary_alias, )* )*
                }
            }

            #[inline]
            fn set_enabled(&self, cs: &::board::CriticalSection, enabled: bool) {
                match *self {
                    $(
                        $( x @ GpioPin::$primary_alias $( | x @ GpioPin::$alias )* )|* => {
                            // resolve the current PIN to the relative index of the port (e.g. B10 => 10)
                            let pin = x.native() as u32 - [$( NativeGpioPin::$primary_alias ),*][0] as u32;
                            Clock::$gpio_reg.enable(cs);
                            unsafe {
                                board::imp::$gpio_reg.borrow(cs).bsrr.write(|w| w.bits(((0x10000 | enabled as u32) << pin)));
                            }
                            Clock::$gpio_reg.disable(cs);
                        }
                    )*
                }
            }
        }
    }
}

#[derive(Copy, Clone)]
pub enum Mode {
    Input,
    Output,
    Special,
    Analog,
}

#[derive(Copy, Clone)]
pub enum Ty {
    PushPull,
    OpenDrain,
}

#[derive(Copy, Clone)]
pub enum Pull {
    None,
    Up,
    Down,
}

#[derive(Copy, Clone)]
pub enum InitCtl {
    Set,
    Skip,
}

#[derive(Copy, Clone)]
pub enum InitVal {
    Low,
    High
}

#[derive(Copy, Clone)]
pub enum Speed {
    _2MHZ,
    _25MHZ,
    _50MHZ,
    _100MHZ,
}

special! {
    SYS = 0,
    TIM1 = 1,
    TIM2 = 1,
    TIM3 = 2,
    TIM4 = 2,
    TIM5 = 2,
    TIM8 = 3,
    TIM9 = 3,
    TIM10 = 3,
    TIM11 = 3,
    I2C1 = 4,
    I2C2 = 4,
    I2C3 = 4,
    I2S2 = 5,
    SPI1 = 5,
    SPI2 = 5,
    SPI3_PD6 = 5,
    SPI4 = 5,
    SPI5 = 5,
    SPI3 = 6,
    I2S2EXT = 6,
    I2S3 = 6,
    SAI1 = 6,
    USART1 = 7,
    USART2 = 7,
    USART3 = 7,
    I2S3EXT = 7,
    UART4 = 8,
    UART5 = 8,
    USART6 = 8,
    UART7 = 8,
    UART8 = 8,
    CAN1 = 9,
    CAN2 = 9,
    TIM12 = 9,
    TIM13 = 9,
    TIM14 = 9,
    LCD_PB0 = 9,
    LCD_PB1 = 9,
    LCD_PG10 = 9,
    LCD_PG12 = 9,
    OTG_FS = 10,
    OTG_HS_ULPI = 10,
    ETH = 11,
    FMC = 12,
    FSMC = 12,
    SDIO = 12,
    OTG_HS = 12,
    DCMI = 13,
    LCD = 14,
    EVENTOUT = 15
}
