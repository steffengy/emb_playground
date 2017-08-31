//! This implements RCC functionality as supported by F4xxx chips,
//! which is a superset of the F2xxx (this) architecture

// TODO: use bit-banding

use cortex_m::interrupt::CriticalSection;
use stm32f439x::{FLASH, RCC};

pub struct ClockRestoreGuard(Option<Clock>, bool);

impl Drop for ClockRestoreGuard {
    fn drop(&mut self) {
        // A guard cannot escape the outer scope, 
        // since a critical section is used with closures
        // Therefore this guard can not escape the scope
        let sec = unsafe { CriticalSection::new() };

        match self.0 {
            Some(clock) if  self.1 => clock.enable(&sec),
            Some(clock) if !self.1 => clock.disable(&sec),
            _ => (),
        }
    }
}

macro_rules! clocks {
    ( $($name:ident => $bit:ident @ $reg:ident),* ) => {
        #[derive(Copy, Clone)]
        pub enum Clock {
            $( $name ),*
        }

        impl Clock {
            #[inline(always)]
            pub fn enable(&self, cs: &CriticalSection) {
    
                let rcc = RCC.borrow(cs);
                match *self {
                    $( Clock::$name => rcc.$reg.modify(|_, w| w.$bit().set_bit()) ),*
                }
                // STM32f2 Errata: 2.1.11 Delay after an RCC peripheral clock enabling
                unsafe { asm!("dsb" ::: "memory" : "volatile"); }
            }

            // enable the given clock and reset it to its old state
            // after the returned guard goes out of scope
            #[inline(always)]
            pub fn enable_guarded(&self, cs: &CriticalSection) -> ClockRestoreGuard {
    
                let rcc = RCC.borrow(cs);
                let mut old = false;
                match *self {$( 
                    Clock::$name => rcc.$reg.modify(|r, w| { 
                        old = r.$bit().bit_is_set(); 
                        w.$bit().set_bit() 
                    }) ),*
                }
                unsafe { asm!("dsb" ::: "memory" : "volatile"); }
                let clock = if !old { Some(self.clone()) } else { None };
                ClockRestoreGuard(clock, old)
            }

            #[inline(always)]
            pub fn disable(&self, cs: &CriticalSection) {
                let rcc = RCC.borrow(cs);
                match *self {
                    $( Clock::$name => rcc.$reg.modify(|_, w| w.$bit().clear_bit()) ),*
                }
            }
        }
    }
}

clocks! {
    GPIOA => gpioaen @ ahb1enr,
    GPIOB => gpioben @ ahb1enr,
    GPIOC => gpiocen @ ahb1enr,
    GPIOD => gpioden @ ahb1enr,
    GPIOE => gpioeen @ ahb1enr,
    GPIOF => gpiofen @ ahb1enr,
    GPIOG => gpiogen @ ahb1enr,
    GPIOH => gpiohen @ ahb1enr,
    GPIOI => gpioien @ ahb1enr,
    EthMac => ethmacen @ ahb1enr,
    EthMacTx => ethmactxen @ ahb1enr,
    EthMacRx => ethmacrxen @ ahb1enr,
    SysCfg => syscfgen @ apb2enr
}

#[derive(Eq, PartialEq)]
pub enum ClockSource {
    Hsi,
    Hse,
}

pub enum HseTy {
    Crystal,
    External,
}

pub struct RccConfig {
    pub clk_src: ClockSource,
    pub hse: HseTy,
    pub no_pll: bool,
    pub pllout_freq: u32,
    
    pub apb1_freq: u32,

    pub pllin_divider: u8,
    pub pll_multiplier: u16,
    pub sys_divider: u8,
    pub divider_48m: u8,

    pub ahb_divider: u8,
    pub apb1_divider: u8,
    pub apb2_divider: u8,
}

macro_rules! reset_rcc {
    ($rcc:expr, $($reg:ident),*) => (unsafe {
        $(
            $rcc.$reg.write(|w| w.bits(u32::max_value()));
            $rcc.$reg.write(|w| w.bits(0));
        )*
    })
}

pub fn init(cs: &CriticalSection, cfg: &RccConfig) {
    let rcc = RCC.borrow(cs);
    // reset all peripherals
    reset_rcc!(rcc, ahb1rstr, ahb2rstr, ahb3rstr, apb1rstr, apb2rstr);

    unsafe {
        // disable all RCC interrupts
        rcc.cir.write(|w| w.bits(0));

        // setup some sane defaults
        rcc.ahb1enr.write(|w| w.bits(0).ccmdataramen().set_bit());
        rcc.ahb1lpenr.write(|w| w.bits(0)
                                 .crclpen().set_bit()
                                 .dma1lpen().set_bit()
                                 .dma2lpen().set_bit()
                                 .dma2dlpen().set_bit()
                                 .ethmaclpen().set_bit()
                                 .ethmactxlpen().set_bit()
                                 .ethmacrxlpen().set_bit()
                                 .ethmacptplpen().set_bit()
                                 .otghslpen().set_bit()
                                 .otghsulpilpen().set_bit());
        rcc.ahb2enr.write(|w| w.bits(0));
        rcc.ahb2lpenr.write(|w| w.bits(0)
                                 .dcmilpen().set_bit()
                                 .cryplpen().set_bit()
                                 .hashlpen().set_bit()
                                 .rnglpen().set_bit()
                                 .otgfslpen().set_bit());
        rcc.ahb3enr.write(|w| w.bits(0));
        rcc.ahb3lpenr.write(|w| w.bits(0).fmclpen().set_bit());
        rcc.apb1enr.write(|w| w.bits(0));
        rcc.apb1lpenr.write(|w| w.bits(0)
                                 .tim2lpen().set_bit()
                                 .tim3lpen().set_bit()
                                 .tim4lpen().set_bit()
                                 .tim5lpen().set_bit()
                                 .tim6lpen().set_bit()
                                 .tim7lpen().set_bit()
                                 .tim12lpen().set_bit()
                                 .tim13lpen().set_bit()
                                 .tim14lpen().set_bit()
                                 .wwdglpen().set_bit()
                                 .spi2lpen().set_bit()
                                 .spi3lpen().set_bit()
                                 .usart2lpen().set_bit()
                                 .usart3lpen().set_bit()
                                 .uart4lpen().set_bit()
                                 .uart5lpen().set_bit()
                                 .i2c1lpen().set_bit()
                                 .i2c2lpen().set_bit()
                                 .i2c3lpen().set_bit()
                                 .can1lpen().set_bit()
                                 .can2lpen().set_bit()
                                 .daclpen().set_bit()
                                 .uart7lpen().set_bit()
                                 .uart8lpen().set_bit());
        rcc.apb2enr.write(|w| w.bits(0));
        rcc.apb2lpenr.write(|w| w.bits(0)
                                 .tim1lpen().set_bit()
                                 .tim8lpen().set_bit()
                                 .usart1lpen().set_bit()
                                 .usart6lpen().set_bit()
                                 .adc1lpen().set_bit()
                                 .adc2lpen().set_bit()
                                 .adc3lpen().set_bit()
                                 .sdiolpen().set_bit()
                                 .spi1lpen().set_bit()
                                 .spi4lpen().set_bit()
                                 .tim9lpen().set_bit()
                                 .tim10lpen().set_bit()
                                 .tim11lpen().set_bit()
                                 .spi5lpen().set_bit()
                                 .spi6lpen().set_bit()
                                 .sai1lpen().set_bit()
                                 .ltdclpen().set_bit());
        // switch to HSI
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        rcc.cfgr.write(|w| w.bits(0).sw().hsi());
        while !rcc.cfgr.read().sws().is_hsi() {}
        // power down HSI & PLL
        rcc.cr.write(|w| w.bits(0).hsion().set_bit().hsitrim().bits(16));
        if cfg.clk_src == ClockSource::Hse {
            rcc.cr.modify(|_, w| {
                w.hseon().set_bit();
                if let HseTy::External = cfg.hse {
                    w.hsebyp().set_bit();
                }
                w
            });
        }

        if !cfg.no_pll {
            // configure and start PLL
            // TODO: handle STM32_PLL_SOURCE & specific stuff (currently hardcoded)
            // STM32_SYS_CLOCK = STM32_AHB_CLOCK = STM32_MAX_AHB_CLOCK = 120000000
            // STM32_PLLOUT_CLOCK = 2 * STM32_SYS_CLOCK = 240000000
            // STM32_PLL_MULTIPLIER ((STM32_PLLOUT_CLOCK) / (STM32_PLLIN_CLOCK)) = 240000000 / 1000000 = 240
            // STM32_SYS_DIVIDER ((STM32_PLLOUT_CLOCK) / (STM32_SYS_CLOCK) / 2 - 1) = 240000000 / 120000000 / 2 - 1 = 0
            rcc.pllcfgr.write(|w| {
                w.bits(0);
                if cfg.clk_src == ClockSource::Hse {
                    w.pllsrc().set_bit();
                }
                w.pllm().bits(cfg.pllin_divider)
                .plln().bits(cfg.pll_multiplier)
                .pllp().bits(cfg.sys_divider)
                .pllq().bits(cfg.divider_48m)
            });


            rcc.cr.modify(|_, w| w.pllon().set_bit());
        }

        // TODO: move to Flash::Init
        {
            let flash = FLASH.borrow(cs);
            // TODO: STM32_FLASH_SPEED = 30000000, STM32_FLASH_WAITSTATES = (STM32_AHB_CLOCK - 1) / STM32_FLASH_SPEED = 3
            flash.acr.write(|w| w.bits(0).dcen().set_bit().icen().set_bit().latency().bits(3));
        }

        // configure system clocks & prescalers
        
        rcc.cfgr.modify(|_, w| w.sw().pll() // TODO PLL
                                .hpre().bits(cfg.ahb_divider)
                                .ppre1().bits(cfg.apb1_divider)
                                .ppre2().bits(cfg.apb2_divider));

        // shutdown HSI
        if cfg.clk_src != ClockSource::Hsi {
            while rcc.cfgr.read().sws().is_hsi() {}
            rcc.cr.modify(|_, w| w.hsion().clear_bit());
        }
    }
}
