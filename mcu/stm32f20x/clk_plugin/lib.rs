#![feature(proc_macro, range_contains)]

extern crate proc_macro;

use proc_macro::{quote, Literal, TokenStream, TokenNode, Spacing, Term};

 // TODO: eventually move to another crate to reduce duplication
 // TODO: replace with constexpr/miri when available
mod common {
    pub const MAX_SYS_FREQ: u32  = 120000000;
    pub const MAX_AHB_FREQ: u32  = 120000000;
    pub const MAX_APB1_FREQ: u32 =  30000000;
    pub const MAX_APB2_FREQ: u32 =  60000000;
   
    #[derive(Debug)]
    pub enum ClockSource {
        Hsi,
        Hse,
    }

    #[derive(Debug)]
    pub enum HseTy {
        Crystal,
        External,
    }
}
use common::*;

fn extract_ident(t: &TokenNode) -> Term {
    match *t {
        TokenNode::Term(term) => term.clone(),
        _ => panic!("no identifier in: {:?}", t),
    }
}

fn extract_u32_literal(t: &TokenNode) -> u32 {
    let err = format!("no uint literal in: {:?}", t);
    match *t {
        TokenNode::Literal(ref lit) => format!("{}", lit).parse::<u32>().expect(&err),
        _ => panic!(err),
    }
}

macro_rules! expect_op {
    ($iter: expr, $op:pat, $block: expr) => {
        match $iter.next().unwrap().kind {
            TokenNode::Op($op, Spacing::Alone) => $block,
            _ => panic!("expected {} in clock setting value", stringify!($op)),
        }
    }
}

/// a list of settings that can be configured using the clk_config macro
/// used internally for parsing
enum ClkSetting {
    ClockSource,
    Hse,
    HseFreq,
    NoPll,
    PllInFreq
}

/// the current parsing state for the clk_config macro
struct ParsedClkSettings {
    clk_src: Option<ClockSource>,
    sys_freq: Option<u32>,
    ahb_freq: u32,
    apb1_freq: u32,
    apb2_freq: u32,
    hsi_freq: u32,
    hse: Option<HseTy>,
    hse_freq: u32,
    no_pll: bool,
    pllin_freq: u32,
    pllout_freq: Option<u32>,
}

#[proc_macro]
pub fn clk_config(ts: TokenStream) -> TokenStream {
    let mut iter = ts.into_iter();
    
    let mut cfg = ParsedClkSettings {
        clk_src: None,
        sys_freq: None,
        ahb_freq: MAX_AHB_FREQ,
        apb1_freq: MAX_APB1_FREQ,
        apb2_freq: MAX_APB2_FREQ,
        hsi_freq: 16000000,
        hse: None,
        hse_freq: 0,
        no_pll: false,
        pllin_freq: 2000000,
        pllout_freq: None,
    };

    // parse the settings we provide
    while let Some(next) = iter.next() {
        let setting = match extract_ident(&next.kind).as_str() {
            "ClockSource" => ClkSetting::ClockSource,
            "Hse" => ClkSetting::Hse,
            "HseFreq" => ClkSetting::HseFreq,
            "NoPll" => ClkSetting::NoPll,
            "PllInFreq" => ClkSetting::PllInFreq,
            ident => panic!("unexpected clock setting: {:?}", ident)
        };
        expect_op!(iter, '=', ());
        let val = iter.next().expect("expected setting value").kind;
        match setting {
            ClkSetting::ClockSource => {
                cfg.clk_src = Some(match &extract_ident(&val).as_str().to_lowercase()[..] {
                    "hsi" => ClockSource::Hsi,
                    "hse" => ClockSource::Hse,
                    x => panic!("unknown clock source {:?}", x),
                });
            }
            ClkSetting::Hse => {
                cfg.hse = Some(match &extract_ident(&val).as_str().to_lowercase()[..] {
                    "crystal" => HseTy::Crystal,
                    "external" => HseTy::External,
                    x => panic!("unknown hse ty: {:?}", x),
                });
            }
            ClkSetting::HseFreq => cfg.hse_freq = extract_u32_literal(&val),
            ClkSetting::NoPll => cfg.no_pll = true,
            ClkSetting::PllInFreq => cfg.pllin_freq = extract_u32_literal(&val),
        }
        expect_op!(iter, ',', ());
    }

    // calclulate some frequencies and validate that everything works out
    let pll_source_freq = match cfg.clk_src {
        Some(ClockSource::Hse) => {
            let range = 4000000..26000000;
            if !range.contains(cfg.hse_freq) {
                panic!("HSE frequency of {} not in allowed range of {:?}", cfg.hse_freq, range);
            }
            if cfg.hse.is_none() {
                panic!("HSE oscillator not configured");
            }
            // #define STM32_PLL_BYPASS CFGR.b.SW_HSE
            // #define STM32_PLL_SOURCE PLLCFGR.b.PLLSRC_HSE
            cfg.hse_freq
        }
        Some(ClockSource::Hsi) => {
            // #define STM32_PLL_BYPASS CFGR.b.SW_HSI
            // #define STM32_PLL_SOURCE PLLCFGR.b.PLLSRC_HSI
            cfg.hsi_freq
        }
        None => {
            panic!("no clock source configuration");
        }
    };

    let sys_freq = cfg.sys_freq.unwrap_or(cfg.ahb_freq);
    cfg.sys_freq = Some(sys_freq);

    let pll_block = if cfg.no_pll {
        // #define STM32_SYS_CLOCK STM32_PLL_SOURCE_FREQUENCY
        // #define STM32_SYS_SOURCE STM32_PLL_BYPASS
        unimplemented!()
    } else {
        // TODO: #define STM32_SYS_SOURCE CFGR.b.SW_PLL
        let range = 1000000..2000000;
        if !range.contains(cfg.pllin_freq) {
            panic!("PLL frequency is out of range {:?}", range);
        }
        let pllin_divider = pll_source_freq / cfg.pllin_freq;
        if cfg.pllin_freq * pllin_divider != pll_source_freq {
            panic!("can not derive PLL frequency from specified input clock");
        }
        
        if cfg.pllout_freq.is_none() {
            for i in &[2, 4, 6, 8] {
                let freq = i * sys_freq;
                if freq > 64000000 && freq <= 432000000 && freq % 48000000 == 0 {
                    cfg.pllout_freq = Some(freq);
                    break;
                }
            }
            cfg.pllout_freq = Some(cfg.pllout_freq.unwrap_or(0));
        }
        let pllout_range = 64000000..432000000;
        let pllout_freq = cfg.pllout_freq.unwrap();
        if !pllout_range.contains(pllout_freq) {
            panic!("PLL output frequency of {} is out of range {:?}", pllout_freq, pllout_range);
        }

        let pll_multiplier = pllout_freq / cfg.pllin_freq;
        if cfg.pllin_freq * pll_multiplier != pllout_freq {
            panic!("can not find a PLL multiplier for the given output frequency");
        }
        let pll_multiplier_range = 64..432;
        if !pll_multiplier_range.contains(pll_multiplier) {
            panic!("PLL multiplier of {} is out of range {:?}", pll_multiplier, pll_multiplier_range);
        }

        let sys_divider = pllout_freq / sys_freq / 2 - 1;
        if sys_freq * (sys_divider + 1) * 2 != pllout_freq || sys_divider > 3 {
            panic!("sysfreq can not be derived from given PLL output frequency");
        }
        
        let divider_48m = match (pllout_freq + 47999999) / 48000000 {
            0...1 => 2,
            x @ 2...15 => x,
            _ => 15,
        };

        if 48000000 * divider_48m != pllout_freq {
            panic!("a multiple of 48000000Hz is required for some peripherals!");
        }

        // prepare to output the !NO_PLL specific calculations
        let pllout_freq = TokenNode::Literal(Literal::u32(pllout_freq));
        let pllin_divider = TokenNode::Literal(Literal::u8(pllin_divider as u8));
        let pll_multiplier = TokenNode::Literal(Literal::u16(pll_multiplier as u16));
        let sys_divider = TokenNode::Literal(Literal::u8(sys_divider as u8));
        let divider_48m = TokenNode::Literal(Literal::u8(divider_48m as u8));
        quote!(
            no_pll: false,
            pllout_freq: $pllout_freq,
            pllin_divider: $pllin_divider,
            pll_multiplier: $pll_multiplier,
            sys_divider: $sys_divider,
            divider_48m: $divider_48m,
        )
    };

    // check that frequencies are in-bound
    match 1 {
        1 if     sys_freq > MAX_SYS_FREQ    => panic!("sysclk frequency too high"),
        1 if cfg.ahb_freq > MAX_AHB_FREQ    => panic!("ahb frequency too high"),
        1 if cfg.apb1_freq > MAX_APB1_FREQ  => panic!("apb1 frequency too high"),
        1 if cfg.apb2_freq > MAX_APB2_FREQ  => panic!("apb2 frequency too high"),
        _ => ()
    }

    fn find_divider<F: Fn(u32) -> bool>(factors: &[u32], start: u32, check: F) -> Option<u32> {
        for (i, factor) in factors.iter().enumerate() {
            if check(*factor) {
                return Some((i > 0) as u32 * start + i as u32);
            }
        }
        None
    }

    let ahb_divider = find_divider(&[1, 2, 4, 8, 16, 64, 128, 256, 512], 7, |x| cfg.ahb_freq * x == sys_freq)
        .expect("can not determine an appropriate AHB divider");
    let apb1_divider = find_divider(&[1, 2, 4, 8, 16], 3, |x| cfg.apb1_freq * x == cfg.ahb_freq)
        .expect("can not determine an appropriate APB1 divider");
    let apb2_divider = find_divider(&[1, 2, 4, 8, 16], 3, |x| cfg.apb2_freq * x == cfg.ahb_freq)
        .expect("can not determine an appropriate APB2 divider");

    // prepare the general calculated values to be returned
    let clk_src = TokenNode::Term(Term::intern(&format!("{:?}", cfg.clk_src.unwrap())));
    let hse = TokenNode::Term(Term::intern(&format!("{:?}", cfg.hse.unwrap())));
    let apb1_freq = TokenNode::Literal(Literal::u32(cfg.apb1_freq));
    let ahb_divider = TokenNode::Literal(Literal::u8(ahb_divider as u8));
    let apb1_divider = TokenNode::Literal(Literal::u8(apb1_divider as u8));
    let apb2_divider = TokenNode::Literal(Literal::u8(apb2_divider as u8));
    let body = quote!(
        static RCC_CONFIG: RccConfig = RccConfig {
            clk_src: ClockSource::$clk_src,
            hse: HseTy::$hse,
            apb1_freq: $apb1_freq,
            ahb_divider: $ahb_divider,
            apb1_divider: $apb1_divider,
            apb2_divider: $apb2_divider,
            $pll_block
        };
    );

    // TODO: avoid this reparsing hack (quote somehow can't handle use statements yet? [wrong AST representation Tree])
    let block = format!("use board::imp::rcc::{{ClockSource, RccConfig, HseTy}}; {}", body);
    block.parse().unwrap()
}
