#![no_std]

pub extern crate hal;

#[cfg(feature="board_flexperiment_mini")]
#[macro_reexport(gpio_config)]
pub extern crate board_flexperiment_mini as imp;

#[cfg(feature="board_flexperiment_nano")]
pub extern crate board_flexperiment_nano as imp;

#[cfg(feature="board_netboard")]
pub extern crate board_netboard as imp;

// TODO: move to HAL
pub use imp::interrupt::{self, CriticalSection};
