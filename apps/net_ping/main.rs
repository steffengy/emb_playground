//! This exposes a network stack that sends a UDP packet and can be pinged
//! TODO: some of this netstack needs to be refactored & moved into the stm32f2xxx crate
//! TODO: support dynamic allocation in transmit & receive for sizes bigger than one DMA buffer
//! TODO: interrupt driven
#![feature(asm, use_extern_macros)]
#![no_std]

use core::slice;
use core::mem;

extern crate board;
extern crate smoltcp;

use board::imp::{gpio_config, clk_config, interrupt};
use board::imp::rcc::Clock;

use smoltcp::Error;
use smoltcp::wire::EthernetAddress;
use smoltcp::iface::{ArpCache, SliceArpCache, EthernetInterface};
use smoltcp::phy::{Checksum, ChecksumCaps, Device, DeviceCapabilities};
use smoltcp::socket::{AsSocket, UdpSocket, UdpSocketBuffer, UdpPacketBuffer, SocketSet};
use smoltcp::wire::{IpAddress, Ipv4Address, IpEndpoint};

gpio_config! {
    GPIOA = [
        A0,  LCD_DC:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        A1,  ETH_RMII_REF_CLK:  (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH),
        A2,  ETH_MDIO:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::Low,  Special::ETH),
        A3,  LCD_nCS:           (Mode::Output,  Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SYS),
        A4,  ST0_ISET:          (Mode::Analog,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        A5,  ST1_ISET:          (Mode::Analog,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        A6,  ST0_FLOAT:         (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM13),
        A7,  ETH_RMII_CRS_DV:   (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH),
        A8,  DMX0_DE:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        A9,  DMX0_TX:           (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::High, Special::USART1),
        A10, DMX0_RX:           (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::Low,  Special::USART1),
        A11, OTG_FS_DM:         (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::OTG_FS),
        A12, OTG_FS_DP:         (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::OTG_FS),
        A13, SWDIO:             (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        A14, SWCLK:             (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        A15, SD_nCS:            (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SYS)
    ],
    GPIOB = [
        B0,  ST1_FLOAT:         (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM3),
        B1,  ST1_nERR:          (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::High, Special::SYS),
        B2,  ST1_nRST:          (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::High, Special::SYS),
        B3,  SPI_SCK:           (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::SPI1),
        B4,  SPI_MISO:          (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SPI1),
        B5,  SPI_MOSI:          (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SPI1),
        B6,  Flash_nCS:         (Mode::Output,  Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SYS),
        B7,  NRF_nIRQ:          (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::High, Special::SYS),
        B8,  ST0_STEP:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM10),
        B9,  ST1_STEP:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM11),
        B10, SD_SCK:            (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::SPI2),
        B11, ETH_RMII_TX_EN:    (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH),
        B12, ETH_RMII_TXD0:     (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH),
        B13, ETH_RMII_TXD1:     (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH),
        B14, OTG_HS_DM:         (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::OTG_HS),
        B15, OTG_HS_DP:         (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::OTG_HS)
    ],
    GPIOC = [
        C0,  LED_R:             (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS), 
        C1,  ETH_MDC:           (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH),
        C2,  SD_MISO:           (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SPI2), 
        C3,  SD_MOSI:           (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SPI2), 
        C4,  ETH_RMII_RXD0:     (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH), 
        C5,  ETH_RMII_RXD1:     (Mode::Special, Ty::PushPull, Speed::_50MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::ETH), 
        C6,  WS2811_0:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM8), 
        C7,  WS2811_1:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM8), 
        C8,  WS2811_2:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM8), 
        C9,  WS2811_3:          (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM8), 
        C10, NRF_SCK:           (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::SPI3), 
        C11, NRF_MISO:          (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SPI3), 
        C12, NRF_MOSI:          (Mode::Special, Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SPI3), 
        C13, LED_G:             (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS), 
        C14, OSC32_IN:          (Mode::Analog,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS), 
        C15, OSC32_OUT:         (Mode::Analog,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS)
    ],
    GPIOD = [
        D0,  CAN_RX:            (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::Low,  Special::CAN1),
        D1,  CAN_TX:            (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::CAN1),
        D2,  LED_Y:             (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D3,  LED_W:             (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D4,  DMX1_DE:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D5,  DMX1_TX:           (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::High, Special::USART2),
        D6,  DMX1_RX:           (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::Low,  Special::USART2),
        D7,  LED_B:             (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D8,  ST0_nERR:          (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Up,   InitCtl::Set, InitVal::High, Special::SYS),
        D9,  ST1_LED:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D10, ST0_LED:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D11, SD_LED:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        D12, LED_PWM_0:         (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM4),
        D13, LED_PWM_1:         (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM4),
        D14, LED_PWM_2:         (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM4),
        D15, LED_PWM_3:         (Mode::Special, Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::TIM4)
    ],
    GPIOE = [
        E0,  NRF_nCS:           (Mode::Output,  Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::High, Special::SYS),
        E1,  NRF_CE:            (Mode::Output,  Ty::PushPull, Speed::_25MHZ,  Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E2,  LCD_nBL:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::High, Special::SYS),
        E3,  BTN_RIGHT:         (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Down, InitCtl::Set, InitVal::Low,  Special::SYS),
        E4,  BTN_UP:            (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Down, InitCtl::Set, InitVal::Low,  Special::SYS),
        E5,  BTN_DOWN:          (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Down, InitCtl::Set, InitVal::Low,  Special::SYS),
        E6,  BTN_LEFT:          (Mode::Input,   Ty::PushPull, Speed::_2MHZ,   Pull::Down, InitCtl::Set, InitVal::Low,  Special::SYS),
        E7,  ST0_nRST:          (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::High, Special::SYS),
        E8,  ST1_M0:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E9,  ST1_M1:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E10, ST1_M2:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E11, ST1_DIR:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E12, ST0_M0:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E13, ST0_M1:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E14, ST0_M2:            (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS),
        E15, ST0_DIR:           (Mode::Output,  Ty::PushPull, Speed::_2MHZ,   Pull::None, InitCtl::Set, InitVal::Low,  Special::SYS)
    ]
}

clk_config! {
    ClockSource = Hse,
    Hse = Crystal,
    HseFreq = 25000000,
    PllInFreq = 1000000,
}

#[repr(packed)]
#[derive(Copy, Clone)]
struct TxDescEnhanced {
    flags1: u32,
    flags2: u32,
    addr: *const u8,
    next_descriptor: *const TxDescEnhanced,
    reserved: u64,
    tts: u64,
}

enum ChecksumInsertCtrl {
    None,
    HeaderCalc,
    HeaderAndPayload,
    HeaderAndPayloadCalc,
}

impl TxDescEnhanced {
    #[inline]
    fn set_fs(&mut self, val: bool) -> &mut Self {
        self.flags1 &= !(1 << 28);
        self.flags1 |= (val as u32) << 28;
        self
    }

    #[inline]
    fn set_ls(&mut self, val: bool) -> &mut Self {
        self.flags1 &= !(1 << 29);
        self.flags1 |= (val as u32) << 29;
        self
    }

    #[inline]
    fn set_owned(&mut self, owned: bool) -> &mut Self {
        unsafe {
            ::core::ptr::write_volatile(&mut self.flags1, (self.flags1 & !(1 << 31)) | (owned as u32) << 31);
        }
        self
    }

    /// Returns whether this descriptor is currently owned by the DMA
    #[inline]
    fn is_owned(&self) -> bool {
        unsafe {
            core::ptr::read_volatile(&self.flags1) & (1 << 31) > 0
        }
    }

    #[inline]
    fn set_tch(&mut self, chained: bool) -> &mut Self {
        self.flags1 &= !(1 << 20);
        self.flags1 |= (chained as u32) << 20;
        self
    }

    #[inline]
    fn set_cic(&mut self, val: ChecksumInsertCtrl) -> &mut Self {
        self.flags1 &= !(1 << 22 | 1 << 23);
        self.flags1 |= (val as u32) << 22;
        self
    }

    #[inline]
    fn set_tx_buf(&mut self, addr: &[u8]) -> &mut Self {
        self.addr = addr.as_ptr();
        self.set_tx_buf_len(addr.len())
    }

    #[inline]
    fn set_tx_buf_len(&mut self, len: usize) -> &mut Self {
        self.flags2 &= !((1 << 13) - 1);
        self.flags2 |= len as u32;
        self
    }

    #[inline]
    fn tx_buf_len(&self) -> usize {
        (self.flags2 & (1 << 13) - 1) as usize
    }

    #[inline]
    unsafe fn tx_buf(&self) -> &[u8] {
        slice::from_raw_parts(self.addr as *mut _, self.tx_buf_len())
    }

    #[inline]
    unsafe fn tx_buf_mut(&self) -> &mut [u8] {
        slice::from_raw_parts_mut(self.addr as *mut _, self.tx_buf_len())
    }

    #[inline]
    fn set_next_descr(&mut self, descr: *const TxDescEnhanced) -> &mut Self {
        self.next_descriptor = descr;
        self
    }
}

// TODO: volatile own bit
#[repr(packed)]
#[derive(Copy, Clone)]
struct RxDescEnhanced {
    flags1: u32,
    flags2: u32,
    addr: *const u8,
    next_descriptor: *const RxDescEnhanced,
    extended_status: u32,
    reserved: u32,
    tts: u64,
}

impl RxDescEnhanced {
    #[inline]
    fn set_owned(&mut self, owned: bool) -> &mut Self {
        self.flags1 &= !(1 << 31);
        self.flags1 |= (owned as u32) << 31;
        self
    }

    /// Returns whether this descriptor is currently owned by the DMA
    #[inline]
    fn is_owned(&self) -> bool {
        self.flags1 & (1 << 31) > 0
    }

    #[inline]
    fn is_fs(&self) -> bool {
        self.flags1 & (1 << 8) > 0
    }

    #[inline]
    fn is_ls(&self) -> bool {
        self.flags1 & (1 << 9) > 0
    }

    #[inline]
    fn set_rch(&mut self, chained: bool) -> &mut Self {
        self.flags2 &= !(1 << 14);
        self.flags2 |= (chained as u32) << 14;
        self
    }

    #[inline]
    fn set_rx_buf(&mut self, addr: &[u8]) -> &mut Self {
        self.addr = addr.as_ptr();
        self.flags2 &= !((1 << 13) - 1);
        self.flags2 |= addr.len() as u32;
        self
    }

    #[inline]
    fn fl(&self) -> usize {
        ((self.flags1 >> 16) & (1 << 13) - 1) as usize - 4
    }

    /// Get the rx_buffer based on the set address and the length set in FL
    #[inline]
    unsafe fn rx_buf(&self) -> &[u8] {
        slice::from_raw_parts(self.addr as *mut _, self.fl())
    }

    #[inline]
    unsafe fn rx_buf_mut(&self) -> &mut [u8] {
        slice::from_raw_parts_mut(self.addr as *mut _, self.fl())
    }

    #[inline]
    fn set_next_descr(&mut self, descr: *const RxDescEnhanced) -> &mut Self {
        self.next_descriptor = descr;
        self
    }
}

const NUM_TX_BUFS: usize = 16;
const NUM_RX_BUFS: usize = 16;
const BUFFER_SIZE: usize = 384;

struct EthernetDevice {
    tx_bufs: [[u8; BUFFER_SIZE]; NUM_TX_BUFS],
    tx_descs: [TxDescEnhanced; NUM_TX_BUFS],
    tx_next: usize,
    rx_bufs: [[u8; BUFFER_SIZE]; NUM_RX_BUFS],
    rx_descs: [RxDescEnhanced; NUM_RX_BUFS],
    rx_next: usize,
}

impl EthernetDevice {
    fn new() -> EthernetDevice {
        EthernetDevice {
            tx_bufs:  [[0u8; BUFFER_SIZE]; NUM_TX_BUFS],
            tx_descs: [unsafe { mem::zeroed() }; NUM_TX_BUFS],
            tx_next: 0,
            rx_bufs:  [[0u8; BUFFER_SIZE]; NUM_RX_BUFS],
            rx_descs: [unsafe { mem::zeroed() }; NUM_RX_BUFS],
            rx_next: 0,
        }
    }

    fn init(&mut self, mac_addr: EthernetAddress) {
        // configure TX buffers in TX descriptor
        for i in 0..self.tx_descs.len() {
            let next_descr = self.tx_descs.get(i + 1).unwrap_or(&self.tx_descs[0]) as *const _;
            self.tx_descs[i]
                .set_tch(true)
                .set_cic(ChecksumInsertCtrl::HeaderAndPayloadCalc)
                .set_tx_buf(&self.tx_bufs[i])
                .set_next_descr(next_descr);
        }
        // configure RX buffers in RX descriptor
        for i in 0..self.rx_descs.len() {
            let next_descr = self.rx_descs.get(i + 1).unwrap_or(&self.rx_descs[0]) as *const _;
            self.rx_descs[i]
                .set_rch(true)
                .set_rx_buf(&self.rx_bufs[i])
                .set_next_descr(next_descr)
                .set_owned(true);
        }

        let cs = &unsafe { board::CriticalSection::new() };
        board::interrupt::free(|cs| unsafe {
            let eth_mac = board::imp::ETHERNET_MAC.borrow(cs);
            let eth_dma = board::imp::ETHERNET_DMA.borrow(cs);

            Clock::EthMac.enable(cs);

            eth_mac.maccr.write(|w| w.bits(0)
                                     .fes().set_bit()
                                     .rod().set_bit()
                                     .ipco().set_bit()
                                     .dm().set_bit());
            eth_mac.macffr.write(|w| w.bits(0).pcf().set_bit()); //TODO: PCF are 2 bits
            eth_mac.macfcr.write(|w| w.bits(0)
                                    .rfce().set_bit()
                                    .tfce().set_bit()
                                    .pt().bits(0x100));
            eth_dma.dmaomr.write(|w| w.bits(0)
                                    .rsf().set_bit()
                                    .tsf().set_bit()
                                    .osf().set_bit());
            eth_dma.dmabmr.write(|w| w.bits(0)
                                    .aab().set_bit()
                                    .fb().set_bit()
                                    .rtpr().bits(1)
                                    .pbl().bits(2)
                                    .edfe().set_bit());
            let mac_addr = mac_addr.as_bytes();
            eth_mac.maca0hr.write(|w| w.maca0h().bits((mac_addr[5] as u16) <<  8 |  mac_addr[4] as u16));
            eth_mac.maca0lr.write(|w| w.maca0l().bits((mac_addr[3] as u32) << 24 | (mac_addr[2] as u32) << 16 | 
                                                      (mac_addr[1] as u32) <<  8 |  mac_addr[0] as u32));

            // pass descriptors to hardware
            eth_dma.dmatdlar.write(|w| w.stl().bits(self.tx_descs.as_ptr() as u32));
            eth_dma.dmardlar.write(|w| w.srl().bits(self.rx_descs.as_ptr() as u32));
            
            Clock::EthMacTx.enable(cs);
            Clock::EthMacRx.enable(cs);

            eth_dma.dmaomr.modify(|_, w| w.ftf().set_bit());
            while eth_dma.dmaomr.read().ftf().bit_is_set() {}

            eth_mac.maccr.modify(|_, w| w.te().set_bit()
                                         .re().set_bit());
        
            eth_dma.dmaomr.modify(|_, w| w.st().set_bit()
                                          .sr().set_bit());
            eth_dma.dmaier.modify(|_, w| w.nise().set_bit()
                                          .aise().set_bit()
                                          .rbuie().set_bit()
                                          .rie().set_bit());

            Clock::EthMac.disable(cs);
            Clock::EthMacTx.disable(cs);
            Clock::EthMacRx.disable(cs);

            // enable ETH interrupt
            let nvic = ::board::imp::NVIC.borrow(cs);
            nvic.enable(::board::imp::Interrupt::ETH);

            // TODO: EthMac::Update, for simplicity currently here:
            let eth_mac = board::imp::ETHERNET_MAC.borrow(cs);
            Clock::EthMac.enable(cs);

            eth_mac.maccr.write(|w| w.bits(0)
                                     .fes().set_bit()
                                     .rod().set_bit()
                                     .ipco().set_bit()
                                     .dm().set_bit()
                                     .re().set_bit()
                                     .te().set_bit());
            Clock::EthMacTx.enable(cs);
            Clock::EthMacRx.enable(cs);
        });
    }
}

struct TxBuffer(*mut TxDescEnhanced);

impl AsRef<[u8]> for TxBuffer {
    fn as_ref(&self) -> &[u8] {
        unsafe {
            (*self.0).tx_buf()
        }
    }
}

impl AsMut<[u8]> for TxBuffer {
    fn as_mut(&mut self) -> &mut [u8] {
        unsafe {
            (*self.0).tx_buf_mut()
        }
    }
}

impl Drop for TxBuffer {
    fn drop(&mut self) {
        let desc = unsafe { &mut *self.0 };

        board::interrupt::free(|cs| {
            desc.set_fs(true).set_ls(true).set_owned(true);

            // DMA: trigger a poll of transmit descriptors
            let eth_dma = board::imp::ETHERNET_DMA.borrow(cs);
            unsafe {
                eth_dma.dmatpdr.write(|w| w.bits(0));
            }
        });
    }
}

struct RxBuffer(*mut RxDescEnhanced);

impl AsRef<[u8]> for RxBuffer {
    fn as_ref(&self) -> &[u8] {
        unsafe {
            (*self.0).rx_buf()
        }
    }
}

impl AsMut<[u8]> for RxBuffer {
    fn as_mut(&mut self) -> &mut [u8] {
        unsafe {
            (*self.0).rx_buf_mut()
        }
    }
}

impl Drop for RxBuffer {
    fn drop(&mut self) {
        let desc = unsafe { &mut *self.0 };
        // return the rx buffer back to the DMA after it is not used by us anymore
        desc.set_owned(true);
    }
}

impl Device for EthernetDevice {
    type RxBuffer = RxBuffer;
    type TxBuffer = TxBuffer;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut limits = DeviceCapabilities::default();
        limits.max_transmission_unit = 576;
        limits.max_burst_size = Some(BUFFER_SIZE);

        // enable hardware based checksums
        let mut checksum = ChecksumCaps::default();
        checksum.ipv4 = Checksum::None;
        checksum.udp = Checksum::None;
        checksum.tcp = Checksum::None;
        checksum.icmpv4 = Checksum::None;
        limits.checksum = checksum;
        limits
    }

    fn receive(&mut self, _timestamp: u64) -> Result<Self::RxBuffer, Error> {
        let idx = self.rx_next;
        let rx_desc = &mut self.rx_descs[idx];

        // check if we own the next buffer (the DMA is done with it)
        if rx_desc.is_owned() {
            return Err(Error::Exhausted)
        }
        let buf = RxBuffer(rx_desc as *mut _);
        self.rx_next = (self.rx_next + 1) % self.rx_bufs.len();

        if !rx_desc.is_fs() || !rx_desc.is_ls() {
            // TODO: how to receive if > rxbuf.len()? even heap allocated impossible?
            return Err(Error::Exhausted);
        }

        Ok(buf)
    }

    fn transmit(&mut self, _timestamp: u64, length: usize) -> Result<Self::TxBuffer, Error> {
        // check if the required descriptors are owned
        let idx = self.tx_next;
        let desc = &mut self.tx_descs[idx];
        let buf = &self.tx_bufs[idx];
        if desc.is_owned() {
            return Err(Error::Exhausted);
        }

        assert!(length <= buf.len());
        self.tx_next += 1;
        desc.set_tx_buf(&buf[..length]);
        Ok(TxBuffer(desc as *mut _))
    }
}

interrupt!(ETH, irq_eth);

fn irq_eth() {
    unsafe {
        let eth_dma = &*board::imp::ETHERNET_DMA.get();

        // mark the interrupt cause as "resolved" (tell that we handled all causes)
        eth_dma.dmasr.write(|w| w.bits(0).nis().set_bit()
                                  .ais().set_bit()
                                  .rbus().set_bit()
                                  .rs().set_bit());

        //eth_dma.dmarpdr.write(|w| w.bits(0));
    }
}

pub fn main() {
    let cs = &unsafe { board::CriticalSection::new() };
    board::interrupt::free(|cs| {
        board::imp::rcc::init(cs, &RCC_CONFIG);
        setup_gpio(cs);
        unsafe {
            // TODO: this sets up TIM2 to provide a ms-tick 
            //       (move to ethmac/replace with other sysclock setup, required for lan8720)
            let rcc = board::imp::RCC.borrow(cs);
            let eth_mac = board::imp::ETHERNET_MAC.borrow(cs);

            rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
            let tim2 = board::imp::TIM2.borrow(cs);
            tim2.psc.write(|w| w.bits(0).psc().bits((RCC_CONFIG.apb1_freq * 2 / 1000000 - 1) as u16));
            tim2.arr.write(|w| w.bits(0xffffffff));
            tim2.egr.write(|w| w.bits(0).ug().set_bit());
            tim2.cr1.write(|w| w.bits(0).cen().set_bit());
        }
        board::imp::ethmac::init(cs);
    });

    // setup TCP stack for local test case
    let mut protocol_addrs = [IpAddress::v4(192, 168, 0, 177)];
    let mut arp_cache_entries: [_; 8] = Default::default();
    let mut arp_cache = SliceArpCache::new(&mut arp_cache_entries[..]);
    let mut device = EthernetDevice::new();
    let hardware_addr = EthernetAddress([0x10, 0xE2, 0xD5, 0x00, 0x03, 0x00]); // TODO: use real MAC
    device.init(hardware_addr);

    let mut iface = EthernetInterface::new(&mut device, &mut arp_cache as &mut ArpCache,
                                           hardware_addr, &mut protocol_addrs[..]);


    // setup tx & rx buffers to use a smoltcp UDP socket
    let mut rx_data: [u8; 1024] = [0; 1024];
    let mut tx_data: [u8; 1024] = [0; 1024];
    let mut udp_packet_rx = [UdpPacketBuffer::new(&mut rx_data[..])];
    let mut udp_packet_tx = [UdpPacketBuffer::new(&mut tx_data[..])];
    let udp_rx_buffer = UdpSocketBuffer::new(&mut udp_packet_rx[..]);
    let udp_tx_buffer = UdpSocketBuffer::new(&mut udp_packet_tx[..]);
    let socket = UdpSocket::new(udp_rx_buffer, udp_tx_buffer);

    // get a socket set so we can poll and trigger the transmission
    let mut sockets: [_; 1] = Default::default();
    let mut socket_set = SocketSet::new(&mut sockets[..]);
    let socket_handle = socket_set.add(socket);
    {
        let udp_socket: &mut UdpSocket = socket_set.get_mut(socket_handle).as_socket();
        let endpoint = IpEndpoint { addr: IpAddress::Ipv4(Ipv4Address([255; 4])), port: 6666 };
        udp_socket.bind(IpEndpoint { addr: iface.protocol_addrs()[0], port: 6666 }).unwrap();
        udp_socket.send_slice(&[0xde, 0xad, 0xbe, 0xef], endpoint).unwrap();
    }

    // loop and poll and blink (TODO: interrupt triggered)
    let mut prev = false;
    loop {
        GpioPin::LED_Y.set_enabled(&cs, !prev);
        prev = !prev;
        
        let ret = iface.poll(&mut socket_set, 666);
        match ret {
            Ok(_) | Err(Error::Exhausted) | Err(Error::Unrecognized) => (),
            Err(e) => panic!("poll error: {}", e),
        }
    }
}
