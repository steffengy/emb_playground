use rcc::Clock;
use cortex_m::interrupt::CriticalSection;
use stm32f439x::{ethernet_mac, ETHERNET_DMA, ETHERNET_MAC, RCC, SYSCFG};

fn macmiiar_base(w: &mut ethernet_mac::macmiiar::W) -> &mut ethernet_mac::macmiiar::W {
    // TODO: rcc specific CR
    unsafe { w.bits(0).mb().set_bit().cr().bits(1) }
}

pub fn init(cs: &CriticalSection) {
    let eth_mac = ETHERNET_MAC.borrow(cs);
    let eth_dma = ETHERNET_DMA.borrow(cs);
    let rcc = RCC.borrow(cs);
    let syscfg = SYSCFG.borrow(cs);

    eth_mac.macmiiar.write(macmiiar_base);
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
    // TODO: move to SYSCFG::setMIIMode(type == Ethernet::MII::TYPE_RMII);
    {
        let _guard = Clock::SysCfg.enable_guarded(cs);
        syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());
    }

    Clock::EthMac.enable(cs);
    Clock::EthMacTx.enable(cs);
    Clock::EthMacRx.enable(cs);

    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());

    unsafe { eth_dma.dmabmr.write(|w| w.bits(0).sr().set_bit()); }
    while eth_dma.dmabmr.read().sr().bit_is_set() {}

    Clock::EthMac.disable(cs);
    Clock::EthMacTx.disable(cs);
    Clock::EthMacRx.disable(cs);

    // TODO: remove this from here
    lan8720_init(cs);
}

pub fn phy_write(cs: &CriticalSection, phy_id: u8, reg: u8, data: u16) {
    let eth_mac = ETHERNET_MAC.borrow(cs);

    unsafe {
        let _guard = Clock::EthMac.enable_guarded(cs);


        eth_mac.macmiidr.write(|w| w.bits(0).td().bits(data));

        eth_mac.macmiiar.write(|w| macmiiar_base(w)
                                      .pa().bits(phy_id)
                                      .mr().bits(reg)
                                      .mw().set_bit());

        while eth_mac.macmiiar.read().mb().bit_is_set() {}
    }
}

fn phy_read(cs: &CriticalSection, phy_id: u8, reg: u8) -> u16 {
    let eth_mac = ETHERNET_MAC.borrow(cs);

    unsafe {
        let _guard = Clock::EthMac.enable_guarded(cs);


        eth_mac.macmiiar.write(|w| macmiiar_base(w)
                                      .pa().bits(phy_id)
                                      .mr().bits(reg));


        while eth_mac.macmiiar.read().mb().bit_is_set() {}

        eth_mac.macmiidr.read().td().bits()
    }
}

// TODO: move me
/// Initializes the LAN8720 PHY
fn lan8720_init(cs: &CriticalSection) {
    const PHY_ID: u8 = 0;

    // wait 10ms until the PHY is ready to take commands
    // TODO: use proper timer (tim2 setup is still handled in the app code currently)
    let tim2 = ::stm32f439x::TIM2.borrow(cs);
    while tim2.cnt.read().bits() < 10000 {}

    // configure "All capable. Auto-negotiation enabled." mode
    phy_write(cs, PHY_ID, 0x12, 0x40e0);

    // perform a soft reset: "Bit is self-clearing"
    phy_write(cs, PHY_ID, 0x00, 0x8000);

    // Wait for the reset to be complete (bit is cleared)
    while {
        phy_read(cs, PHY_ID, 0) & 0x8000 > 0
    } {}

    // Enable the "Energy Detect Power-Down" mode
    // "In this mode, when no energy is present on the line the transceiver is powered down"
    phy_write(cs, PHY_ID, 0x11, 0x2000);

    // enable the auto-negotiation process
    phy_write(cs, PHY_ID, 0x00, 0x1000);

    // wait for link up (TODO this [and the missing part] should be moved into a "sense" function
    // that is periodically called to determine the current PHY link state and act accordingly)
    while phy_read(cs, PHY_ID, 0x01) & 0x0004 == 0 {}
}
