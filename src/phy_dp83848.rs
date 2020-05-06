use core::option::Option;

#[cfg(feature = "stm32f107")]
use stm32f1::stm32f107::ethernet_mac::{MACMIIAR, MACMIIDR};
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::stm32::ethernet_mac::{MACMIIAR, MACMIIDR};

use crate::smi::SMI;

#[allow(dead_code)]
mod consts {
    // Some of these constants seems to be common to all Phy:s?
    pub const PHY_REG_BMCR: u8 = 0x00; // Basic mode control register
    pub const PHY_REG_BMSR: u8 = 0x01; // Basic mode status register
    pub const PHY_REG_PHYIDR1: u8 = 0x02; // PHY Identifier Register #1
    pub const PHY_REG_PHYIDR2: u8 = 0x03; // PHY Identifier Register #2
    pub const PHY_REG_ANAR: u8 = 0x04; // Auto-Negotiation Advertisement Register
    pub const PHY_REG_ANLPAR: u8 = 0x05; // Auto-Negotiation Link Partner Ability Register (Base Page)
    pub const PHY_REG_ANER: u8 = 0x06; // Auto-Negotiation Expansion Register
    pub const PHY_REG_ANNPTR: u8 = 0x07; // Auto-Negotiation Next Page TX
    pub const PHY_REG_PHYSTS: u8 = 0x10; // PHY Status Register
    pub const PHY_REG_FCSCR: u8 = 0x14; // False Carrier Sense Counter Register
    pub const PHY_REG_RECR: u8 = 0x15; // Receive Error Counter Register
    pub const PHY_REG_PCSR: u8 = 0x16; // PCS Sub-Layer Configuration and Status Register
    pub const PHY_REG_RBR: u8 = 0x17; // RMII and Bypass Register
    pub const PHY_REG_LEDCR: u8 = 0x18; // LED Direct Control Register
    pub const PHY_REG_PHYCR: u8 = 0x19; // PHY Control Register
    pub const PHY_REG_10BTSCR: u8 = 0x1A; // 10Base-T Status/Control Register
    pub const PHY_REG_CDCTRL1: u8 = 0x1B; // CD Test Control Register and BIST Extensions Register
    pub const PHY_REG_EDCR: u8 = 0x1D; // Energy Detect Control Register

    pub const PHY_REG_BMCR_RESET: u16 = 1 << 15; // 1 = reset, self-clearing upon completed reset
    pub const PHY_REG_BMCR_LOOPBACK: u16 = 1 << 14;
    pub const PHY_REG_BMCR_SPEED_SELECTION: u16 = 1 << 13; // 1 = 100Mb/s, 0 = 10 Mb/s
    pub const PHY_REG_BMCR_AUTO_NEGOTIATION: u16 = 1 << 12; // 1 = on, disables speed and duplex selection
    pub const PHY_REG_BMCR_POWER_DOWN: u16 = 1 << 11; // 1 = power down
    pub const PHY_REG_BMCR_ISOLATE: u16 = 1 << 10; // 1 = isolate power from MII
    pub const PHY_REG_BMCR_RESTART_AUTONEG: u16 = 1 << 9; // 1 = restart auto-negotiation, self-clearing
    pub const PHY_REG_BMCR_DUPLEX_MODE: u16 = 1 << 8; // 1 = full duplex
    pub const PHY_REG_BMCR_COLLISION_TEST: u16 = 1 << 7; // 1 = collision test enabled

    pub const PHY_REG_BMSR_100_FULL: u16 = 1 << 14; // 1 = Device able to perform 100BASE-TX in full duplex mode
    pub const PHY_REG_BMSR_100_HALF: u16 = 1 << 13; // 1 = Device able to perform 100BASE-TX in half duplex mode
    pub const PHY_REG_BMSR_10_FULL: u16 = 1 << 12; // 1 = Device able to perform 10BASE-T in full duplex mode
    pub const PHY_REG_BMSR_10_HALF: u16 = 1 << 11; // 1 = Device able to perform 10BASE-T in half duplex mode

    pub const PHY_REG_BMSR_AUTONEG_COMPLETE: u16 = 1 << 5; // 1 = complete
    pub const PHY_REG_BMSR_REMOTE_FAULT: u16 = 1 << 4; // 1 = Remote Fault condition detected
    pub const PHY_REG_BMSR_LINK_STATUS: u16 = 1 << 2; // 1 = valid link
}

use self::consts::*;

pub struct Phy<'a> {
    smi: SMI<'a>,
    phy: u8,
}

impl<'a> Phy<'a> {
    /// Allocate
    pub fn new(macmiiar: &'a MACMIIAR, macmiidr: &'a MACMIIDR, phy: u8) -> Self {
        let smi = SMI::new(macmiiar, macmiidr);

        Phy { smi, phy }
    }

    /// Read current status registers
    ///
    /// You may keep the returned [`PhyStatus`](struct.PhyStatus.html)
    /// to compare it with to a future [`status()`](#method.status).
    pub fn status(&self) -> PhyStatus {
        PhyStatus {
            bmsr: self.smi.read(self.phy, PHY_REG_BMSR),
        }
    }

    /// Reset the PHY
    pub fn reset(&self) -> &Self {
        self.smi
            .set_bits(self.phy, PHY_REG_BMCR, PHY_REG_BMCR_RESET);

        // wait until reset bit is cleared by phy
        while (self.smi.read(self.phy, PHY_REG_BMCR) & PHY_REG_BMCR_RESET) == PHY_REG_BMCR_RESET {}

        self
    }

    /// Enable 10/100 Mbps half/full-duplex auto-negotiation
    pub fn set_autoneg(&self) -> &Self {
        self.smi
            .set_bits(self.phy, PHY_REG_BMCR, PHY_REG_BMCR_AUTO_NEGOTIATION);
        // wait until auto-neg complete bit is set by phy
        while (self.smi.read(self.phy, PHY_REG_BMSR) & PHY_REG_BMSR_AUTONEG_COMPLETE)
            == PHY_REG_BMSR_AUTONEG_COMPLETE
        {}

        self
    }
}

/// PHY status register
#[derive(Copy, Clone)]
pub struct PhyStatus {
    bmsr: u16,
}

impl PhyStatus {
    /// Has link?
    pub fn link_detected(&self) -> bool {
        (self.bmsr & PHY_REG_BMSR_LINK_STATUS) == PHY_REG_BMSR_LINK_STATUS
    }

    /// Has auto-negotiated?
    pub fn autoneg_done(&self) -> bool {
        (self.bmsr & PHY_REG_BMSR_AUTONEG_COMPLETE) == PHY_REG_BMSR_AUTONEG_COMPLETE
    }

    /// FD, not HD?
    pub fn is_full_duplex(&self) -> Option<bool> {
        if (self.bmsr & PHY_REG_BMSR_LINK_STATUS) != PHY_REG_BMSR_LINK_STATUS {
            return None;
        }
        if (self.bmsr & PHY_REG_BMSR_100_FULL) == PHY_REG_BMSR_100_FULL
            || (self.bmsr & PHY_REG_BMSR_10_FULL) == PHY_REG_BMSR_10_FULL
        {
            return Some(true);
        }
        return Some(false);
    }

    /// 10, 100, or 0 Mbps
    pub fn speed(&self) -> u32 {
        if (self.bmsr & PHY_REG_BMSR_LINK_STATUS) != PHY_REG_BMSR_LINK_STATUS {
            return 0;
        }
        if (self.bmsr & PHY_REG_BMSR_100_FULL) == PHY_REG_BMSR_100_FULL
            || (self.bmsr & PHY_REG_BMSR_100_HALF) == PHY_REG_BMSR_100_HALF
        {
            return 100;
        }
        if (self.bmsr & PHY_REG_BMSR_10_FULL) == PHY_REG_BMSR_10_FULL
            || (self.bmsr & PHY_REG_BMSR_10_HALF) == PHY_REG_BMSR_10_HALF
        {
            return 10;
        }
        return 0;
    }

    /// Error?
    pub fn remote_fault(&self) -> bool {
        (self.bmsr & PHY_REG_BMSR_REMOTE_FAULT) == PHY_REG_BMSR_REMOTE_FAULT
    }
}

/// Compare on base of link detected, full-duplex, and speed
/// attributes.
impl PartialEq for PhyStatus {
    fn eq(&self, other: &PhyStatus) -> bool {
        (self.link_detected() == false && other.link_detected() == false)
            || (self.link_detected() == other.link_detected()
                && self.is_full_duplex() == other.is_full_duplex()
                && self.speed() == other.speed())
    }
}
