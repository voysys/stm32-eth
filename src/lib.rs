#![no_std]

#[cfg(feature = "stm32f107")]
use stm32f1xx_hal::stm32::{Interrupt, ETHERNET_DMA, ETHERNET_MAC, NVIC};
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::stm32::{Interrupt, ETHERNET_DMA, ETHERNET_MAC, NVIC};

// If no phy specified, print error message.
#[cfg(not(any(feature = "dp83848", feature = "lan8742",)))]
compile_error!("Phy not specified. A `--features <phy-name>` is required.");

// If any two or more phy:s are specified, print error message.
#[cfg(all(feature = "dp83848", feature = "lan8742"))]
compile_error!("Multiple Phy:s specified. Only a single `--features <phy-name>` can be specified.");

#[cfg(feature = "lan8742")]
pub mod phy_lan8742;
#[cfg(feature = "lan8742")]
pub use phy_lan8742 as phy;
#[cfg(feature = "lan8742")]
use phy_lan8742::{Phy, PhyStatus};

#[cfg(feature = "dp83848")]
pub mod phy_dp83848;
#[cfg(feature = "dp83848")]
pub use phy_dp83848 as phy;
#[cfg(feature = "dp83848")]
use phy_dp83848::{Phy, PhyStatus};

mod ring;
mod smi;
pub use ring::RingEntry;
mod desc;
mod rx;
pub use rx::{RxDescriptor, RxError};
use rx::{RxPacket, RxRing, RxRingEntry};
mod tx;
pub use tx::{TxDescriptor, TxError};
use tx::{TxRing, TxRingEntry};
mod setup;
pub use setup::setup;
#[cfg(feature = "nucleo-f429zi")]
pub use setup::setup_pins;

#[cfg(feature = "smoltcp-phy")]
pub use smoltcp;
#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::{EthRxToken, EthTxToken};

#[cfg(feature = "lan8742")]
const PHY_ADDR: u8 = 0;
#[cfg(feature = "dp83848")]
const PHY_ADDR: u8 = 1;

/// From the datasheet: *VLAN Frame maxsize = 1522*
const MTU: usize = 1522;

/// Ethernet driver for *STM32* chips.
/// [`Phy`](phy/struct.Phy.html) can be selected via feature as:
/// *lan8742* (e.g. on STM Nucleo-144 boards)
/// *dp83848*
pub struct Eth<'rx, 'tx> {
    eth_mac: ETHERNET_MAC,
    eth_dma: ETHERNET_DMA,
    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx>,
}

impl<'rx, 'tx> Eth<'rx, 'tx> {
    /// Initialize and start tx and rx DMA engines.
    ///
    /// You must call [`setup()`](fn.setup.html) before to initialize
    /// the hardware!
    ///
    /// Make sure that the buffers reside in a memory region that is
    /// accessible by the peripheral. Core-Coupled Memory (CCM) is
    /// usually not.
    ///
    /// Other than that, initializes and starts the Ethernet hardware
    /// so that you can [`send()`](#method.send) and
    /// [`recv_next()`](#method.recv_next).
    pub fn new(
        eth_mac: ETHERNET_MAC,
        eth_dma: ETHERNET_DMA,
        rx_buffer: &'rx mut [RxRingEntry],
        tx_buffer: &'tx mut [TxRingEntry],
    ) -> Self {
        let mut eth = Eth {
            eth_mac,
            eth_dma,
            rx_ring: RxRing::new(rx_buffer),
            tx_ring: TxRing::new(tx_buffer),
        };
        eth.init();
        eth.rx_ring.start(&eth.eth_dma);
        eth.tx_ring.start(&eth.eth_dma);
        eth
    }

    fn init(&mut self) -> &Self {
        self.reset_mac_and_wait();

        // set clock range in MAC MII address register
        #[cfg(feature = "stm32f107")]
        {
            self.eth_mac.macmiiar.modify(|_, w| w.cr().cr_35_60());
        }
        #[cfg(feature = "stm32f4xx")]
        {
            self.eth_mac.macmiiar.modify(|_, w| w.cr().cr_20_35());
        }

        self.get_phy().reset().set_autoneg();

        // Configuration Register
        #[cfg(feature = "stm32f4xx")]
        self.eth_mac.maccr.modify(|_, w| {
            // CRC stripping for Type frames
            w.cstf()
                .set_bit()
                // Fast Ethernet speed
                .fes()
                .set_bit()
                // Duplex mode
                .dm()
                .set_bit()
                // Automatic pad/CRC stripping
                .apcs()
                .set_bit()
                // Retry disable in half-duplex mode
                .rd()
                .set_bit()
                // Receiver enable
                .re()
                .set_bit()
                // Transmitter enable
                .te()
                .set_bit()
        });
        #[cfg(feature = "stm32f107")]
        self.eth_mac.maccr.modify(|_, w| {
            // Fast Ethernet speed
            w.fes()
                .set_bit()
                // Duplex mode
                .dm()
                .set_bit()
                // Automatic pad/CRC stripping
                .apcs()
                .set_bit()
                // Retry disable in half-duplex mode
                .rd()
                .set_bit()
                // Receiver enable
                .re()
                .set_bit()
                // Transmitter enable
                .te()
                .set_bit()
                // Checksum offload
                .ipco()
                .set_bit()
        });

        // frame filter register
        self.eth_mac.macffr.modify(|_, w| {
            // Receive All
            w.ra()
                .set_bit()
                // Promiscuous mode
                .pm()
                .set_bit()
        });
        // Flow Control Register
        self.eth_mac.macfcr.modify(|_, w| {
            // Pause time
            w.pt().bits(0x100)
        });
        // operation mode register
        self.eth_dma.dmaomr.modify(|_, w| {
            // Dropping of TCP/IP checksum error frames disable
            w.dtcefd()
                .set_bit()
                // Receive store and forward
                .rsf()
                .set_bit()
                // Disable flushing of received frames
                .dfrf()
                .set_bit()
                // Transmit store and forward
                .tsf()
                .set_bit()
                // Forward error frames
                .fef()
                .set_bit()
                // Operate on second frame
                .osf()
                .set_bit()
        });
        // bus mode register
        self.eth_dma.dmabmr.modify(|_, w| unsafe {
            // Address-aligned beats
            w.aab()
                .set_bit()
                // Fixed burst
                .fb()
                .set_bit()
                // Rx DMA PBL
                .rdp()
                .bits(32)
                // Programmable burst length
                .pbl()
                .bits(32)
                // Rx Tx priority ratio 2:1
                .pm()
                .bits(0b01)
                // Use separate PBL
                .usp()
                .set_bit()
        });

        self
    }

    /// reset all MAC subsystem internal registers and logic
    fn reset_mac_and_wait(&self) {
        self.eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

        // Wait until done
        while self.eth_dma.dmabmr.read().sr().bit_is_set() {}
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html) to
    /// clear interrupt pending bits. Otherwise the interrupt will
    /// reoccur immediately.
    pub fn enable_interrupt(&self) {
        self.eth_dma.dmaier.modify(|_, w| {
            w
                // Normal interrupt summary enable
                .nise()
                .set_bit()
                // Receive Interrupt Enable
                .rie()
                .set_bit()
                // Transmit Interrupt Enable
                .tie()
                .set_bit()
        });

        // Enable ethernet interrupts
        let interrupt = Interrupt::ETH;

        unsafe {
            NVIC::unmask(interrupt);
        }
    }

    /// Calls [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html)
    pub fn interrupt_handler(&self) {
        eth_interrupt_handler(&self.eth_dma);
    }

    /// Construct a PHY driver
    pub fn get_phy<'a>(&'a self) -> Phy<'a> {
        Phy::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, PHY_ADDR)
    }

    /// Obtain PHY status
    pub fn status(&self) -> PhyStatus {
        self.get_phy().status()
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call `recv_next()` to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        self.rx_ring.running_state(&self.eth_dma).is_running()
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(&mut self) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(&self.eth_dma)
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        self.tx_ring.is_running(&self.eth_dma)
    }

    /// Send a packet
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        f: F,
    ) -> Result<R, TxError> {
        let result = self.tx_ring.send(length, f);
        self.tx_ring.demand_poll(&self.eth_dma);
        result
    }
}

/// Call in interrupt handler to clear interrupt reason, when
/// [`enable_interrupt()`](struct.Eth.html#method.enable_interrupt).
///
/// There are two ways to call this:
///
/// * Via the [`Eth`](struct.Eth.html) driver instance that your interrupt handler has access to.
/// * By unsafely getting `Peripherals`.
///
/// TODO: could return interrupt reason
pub fn eth_interrupt_handler(eth_dma: &ETHERNET_DMA) {
    eth_dma
        .dmasr
        .write(|w| w.nis().set_bit().rs().set_bit().ts().set_bit());
}
