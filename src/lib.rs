#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GtChannel {
    Channel1 = 0,
    Channel2 = 1,
    Channel3 = 2,
    Channel4 = 3,
    Channel5 = 4,
    Channel6 = 5,
    Channel7 = 6,
    Channel8 = 7,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GtAddress {
    Gt0 = 0,
    Gt1 = 1,
    Gt2 = 2,
    Gt3 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CvChannel {
    Channel1 = 0x00,
    Channel2 = 0x20,
    Channel3 = 0x40,
    Channel4 = 0x60,
    Channel5 = 0x70,
    Channel6 = 0x50,
    Channel7 = 0x30,
    Channel8 = 0x10,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CvAddress {
    Cv1 = 1,
    Cv2 = 2,
    Cv3 = 3,
    Cv4 = 4,
    Cv5 = 5,
    Cv6 = 6,
    Cv7 = 7,
}

pub struct Fhx<'a, Spi, PinCS, PinA0, PinA1, PinA2> {
    spi: Spi,
    cs: PinCS,
    addr0: PinA0,
    addr1: PinA1,
    addr2: PinA2,

    // current gate values, cached to allow switching a single gate
    gates_value: [u8; 4],

    /// Transmit buffer for SPI communication. Must be DMA-accessible.
    tx_buffer: &'a mut [u8; 4],
}

impl<'a, Spi, PinCS, PinA0, PinA1, PinA2> Fhx<'a, Spi, PinCS, PinA0, PinA1, PinA2>
where
    Spi: embedded_hal_async::spi::SpiBus<u8> + embedded_hal::spi::SpiBus<u8>,
    PinCS: embedded_hal::digital::OutputPin,
    PinA0: embedded_hal::digital::OutputPin,
    PinA1: embedded_hal::digital::OutputPin,
    PinA2: embedded_hal::digital::OutputPin,

{
    pub fn new(spi: Spi, mut cs: PinCS, addr0: PinA0, addr1: PinA1, addr2: PinA2, tx_buffer: &'a mut [u8; 4]) -> Self {
        cs.set_high().unwrap();

        let mut helper = Self {
            spi,
            cs,
            addr0,
            addr1,
            addr2,

            gates_value: [0; 4],

            tx_buffer,
        };

        // Initialize all gates to 0
        helper.update_gates_blocking(GtAddress::Gt0);
        helper.update_gates_blocking(GtAddress::Gt1);
        helper.update_gates_blocking(GtAddress::Gt2);
        helper.update_gates_blocking(GtAddress::Gt3);

        helper
    }

    pub async fn set_gates(&mut self, addr: GtAddress, values: u8) {
        self.gates_value[addr as usize] = values;
        self.update_gates(addr).await;
    }

    pub async fn gate_high(&mut self, addr: GtAddress, channel: GtChannel) {
        self.gates_value[addr as usize] |= 1 << channel as u8;
        self.update_gates(addr).await;
    }

    pub async fn gate_low(&mut self, addr: GtAddress, channel: GtChannel) {
        self.gates_value[addr as usize] &= !(1 << channel as u8);
        self.update_gates(addr).await;
    }

    // ---

    pub fn set_cv_polarity(&mut self, addr: CvAddress, polarity: u8) {
        self.set_cv_address(addr);

        self.write_blocking(&[0x14u8, 0x00, polarity, 0x00]);
    }

    pub async fn set_cv_raw(&mut self, addr: CvAddress, channel: CvChannel, value: u16) {
        let message = [
            0x03u8,
            channel as u8 | ((value & 0xF000) >> 12) as u8,
            ((value & 0x0FF0) >> 4) as u8,
            ((value & 0x000F) << 4) as u8,
        ];

        self.set_cv_address(addr);
        self.write(&message).await;
    }

    // ---

    async fn update_gates(&mut self, addr: GtAddress) {
        match addr {
            GtAddress::Gt0 => {
                self.set_address(false, false, false);
            }
            GtAddress::Gt1 => {
                self.set_address(true, false, false);
            }
            GtAddress::Gt2 => {
                self.set_address(false, true, false);
            }
            GtAddress::Gt3 => {
                self.set_address(true, true, false);
            }
        }

        self.write(&[0x04u8, 0x00, self.gates_value[addr as usize], 0x00])
            .await;
    }

    fn update_gates_blocking(&mut self, addr: GtAddress) {
        match addr {
            GtAddress::Gt0 => {
                self.set_address(false, false, false);
            }
            GtAddress::Gt1 => {
                self.set_address(true, false, false);
            }
            GtAddress::Gt2 => {
                self.set_address(false, true, false);
            }
            GtAddress::Gt3 => {
                self.set_address(true, true, false);
            }
        }

        self.write_blocking(&[0x04u8, 0x00, self.gates_value[addr as usize], 0x00]);
    }

    fn set_cv_address(&mut self, addr: CvAddress) {
        match addr {
            CvAddress::Cv1 => self.set_address(true, false, false),
            CvAddress::Cv2 => self.set_address(false, true, false),
            CvAddress::Cv3 => self.set_address(true, true, false),
            CvAddress::Cv4 => self.set_address(false, false, true),
            CvAddress::Cv5 => self.set_address(true, false, true),
            CvAddress::Cv6 => self.set_address(false, true, true),
            CvAddress::Cv7 => self.set_address(true, true, true),
        }
    }

    #[inline]
    fn set_address(&mut self, a0: bool, a1: bool, a2: bool) {
        self.addr0.set_state(a0.into()).unwrap();
        self.addr1.set_state(a1.into()).unwrap();
        self.addr2.set_state(a2.into()).unwrap();
    }

    async fn write(&mut self, data: &[u8; 4]) {
        self.tx_buffer.copy_from_slice(data);

        self.cs.set_low().unwrap();
        <Spi as embedded_hal_async::spi::SpiBus<u8>>::write(&mut self.spi, self.tx_buffer)
            .await
            .unwrap();
        self.cs.set_high().unwrap();
    }

    fn write_blocking(&mut self, data: &[u8; 4]) {
        self.cs.set_low().unwrap();
        <Spi as embedded_hal::spi::SpiBus<u8>>::write(&mut self.spi, data).unwrap();
        self.cs.set_high().unwrap();
    }
}
