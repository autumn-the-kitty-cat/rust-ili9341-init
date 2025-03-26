#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
    gpio::{Output, Pin},
    hal::spi,
    pac::{self, SPI1},
    prelude::*,
    spi::{Error, NoMiso, Spi},
    timer::SysDelay,
};

// COMMAND DEFINITION
// ---------------------------------------------------------------

struct Ili9341Command {
    delay: u8,
    command: u8,
    arguments: [u8; 2],
    arg_count: u8,
}

struct Ili9341 {
    dc: Pin<'C', 2, Output>,
    spi: Spi<SPI1>,
    delay: SysDelay,
}

const ILI9341_BLACK: u16 = 0x0000; //<   0,   0,   0
const ILI9341_NAVY: u16 = 0x000F; //<   0,   0, 123
const ILI9341_DARKGREEN: u16 = 0x03E0; //<   0, 125,   0
const ILI9341_DARKCYAN: u16 = 0x03EF; //<   0, 125, 123
const ILI9341_MAROON: u16 = 0x7800; //< 123,   0,   0
const ILI9341_PURPLE: u16 = 0x780F; //< 123,   0, 123
const ILI9341_OLIVE: u16 = 0x7BE0; //< 123, 125,   0
const ILI9341_LIGHTGREY: u16 = 0xC618; //< 198, 195, 198
const ILI9341_DARKGREY: u16 = 0x7BEF; //< 123, 125, 123
const ILI9341_BLUE: u16 = 0x001F; //<   0,   0, 255
const ILI9341_GREEN: u16 = 0x07E0; //<   0, 255,   0
const ILI9341_CYAN: u16 = 0x07FF; //<   0, 255, 255
const ILI9341_RED: u16 = 0xF800; //< 255,   0,   0
const ILI9341_MAGENTA: u16 = 0xF81F; //< 255,   0, 255
const ILI9341_YELLOW: u16 = 0xFFE0; //< 255, 255,   0
const ILI9341_WHITE: u16 = 0xFFFF; //< 255, 255, 255
const ILI9341_ORANGE: u16 = 0xFD20; //< 255, 165,   0
const ILI9341_GREENYELLOW: u16 = 0xAFE5; //< 173, 255,  41
const ILI9341_PINK: u16 = 0xFC18; //< 255, 130, 198

impl Ili9341 {
    const ILI9341_NOP: u8 = 0x00; // No Operation
    const ILI9341_SWRESET: u8 = 0x01; // Software Reset
    const ILI9341_RDDIDIF: u8 = 0x04; // Read Display Identification Information
    const ILI9341_RDDST: u8 = 0x09; // Read Display Status
    const ILI9341_RDDPM: u8 = 0x0A; // Read Display Power Mode
    const ILI9341_RDDMADCTL: u8 = 0x0B; // Read Display MADCTL
    const ILI9341_RDDCOLMOD: u8 = 0x0C; // Read Display Pixel Format
    const ILI9341_RDDIM: u8 = 0x0D; // Read Display Image Format
    const ILI9341_RDDSM: u8 = 0x0E; // Read Display Signal Mode
    const ILI9341_RDDSDR: u8 = 0x0F; // Read Display Self Diagnostics Result
                                     // ---------------------------------------------------------------
    const ILI9341_SLPIN: u8 = 0x10; // Enter Sleep Mode
    const ILI9341_SLPOUT: u8 = 0x11; // Sleep Out
    const ILI9341_PTLON: u8 = 0x12; // Partial Mode On
    const ILI9341_NORON: u8 = 0x13; // Normal Display On
                                    // ---------------------------------------------------------------
    const ILI9341_DINVOFF: u8 = 0x20; // Dislpay Inversion Off
    const ILI9341_DINVON: u8 = 0x21; // Dislpay Inversion On
    const ILI9341_GAMSET: u8 = 0x26; // Gamma Set
    const ILI9341_DISPOFF: u8 = 0x28; // Display OFF
    const ILI9341_DISPON: u8 = 0x29; // Display ON
    const ILI9341_CASET: u8 = 0x2A; // Column Address Set
    const ILI9341_PASET: u8 = 0x2B; // Page Address Set
    const ILI9341_RAMWR: u8 = 0x2C; // Memory Write
    const ILI9341_RGBSET: u8 = 0x2D; // Color Set
    const ILI9341_RAMRD: u8 = 0x2E; // Memory Read
                                    // ---------------------------------------------------------------
    const ILI9341_PLTAR: u8 = 0x30; // Partial Area
    const ILI9341_VSCRDEF: u8 = 0x33; // Vertical Scroll Definition
    const ILI9341_TEOFF: u8 = 0x34; // Tearing Effect Line OFF
    const ILI9341_TEON: u8 = 0x35; // Tearing Effect Line ON
    const ILI9341_MADCTL: u8 = 0x36; // Memory Access Control
    const ILI9341_VSSAD: u8 = 0x37; // Vertical Scrolling Start Address
    const ILI9341_IDMOFF: u8 = 0x38; // Idle Mode OFF
    const ILI9341_IDMON: u8 = 0x39; // Idle Mode ON
    const ILI9341_COLMOD: u8 = 0x3A; // Pixel Format Set
    const ILI9341_WMCON: u8 = 0x3C; // Write Memory Continue
    const ILI9341_RMCON: u8 = 0x3E; // Read Memory Continue
                                    // ---------------------------------------------------------------
    const ILI9341_IFMODE: u8 = 0xB0; // RGB Interface Signal Control
    const ILI9341_FRMCRN1: u8 = 0xB1; // Frame Control (In Normal Mode)
    const ILI9341_FRMCRN2: u8 = 0xB2; // Frame Control (In Idle Mode)
    const ILI9341_FRMCRN3: u8 = 0xB3; // Frame Control (In Partial Mode)
    const ILI9341_INVTR: u8 = 0xB4; // Display Inversion Control
    const ILI9341_PRCTR: u8 = 0xB5; // Blanking Porch Control
    const ILI9341_DISCTRL: u8 = 0xB6; // Display Function Control
    const ILI9341_ETMOD: u8 = 0xB7; // Entry Mode Set
    const ILI9341_BKCR1: u8 = 0xB8; // Backlight Control 1
    const ILI9341_BKCR2: u8 = 0xB9; // Backlight Control 2
    const ILI9341_BKCR3: u8 = 0xBA; // Backlight Control 3
    const ILI9341_BKCR4: u8 = 0xBB; // Backlight Control 4
    const ILI9341_BKCR5: u8 = 0xBC; // Backlight Control 5
    const ILI9341_BKCR7: u8 = 0xBE; // Backlight Control 7
    const ILI9341_BKCR8: u8 = 0xBF; // Backlight Control 8
                                    // ---------------------------------------------------------------
    const ILI9341_PWCTRL1: u8 = 0xC0; // Power Control 1
    const ILI9341_PWCTRL2: u8 = 0xC1; // Power Control 2
    const ILI9341_VCCR1: u8 = 0xC5; // VCOM Control 1
    const ILI9341_VCCR2: u8 = 0xC7; // VCOM Control 2
                                    // ---------------------------------------------------------------
    const ILI9341_RDID1: u8 = 0xDA; // Read ID1
    const ILI9341_RDID2: u8 = 0xDB; // Read ID2
    const ILI9341_RDID3: u8 = 0xDC; // Read ID3
                                    // ---------------------------------------------------------------
    const ILI9341_GMCTRP1: u8 = 0xE0; // Positive Gamma Correction
    const ILI9341_GMCTRN1: u8 = 0xE1; // Neagtove Gamma Correction

    const INIT_SEQUENCE: [Ili9341Command; 12] = [
        Ili9341Command {
            delay: 50,
            command: Self::ILI9341_SWRESET,
            arguments: [0, 0],
            arg_count: 0,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_DISPOFF,
            arguments: [0, 0],
            arg_count: 0,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_PWCTRL1,
            arguments: [0x23, 0],
            arg_count: 1,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_PWCTRL2,
            arguments: [0x10, 0],
            arg_count: 1,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_VCCR1,
            arguments: [0x2b, 0x2b],
            arg_count: 2,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_VCCR2,
            arguments: [0xc0, 0],
            arg_count: 1,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_MADCTL,
            arguments: [0x48, 0],
            arg_count: 1,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_COLMOD,
            arguments: [0x55, 0],
            arg_count: 1,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_FRMCRN1,
            arguments: [0, 0x1b],
            arg_count: 2,
        },
        Ili9341Command {
            delay: 0,
            command: Self::ILI9341_ETMOD,
            arguments: [0x07, 0],
            arg_count: 1,
        },
        Ili9341Command {
            delay: 150,
            command: Self::ILI9341_SLPOUT,
            arguments: [0, 0],
            arg_count: 0,
        },
        Ili9341Command {
            delay: 200,
            command: Self::ILI9341_DISPON,
            arguments: [0, 0],
            arg_count: 0,
        },
    ];

    fn init(&mut self) -> Result<(), Error> {
        for command in Self::INIT_SEQUENCE {
            // send command
            self.dc.set_low();
            self.spi.write(&[command.command])?;

            self.dc.toggle();
            if command.arg_count > 0 {
                self.spi.write(&[command.arguments[0]])?;
            }

            if command.arg_count > 1 {
                self.spi.write(&[command.arguments[1]])?;
            }

            self.delay.delay_ms(command.delay as u32);
        }

        Ok(())
    }

    fn fill_region(&mut self, x: u16, y: u16, w: u16, h: u16, color: u16) -> Result<(), Error> {
        let [x1_low, x1_high] = x.to_be_bytes();
        let [x2_low, x2_high] = (x + w - 1).to_be_bytes();
        let [y1_low, y1_high] = x.to_be_bytes();
        let [y2_low, y2_high] = (y + h - 1).to_be_bytes();
        let [color_low, color_high] = color.to_be_bytes();
        self.dc.set_low();
        self.spi.write(&[0x2A])?;
        self.dc.set_high();
        self.spi.write(&[x1_high, x1_low, x2_high, x2_low])?;
        self.dc.set_low();
        self.spi.write(&[0x2A])?;
        self.dc.set_high();
        self.spi.write(&[y1_high, y1_low, y2_high, y2_low])?;
        self.dc.set_low();
        self.spi.write(&[0x2C])?;
        self.dc.set_high();
        for _ in 0..(w as u32 * h as u32) {
            self.spi.write(&[color_low, color_high])?;
        }

        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(180.MHz()).freeze();
    let delay = cp.SYST.delay(&clocks);

    let mut rst = gpioc.pc3.into_push_pull_output();
    rst.set_high();
    let mut dc = gpioc.pc2.into_push_pull_output();
    dc.set_low();
    let mut cs = gpioa.pa4.into_push_pull_output();
    cs.set_low();
    let mut sclk = gpiob.pb3.into_push_pull_output();
    sclk.set_low();
    let mut mosi = gpioa.pa7.into_push_pull_output();
    mosi.set_low();

    let spi = p
        .SPI1
        .spi((sclk, NoMiso::new(), mosi), spi::MODE_0, 20.MHz(), &clocks);

    let x: u16 = 0;
    let y: u16 = 0;
    let w: u16 = 240;
    let h: u16 = 320;
    let color: u16 = ILI9341_BLUE.into();

    let mut ili9341 = Ili9341 { spi, dc, delay };
    ili9341.init().unwrap();
    ili9341.fill_region(x, y, w, h, color).unwrap();

    loop {}
}
