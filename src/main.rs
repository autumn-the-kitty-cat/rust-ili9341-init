#![no_main]
#![no_std]

mod color;
mod ili9341;

use cortex_m_rt::entry;
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use panic_halt as _;
use stm32f4xx_hal::{hal::spi, pac, prelude::*, spi::NoMiso};
use tinybmp::Bmp;

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
    let mut dc = gpioc.pc2.into_push_pull_output();
    let mut cs = gpioa.pa4.into_push_pull_output();
    let mut sclk = gpiob.pb3.into_push_pull_output();
    let mut mosi = gpioa.pa7.into_push_pull_output();

    rst.set_high();
    dc.set_low();
    cs.set_low();
    sclk.set_low();
    mosi.set_low();

    let spi = p
        .SPI1
        .spi((sclk, NoMiso::new(), mosi), spi::MODE_0, 20.MHz(), &clocks);

    let mut lcd = ili9341::Ili9341::new(dc, spi, delay);
    lcd.clear(Rgb565::WHITE).unwrap();

    let bmp_data = include_bytes!("../image.bmp");
    let bmp: Bmp<Rgb565> = Bmp::from_slice(bmp_data).unwrap();

    Image::new(&bmp, Point::new(120, 160))
        .draw(&mut lcd)
        .unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
