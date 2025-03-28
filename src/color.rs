#![allow(unused)]

use embedded_graphics::{pixelcolor::raw::ToBytes, prelude::RawData};
#[derive(Copy, Clone)]
pub enum Color {
    Black = 0x0000,       //<   0,   0,   0
    Navy = 0x000F,        //<   0,   0, 123
    DarkGreen = 0x03E0,   //<   0, 125,   0
    DarkCyan = 0x03EF,    //<   0, 125, 123
    Maroon = 0x7800,      //< 123,   0,   0
    Purple = 0x780F,      //< 123,   0, 123
    Olive = 0x7BE0,       //< 123, 125,   0
    LightGrey = 0xC618,   //< 198, 195, 198
    DarkGrey = 0x7BEF,    //< 123, 125, 123
    Blue = 0x001F,        //<   0,   0, 255
    Green = 0x07E0,       //<   0, 255,   0
    Cyan = 0x07FF,        //<   0, 255, 255
    Red = 0xf800,         //< 255,   0,   0
    Magenta = 0xF81F,     //< 255,   0, 255
    Yellow = 0xFFE0,      //< 255, 255,   0
    White = 0xFFFF,       //< 255, 255, 255
    Orange = 0xFD20,      //< 255, 165,   0
    GreenYellow = 0xAFE5, //< 173, 255,  41
    Pink = 0xFC18,        //< 255, 130, 198
}
