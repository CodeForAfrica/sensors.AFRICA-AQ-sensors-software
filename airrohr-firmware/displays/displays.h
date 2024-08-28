#ifndef DISPLAYS_H
#define DISPLAYS_H

#include "./oledfont.h" // avoids including the default Arial font, needs to be included before SSD1306.h
#include <SSD1306.h>
#include <SH1106.h>
#include <LiquidCrystal_I2C.h>

/*****************************************************************
 * Display definitions                                           *
 *****************************************************************/
SSD1306 display(0x3c, I2C_PIN_SDA, I2C_PIN_SCL);
SH1106 display_sh1106(0x3c, I2C_PIN_SDA, I2C_PIN_SCL);
LiquidCrystal_I2C *lcd_1602 = nullptr;
LiquidCrystal_I2C *lcd_2004 = nullptr;

/*****************************************************************
 * Init OLED display                                             *
 *****************************************************************/
static void init_display()
{
    display.init();
    display_sh1106.init();
    if (cfg::has_flipped_display)
    {
        display.flipScreenVertically();
        display_sh1106.flipScreenVertically();
    }
}

/*****************************************************************
 * Init LCD display                                              *
 *****************************************************************/
static void init_lcd()
{
    if (cfg::has_lcd1602)
    {
        lcd_1602 = new LiquidCrystal_I2C(0x3f, 16, 2);
    }
    else if (cfg::has_lcd1602_27)
    {
        lcd_1602 = new LiquidCrystal_I2C(0x27, 16, 2);
    }
    if (lcd_1602)
    {
        lcd_1602->init();
        lcd_1602->backlight();
    }

    if (cfg::has_lcd2004)
    {
        lcd_2004 = new LiquidCrystal_I2C(0x3f, 20, 4);
    }
    else if (cfg::has_lcd2004_27)
    {
        lcd_2004 = new LiquidCrystal_I2C(0x27, 20, 4);
    }
    if (lcd_2004)
    {
        lcd_2004->init();
        lcd_2004->backlight();
    }
}

#endif
