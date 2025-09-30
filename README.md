# ESP Components for Convenience

This repo contains several components

- oled_text

## Oled_text

This component combines two libraries `esp/esp_lcd_sh1107` and `lvgl/lvgl`. Main purpose is to add an inexpensive oled display to your esp32 development board in order to see information displayed on it.

The oled display chip used in this case is a SSD1306 with a resoution of 128x32.

The component will 
- Initialize i2c
- Create a handle to the screen
- Allow you to display static or scrolling text

More functionality will be added later.
