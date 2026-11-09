/*
 * ESP32-S3-Touch-LCD-4.3 Board Configuration
 * Projeto: ESPRemotecontrol (controle remoto de luzes auxiliares via ESP-NOW)
 *
 * Baseado na configuração do projeto IQ50LCD, com o touchscreen HABILITADO
 * (necessário para a nova interface de controle).
 */

#pragma once

/* Set to 1 if using a custom board */
#define ESP_PANEL_USE_SUPPORTED_BOARD       (0)

/* Set to 1 when using an LCD panel */
#define ESP_PANEL_USE_LCD           (1)

/* LCD Configuration - Landscape Mode 800x480 */
#define ESP_PANEL_LCD_BUS_TYPE      (ESP_PANEL_BUS_TYPE_RGB)
#define ESP_PANEL_LCD_WIDTH         (800)
#define ESP_PANEL_LCD_HEIGHT        (480)

/* LCD Device Vendor */
#define ESP_PANEL_LCD_VENDOR_INIT_CMD       (lcd_init_cmd_st7262)
#define ESP_PANEL_LCD_ST7262                (1)

/* RGB LCD Configuration - Landscape Mode 800x480 */
#define ESP_PANEL_LCD_H_RES         (800)
#define ESP_PANEL_LCD_V_RES         (480)

/* RGB Bus GPIO Configuration */
#define ESP_PANEL_LCD_RGB_IO_HSYNC  (46)
#define ESP_PANEL_LCD_RGB_IO_VSYNC  (3)
#define ESP_PANEL_LCD_RGB_IO_DE     (0)
#define ESP_PANEL_LCD_RGB_IO_PCLK   (7)
#define ESP_PANEL_LCD_RGB_IO_DISP   (-1)

/* RGB Data pins */
#define ESP_PANEL_LCD_RGB_IO_DATA0  (14)
#define ESP_PANEL_LCD_RGB_IO_DATA1  (38)
#define ESP_PANEL_LCD_RGB_IO_DATA2  (18)
#define ESP_PANEL_LCD_RGB_IO_DATA3  (17)
#define ESP_PANEL_LCD_RGB_IO_DATA4  (10)
#define ESP_PANEL_LCD_RGB_IO_DATA5  (39)
#define ESP_PANEL_LCD_RGB_IO_DATA6  (0)
#define ESP_PANEL_LCD_RGB_IO_DATA7  (45)
#define ESP_PANEL_LCD_RGB_IO_DATA8  (48)
#define ESP_PANEL_LCD_RGB_IO_DATA9  (47)
#define ESP_PANEL_LCD_RGB_IO_DATA10 (21)
#define ESP_PANEL_LCD_RGB_IO_DATA11 (1)
#define ESP_PANEL_LCD_RGB_IO_DATA12 (2)
#define ESP_PANEL_LCD_RGB_IO_DATA13 (42)
#define ESP_PANEL_LCD_RGB_IO_DATA14 (41)
#define ESP_PANEL_LCD_RGB_IO_DATA15 (40)

/* RGB Timing Configuration
 * NOTA: estes defines nao sao lidos por nenhum codigo (rgb_lcd_port.cpp usa
 * valores hardcoded via EXAMPLE_LCD_PIXEL_CLOCK_HZ e os porches fixos no
 * proprio esp_lcd_rgb_panel_config_t). Mantidos apenas como documentacao de
 * referencia do painel ST7262 800x480. */
#define ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ    (16 * 1000 * 1000)
#define ESP_PANEL_LCD_RGB_TIMING_HPW        (4)
#define ESP_PANEL_LCD_RGB_TIMING_HBP        (8)
#define ESP_PANEL_LCD_RGB_TIMING_HFP        (8)
#define ESP_PANEL_LCD_RGB_TIMING_VPW        (4)
#define ESP_PANEL_LCD_RGB_TIMING_VBP        (8)
#define ESP_PANEL_LCD_RGB_TIMING_VFP        (8)
#define ESP_PANEL_LCD_RGB_TIMING_FLAGS      (0)

/* RGB Bounce Buffer Configuration */
#define ESP_PANEL_LCD_RGB_BOUNCE_BUF_SIZE   (ESP_PANEL_LCD_WIDTH * 10)

/* LCD Color Configuration */
#define ESP_PANEL_LCD_COLOR_BITS    (16)
#define ESP_PANEL_LCD_RGB_IO_RGB565 (1)

/* Touch Configuration - ENABLED (GT911 via I2C) */
#define ESP_PANEL_USE_TOUCH         (1)
#define ESP_OPEN_TOUCH              (1)

/* I2C Configuration for IO Expander, touch (GT911) and IMU (QMI8658) - shared bus */
#define ESP_PANEL_USE_IO_EXPANDER   (1)
#define ESP_IO_EXPANDER_I2C_NUM     (0)
#define ESP_IO_EXPANDER_I2C_SCL     (9)
#define ESP_IO_EXPANDER_I2C_SDA     (8)
#define ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000 (0x24)
