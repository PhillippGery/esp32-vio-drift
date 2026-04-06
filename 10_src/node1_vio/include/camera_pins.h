#pragma once
// ============================================================
// XIAO ESP32S3 Sense — OV2640 Camera Pin Map
// These are FIXED by the Sense board's FPC connector routing.
// Do NOT change unless you're using a different board.
// ============================================================

#define PWDN_GPIO_NUM     -1   // No power-down pin on Sense board
#define RESET_GPIO_NUM    -1   // No external reset pin
#define XCLK_GPIO_NUM     10   // Camera XCLK
#define SIOD_GPIO_NUM     40   // SCCB (I2C) SDA
#define SIOC_GPIO_NUM     39   // SCCB (I2C) SCL

#define Y9_GPIO_NUM       48   // D7
#define Y8_GPIO_NUM       11   // D6
#define Y7_GPIO_NUM       12   // D5
#define Y6_GPIO_NUM       14   // D4
#define Y5_GPIO_NUM       16   // D3
#define Y4_GPIO_NUM       18   // D2
#define Y3_GPIO_NUM       17   // D1
#define Y2_GPIO_NUM       15   // D0

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13