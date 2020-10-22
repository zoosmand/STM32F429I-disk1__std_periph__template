/**
  ******************************************************************************
  * File Name          : ili9341.c
  * Description        : This file provides code for the configuration
  *                      of ILI9341 TFT Display.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ili9341.h"








////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Sends an initialization command to the display.
  * @param  cmd: a command
  * @return None
  */
 void ILI9341_InitCmd(uint8_t cmd) {
  LCD_WRX_L;
  LCD_CS_L;
  SPI_Write8b(SPI5, cmd);
  Delay(5);
  LCD_CS_H;
}




/**
  * @brief  Senda initialization data to display.
  * @param  data: data
  * @return None
  */
void ILI9341_InitData(uint8_t data) {
  LCD_WRX_H;
  LCD_CS_L;
  SPI_Write8b(SPI5, data);
  Delay(5);
  LCD_CS_H;
}




/**
  * @brief  Sends a sequence of initialization command and data to display.
  * @param  None
  * @return None
  */
void ILI9341_Init(void) {
  ILI9341_InitCmd(ILI9341_SOFTRESET); // 0x01
  Delay(20);

  ILI9341_InitCmd(ILI9341_DISPLAYOFF); // 0x28
  Delay(20);
 
  // // ----------------- Power ------------------
  // ILI9341_InitCmd(ILI9341_POWERCONTROL1); // 0xc0
  // ILI9341_InitData(0x26);
  // ILI9341_InitCmd(ILI9341_POWERCONTROL2); // 0xc1
  // ILI9341_InitData(0x11);
  // ILI9341_InitCmd(ILI9341_VCOMCONTROL1); // 0xc5
  // ILI9341_InitData(0x5c);
  // ILI9341_InitData(0x4c);
  // ILI9341_InitCmd(ILI9341_VCOMCONTROL2); // 0xc7
  // ILI9341_InitData(0x94);
  
  // ---------------- Memory ------------------
  ILI9341_InitCmd(ILI9341_MEMCONTROL); // 0x36
  // -------- Portrait ---------
  // ILI9341_InitData(ILI9341_MADCTL_BGR | ILI9341_MADCTL_MV | ILI9341_MADCTL_MX | ILI9341_MADCTL_MY);
  // -------- Landscape --------
  #ifdef _LANDSCAPE_
    ILI9341_InitData(ILI9341_MADCTL_BGR | ILI9341_MADCTL_MY);
  #else
    ILI9341_InitData(ILI9341_MADCTL_BGR | ILI9341_MADCTL_MV | ILI9341_MADCTL_MX | ILI9341_MADCTL_MY);
  #endif
  
  ILI9341_InitCmd(ILI9341_PIXELFORMAT); // 0x3a
  ILI9341_InitData(0x55);

  // // -------------- Frame rate ---------------
  // ILI9341_InitCmd(ILI9341_FRAMECONTROL); // 0xb1
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0x1b);
  
  // // ---------------- Gamma ------------------
  // ILI9341_InitCmd(ILI9341_GAMMASET); // 0x26
  // ILI9341_InitData(0x01);
  // ILI9341_InitCmd(ILI9341_POSGAMMACORR); // 0xe0
  // ILI9341_InitData(0x1f);
  // ILI9341_InitData(0x1a);
  // ILI9341_InitData(0x18);
  // ILI9341_InitData(0x0a);
  // ILI9341_InitData(0x0f);
  // ILI9341_InitData(0x06);
  // ILI9341_InitData(0x45);
  // ILI9341_InitData(0x87);
  // ILI9341_InitData(0x32);
  // ILI9341_InitData(0x0a);
  // ILI9341_InitData(0x07);
  // ILI9341_InitData(0x02);
  // ILI9341_InitData(0x07);
  // ILI9341_InitData(0x05);
  // ILI9341_InitData(0x00);
  // ILI9341_InitCmd(ILI9341_NEGGAMMACORR); // 0xe1
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0x25);
  // ILI9341_InitData(0x27);
  // ILI9341_InitData(0x05);
  // ILI9341_InitData(0x10);
  // ILI9341_InitData(0x09);
  // ILI9341_InitData(0x3a);
  // ILI9341_InitData(0x78);
  // ILI9341_InitData(0x4d);
  // ILI9341_InitData(0x05);
  // ILI9341_InitData(0x18);
  // ILI9341_InitData(0x0d);
  // ILI9341_InitData(0x38);
  // ILI9341_InitData(0x3a);
  // ILI9341_InitData(0x1f);
  
  // // ---------------- DDRAM -------------------  
  // ILI9341_InitCmd(ILI9341_CASET); // 0x2a
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0xef);
  // ILI9341_InitCmd(ILI9341_PASET); // 0x2b
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0x00);
  // ILI9341_InitData(0x01);
  // ILI9341_InitData(0x3f);
  // ILI9341_InitCmd(ILI9341_RAMWR); // 0x2c
  // ILI9341_InitCmd(ILI9341_ENTRYMODE); // 0xb7
  // ILI9341_InitData(0x07);

  // --------------- Display -----------------
  ILI9341_InitCmd(ILI9341_DISPLAYFUNC); // 0xb6
  ILI9341_InitData(0x0a);
  ILI9341_InitData(0x82);
  ILI9341_InitData(0x27);
  ILI9341_InitData(0x00);
  // ILI9341_InitCmd(ILI9341_INVERTON);

  ILI9341_InitCmd(ILI9341_INTERFACECONTROL); // 0xf6
  ILI9341_InitData(0x01);
  ILI9341_InitData(0x00);
  ILI9341_InitData(0x06);

  ILI9341_InitCmd(ILI9341_SLEEPOUT); // 0x11
  Delay(20);
  ILI9341_InitCmd(ILI9341_DISPLAYON); // 0x29
}
