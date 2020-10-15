/**
  ******************************************************************************
  * File Name          : display.c
  * Description        : This file provides code for the configuration
  *                      of the DISPLAY instances.
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "display.h"


/* Private variables ---------------------------------------------------------*/
/**
  * @brief  Dotted 15x21 pixels font, white, dark-green backgrounded.
  * 18x24 pixels rectangle on the screen.
  */
const Font_TypeDef font_dot15x21 = {
  .width    = 18,
  .height   = 24,
  .color    = ARGB8888_White,
  .bgcolor  = ARGB8888_Darkgreen,
  .use_bg   = 1,
  .font     = (uint8_t*)font_15x21
};

/**
  * @brief  Dotted 20x28 pixels font, white, dark-green backgrounded.
  * 24x32 pixels rectangle on the screen.
  */
const Font_TypeDef font_dot20x28 = {
  .width    = 24,
  .height   = 32,
  .color    = _WHITE,
  .bgcolor  = _WHITE_0,
  .use_bg   = 1,
  .font =   (uint8_t*)font_20x28
};

/**
  * @brief  Dotted 5x7 pixels font, yellow, red-transparent 100% backgrounded.
  * 6x8 pixels rectangle on the screen.
  */
const Font_TypeDef font_dot5x7 = {
  .width = 6,
  .height = 8,
  .color = _YELLOW,
  .bgcolor = _RED_0,
  .use_bg = 1,
  .font = (uint8_t*)font_5x7
};

/**
  * @brief  Dotted 10x14 pixels font, dark-green-transparent(75%), red-transparent(100%) backgrounded.
  * 12x16 pixels rectangle on the screen.
  */
const Font_TypeDef dateTimeFont = {
  .width = 12,
  .height = 16,
  .color = _RED_128,
  .bgcolor = _RED_0,
  .use_bg = 1,
  .font = (uint8_t*)font_10x14
};

static uint16_t xStart = 0;
static uint16_t yStart = 0;










////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Fills a layer of the display with a color
  * @param  layer: a number of the LTDC layer
  * @param  color: a color in format 0xaarrggbb (alpha, red, green, blue)
  * @return None
  */
void FillLayer(uint8_t layer, uint32_t color) {
  __IO uint32_t layerAddr = 0;
  __IO uint32_t layerBytes = 0;

  switch (layer) {
    case L1:
      layerAddr = L1_ADDR;
      layerBytes = L1_BYTES;
      break;
    
    case L2:
      layerAddr = L2_ADDR;
      layerBytes = L2_BYTES;
      break;
    
    default:
      return;
      break;
  }

  for (uint32_t i = 0; i < layerBytes; i += 4) {
    *(__O uint32_t*)(layerAddr + i) = color;
  }
}




/**
  * @brief  Draws a colored pixel on a layer of the display
  * @param  layer: a number of the LTDC layer
  * @param  x: x coordinate
  * @param  y: y coordinate 
  * @param  color: a color in format 0xaarrggbb (alpha, red, green, blue)
  * @return None
  */
void DrawPixel(uint8_t layer, uint16_t x, uint16_t y, uint32_t color) {
  switch (layer) {
    case L1:
      *(__O uint32_t*)(L1_ADDRESS(x, y)) = color;
      break;
    
    case L2:
      *(__O uint32_t*)(L2_ADDRESS(x, y)) = color;
      break;
    
    default:
      return;
      break;
  }
}




/**
  * @brief  Draws a colored vertical line on a layer of the display
  * @param  layer: a number of the LTDC layer
  * @param  x: x coordinate
  * @param  y: y coordinate 
  * @param  h: height of the line
  * @param  color: a color in format 0xaarrggbb (alpha, red, green, blue)
  * @return None
  */
#ifdef _LANDSCAPE_
  void DrawVLine(uint8_t layer, uint16_t x, uint16_t y, uint16_t h, uint32_t color) {
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;

    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(x, y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(x, y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        break;
      
      default:
        return;
        break;
    }

    if ((x >= height) || (y >= width)) return;
    if ((y + h - 1) >= height)  h = height - y;

    // h <<= 2;
    while (h--) {
      *(__O uint32_t*)(layerAddr) = color;
      layerAddr += 4;
    }
  }
#else
  void DrawVLine(uint8_t layer, uint16_t x, uint16_t y, uint16_t h, uint32_t color) {
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t ww = 0;

    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(x, y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        ww = (uint32_t)(L1_WIDTH << 2);
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(x, y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        ww = (uint32_t)(L2_WIDTH << 2);
        break;
      
      default:
        return;
        break;
    }

    if ((x >= height) || (y >= width)) return;
    if ((x + h - 1) >= height)  h = height - y;

    while (h--) {
      *(__O uint32_t*)(layerAddr) = color;
      layerAddr -= ww;
    }
  }
#endif





/**
  * @brief  Draws a colored horizontal line on a layer of the display
  * @param  layer: a number of the LTDC layer
  * @param  x: x coordinate
  * @param  y: y coordinate 
  * @param  w: weight of the line
  * @param  color: a color in format 0xaarrggbb (alpha, red, green, blue)
  * @return None
  */
#ifdef _LANDSCAPE_
  void DrawHLine(uint8_t layer, uint16_t x, uint16_t y, uint16_t w, uint32_t color) {
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t hh = 0;

    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(x, y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        // hh = (uint32_t)(L1_WIDTH << 2);
        hh = (uint32_t)(L1_HEIGHT << 2);
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(x, y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        hh = (uint32_t)(L2_HEIGHT << 2);
        break;
      
      default:
        return;
        break;
    }

    if ((x >= height) || (y >= width)) return;
    if ((x + w - 1) >= width)  w = width - x;

    while (w--) {
      *(__O uint32_t*)(layerAddr) = color;
      layerAddr += hh;
    }
  }
#else
  void DrawHLine(uint8_t layer, uint16_t x, uint16_t y, uint16_t w, uint32_t color) {
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;

    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(x, y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(x, y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        break;
      
      default:
        return;
        break;
    }

    if ((x >= height) || (y >= width)) return;
    if ((y + w - 1) >= height)  w = width - x;

    // h <<= 2;
    while (w--) {
      *(__O uint32_t*)(layerAddr) = color;
      layerAddr += 4;
    }
  }
#endif




/**
  * @brief  Fills rectangle on a layer of the display with a color
  * @param  layer: a number of the LTDC layer
  * @param  x: x coordinate
  * @param  y: y coordinate 
  * @param  h: height of the rectangle
  * @param  w: weight of the rectangle
  * @param  color: a color in format 0xaarrggbb (alpha, red, green, blue)
  * @return None
  */
#ifdef _LANDSCAPE_
  void FillRectangle(uint8_t layer, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color) {
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t ww = 0;

    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(x, y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        ww = (uint32_t)((L1_HEIGHT << 2) - (h << 2));
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(x, y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        ww = (uint32_t)((L2_HEIGHT << 2) - (h << 2));
        break;
      
      default:
        return;
        break;
    }

    if ((x >= width) || (y >= height) || (w == 0) || (h == 0)) return;
    if ((x + w - 1) >= width)  w = width  - x;
    if ((y + h - 1) >= height) h = height - y;

    uint16_t hh = 0;

    while (w--) {
      hh = h;
      while (hh--) {
        *(__O uint32_t*)(layerAddr) = color;
        layerAddr += 4;
      }
      layerAddr += ww;
    }
  }
#else
  void FillRectangle(uint8_t layer, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color) {
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t hh = 0;

    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(x, y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        hh = (uint32_t)((L1_WIDTH << 2) - (w << 2) + (h << 2));
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(x, y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        hh = (uint32_t)((L2_WIDTH << 2) - (w << 2) + (h << 2));
        break;
      
      default:
        return;
        break;
    }

    if ((x >= width) || (y >= height) || (w == 0) || (h == 0)) return;
    if ((x + w - 1) >= width)  w = width  - x;
    if ((y + h - 1) >= height) h = height - y;

    uint16_t ww = 0;

    while (h--) {
      ww = w;
      while (ww--) {
        *(__O uint32_t*)(layerAddr) = color;
        layerAddr += 4;
      }
      layerAddr -= hh;
    }
  }
#endif



/**
  * @brief  Draws symbol on a layer of display
  * @param  layer: number of layer
  * @param  x: pointer to x coordinate
  * @param  y: pointer to y coordinate 
  * @param  font: pointer to font structure containes font paramenters:
  *               - width of a symbol
  *               - height of a symbol
  *               - color of a symbol
  *               - background color behind a symbol
  *               - flag - use or don't use background color
  *               - array of symbols
  * @param  pos: position in the sysmbol's array
  * @return None
  */
#ifdef _LANDSCAPE_
  void DrawSymbol(uint8_t layer, uint16_t *x, uint16_t *y, const Font_TypeDef *font, uint8_t pos) {
    if ((pos > 126) || (pos < 32)) {
      if (pos == 176) pos = 95;
      else return;
    } else {
      pos -= 32;
    }
    int16_t w = font->width;
    int16_t h = font->height;
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t ww = 0;
    
    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(*x, *y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        ww = (uint32_t)((L1_HEIGHT << 2) - (h << 2));
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(*x, *y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        ww = (uint32_t)((L2_HEIGHT << 2) - (h << 2));
        break;
      
      default:
        return;
        break;
    }

    if ((*x >= width) || (*y >= height) || (w == 0) || (h == 0)) return;
    if ((*x + w - 1) >= width)  w = width  - *x;
    if ((*y + h - 1) >= height) h = height - *y;

    uint8_t smb = 0;
    uint8_t z = 0;
    uint8_t *fontAddr = font->font;
    
    uint16_t hh = 0;
    *x += w;

    while (w--) {
      z = font->height / 8;
      while (z--) {
        smb = fontAddr[(pos * font->width * (font->height / 8))];
        fontAddr++;

        hh = 8;
        while (hh--) {
          if (smb & 0x01) {
            *(__O uint32_t*)layerAddr = font->color;
          } else {
            if (font->use_bg) {
              *(__O uint32_t*)layerAddr = font->bgcolor;
            }
          }
          layerAddr +=4;
          smb >>= 1;
        }
      }
      layerAddr += ww;
    }
  }
#else
  void DrawSymbol(uint8_t layer, uint16_t *x, uint16_t *y, const Font_TypeDef *font, uint8_t pos) {
    if ((pos > 126) || (pos < 32)) {
      if (pos == 176) pos = 95;
      else return;
    } else {
      pos -= 32;
    }
    int16_t w = font->width;
    int16_t h = font->height;
    uint32_t layerAddr = 0;
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t ww = 0;
    
    switch (layer) {
      case L1:
        layerAddr = L1_ADDRESS(*x, *y);
        height = L1_HEIGHT;
        width = L1_WIDTH;
        ww = (uint32_t)(L1_WIDTH << 2);
        break;
      
      case L2:
        layerAddr = L2_ADDRESS(*x, *y);
        height = L2_HEIGHT;
        width = L2_WIDTH;
        ww = (uint32_t)(L2_WIDTH << 2);
        break;
      
      default:
        return;
        break;
    }

    if ((*x >= width) || (*y >= height) || (w == 0) || (h == 0)) return;
    if ((*x + w - 1) >= width)  w = width  - *x;
    if ((*y + h - 1) >= height) h = height - *y;

    uint8_t smb = 0;
    uint8_t z = 0;
    uint8_t *fontAddr = font->font;
    
    uint16_t hh = 0;
    *x += w;
    while (w--) {
      z = font->height / 8;
      while (z--) {
        smb = fontAddr[(pos * font->width * (font->height / 8))];
        fontAddr++;

        hh = 8;
        while (hh--) {
          if (smb & 0x01) {
            *(__O uint32_t*)layerAddr = font->color;
          } else {
            if (font->use_bg) {
              *(__O uint32_t*)layerAddr = font->bgcolor;
            }
          }
          layerAddr -= ww;
          smb >>= 1;
        }
      }
      layerAddr += (1 + (font->height * width)) << 2;
    }
  }
#endif



/**
  * @brief  Prints a string on a layer of display
  * @param  layer: number of layer
  * @param  x: pointer to x coordinate
  * @param  y: pointer to y coordinate 
  * @param  font: pointer to font structure containes font paramenters:
  *               - width of a symbol
  *               - height of a symbol
  *               - color of a symbol
  *               - background color behind a symbol
  *               - flag - use or not use background color
  *               - array of symbols
  * @param  buf: pointer to buffer with a string
  * @param  wrap: wrap or not wrap to the next line 
  * @return None
  */
void PrintString(uint8_t layer, uint16_t *x, uint16_t *y, const Font_TypeDef *font, const char *buf, uint8_t wrap) {
  uint32_t width = 0;

  switch (layer) {
    case L1:
      width = L1_WIDTH;
      break;
    
    case L2:
      width = L2_WIDTH;
      break;
    
    default:
      return;
      break;
  }

  while (*buf) {
    if (*x > width - 10) {
      if (wrap) {
        *x = 0;
        *y -= (font->height + 4);
      } else {
        return;
      }
    }
    DrawSymbol(layer, x, y, font, *buf);
    buf++;
  }
}





/**
  * @brief  Reads date and time from the RTC and displays them
  * @param  None
  * @return None
  */
void DisplayDateAndTime_Handler(void) {
    static char dateStr[12];
    static char timeStr[12];
    
    sprintf(timeStr, "%2x:%02x:%02x ", RTC_GetHour, RTC_GetMinute, RTC_GetSecond);
    sprintf(dateStr, "%2x/%x/%4x ", RTC_GetDay, RTC_GetMonth, RTC_GetYear);
    // printf("date: %s\n\r", dateStr);
    // printf("time: %s\n\r", timeStr);

    DisplayDate(dateStr);
    DisplayTime(timeStr);

    // DrawVLine(L1, 10, 20, 100, _YELLOW);
    // DrawHLine(L1, 10, 20, 100, _WHITE);

    // FillRectangle(L1, 40, 120, 50, 100, _ORANGE);
}



/**
  * @brief  Displays date on a layer 1 of display to certain coordinates
  * @param  buf: pointer to buffer with date digits
  * @return None
  */
void DisplayDate(const char* buf) {
  #ifdef _LANDSCAPE_
    xStart = 100;
    yStart = 120;
  #else
    xStart = 60;
    yStart = 170;
  #endif

  PrintString(L1, &xStart, &yStart, &dateTimeFont, buf, 0);
}




/**
  * @brief  Displays time on a layer 1 of display to certain coordinates
  * @param  buf: pointer to buffer with time digits
  * @return None
  */
void DisplayTime(const char* buf) {
  #ifdef _LANDSCAPE_
    xStart = 100;
    yStart = 90;
  #else
    xStart = 60;
    yStart = 140;
  #endif

  PrintString(L1, &xStart, &yStart, &dateTimeFont, buf, 0);
}






/**
  * @brief  Displays coordinates of the touch screen
  * @param  buf: pointer to buffer with data
  * @return None
  */
void DisplayCoordinates(const uint8_t *buf) {
  //char tapStr[128];
  char tapStrX[16];
  char tapStrY[16];

  sprintf(tapStrX, "x: %x   ", buf[0]);
  sprintf(tapStrY, "y: %x   ", buf[2]);

  xStart = 0;
  yStart = 184;
  PrintString(L2, &xStart, &yStart, &font_dot5x7, tapStrX, 0);

  xStart = 0;
  yStart -= 24;
  PrintString(L2, &xStart, &yStart, &font_dot5x7, tapStrY, 0);

}





/**
  * @brief  Displays Hyroscope angles (raw data)
  * @param  buf: pointer to buffer with data
  * @return None
  */
void DisplayHyroscopeAngles(const uint8_t *buf) {
  char tapStrX[16];
  char tapStrY[16];
  char tapStrZ[16];

  sprintf(tapStrX, "X: %d     ", (int16_t)((buf[4] << 8) | buf[3]));
  sprintf(tapStrY, "Y: %d     ", (int16_t)((buf[6] << 8) | buf[5]));
  sprintf(tapStrZ, "Z: %d     ", (int16_t)((buf[8] << 8) | buf[7]));

  xStart = 0;
  yStart = 100;
  PrintString(L2, &xStart, &yStart, &font_dot5x7, tapStrX, 0);

  xStart = 0;
  yStart -= 24;
  PrintString(L2, &xStart, &yStart, &font_dot5x7, tapStrY, 0);

  xStart = 0;
  yStart -= 24;
  PrintString(L2, &xStart, &yStart, &font_dot5x7, tapStrZ, 0);

}
