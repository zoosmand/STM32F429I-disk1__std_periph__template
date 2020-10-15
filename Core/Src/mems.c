/**
  ******************************************************************************
  * File Name          : mems.c
  * Description        : This file provides code for the Hyroscope.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mems.h"

/* Global variables ----------------------------------------------------------*/
uint32_t _MEMSREG_ = 0;






void MEMS_Handler(void) {
  uint8_t tmp[12];

  /* Read aimed data */
  tmp[0] = L3GD20_OUT_TEMP;
  L3GD20_Read(tmp, 8);
  DisplayHyroscopeAngles(tmp);
  
  /* Reading status clears interrupt pendong bits (for INT1 interrupt) */
  tmp[0] = L3GD20_INT1_SRC;
  L3GD20_Read(tmp, 1);

  /* Setting Bypass mode clears FIFO data (for INT2 interrupt) */
  tmp[0] = L3GD20_FIFO_CTRL_REG;
  tmp[1] = 0x00;
  L3GD20_Write(tmp, 1);

  /* Repeatedly set FIFO mode and watermark level */
  tmp[0] = L3GD20_FIFO_CTRL_REG;
  tmp[1] = (FIFO_CTRL_REG_FM_FIFO | MEMS_WATERMARK);
  L3GD20_Write(tmp, 1);
}

