/**
  ******************************************************************************
  * File Name          : tc.c
  * Description        : This file provides code for the configuration
  *                      of the TouchScreen instances.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ts.h"

/* Global variables ----------------------------------------------------------*/
uint32_t _TSREG_ = 0;

/* Private variables ---------------------------------------------------------*/







/**
  * @brief  Reads data from an STMPE811 chip and displays it
  * @param  None
  * @return  None
  */
void TS_Handler(void) {
  uint8_t tmp[8];
  /* Read Coordinates */
  PE811_Read(PE811_TSC_DATA_X, tmp, 8);
  /* Read Temperature registers */
  PE811_Read(PE811_TEMP_CTRL, &tmp[5], 3);
  /* Display on the screen */  
  DisplayCoordinates(tmp);
  /* Clear pending bits */
  PE811_Write(PE811_INT_STA, 0x0ff);
}

