/**
  ******************************************************************************
  * File Name          : pe811.c
  * Description        : This file provides code for the configuration
  *                      of STMPE811 Touch Screen for a Display.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pe811.h"


/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/







/**
  * @brief  Initializes the STMPE811 chip
  * @param  None
  * @return  None
  */
void PE811_Init(void) {
  /* Enable clock */
  PE811_Write(PE811_SYS_CTRL2, (~(SYS_CTRL2_ADC_OFF | SYS_CTRL2_TSC_OFF | SYS_CTRL2_TS_OFF) & 0x0f));
  /* Enable interrupts */
  PE811_Write(PE811_INT_EN, (INT_EN_GLOBAL | INT_EN_TYPE | INT_EN_POL));
  /* Confugure ADC */
  PE811_Write(PE811_ADC_CTRL1, (ADC_CTRL1_SAMPLE_TIME_80 | ADC_CTRL1_MOD_12B));
  Delay(10);
  /* Set ADC frequency */
  PE811_Write(PE811_ADC_CTRL2, ADC_CTRL2_FREQ_3_25);
  /* Set GPIO pins as touch screen pins */
  PE811_Write(PE811_GPIO_AF, (~(GPIO_ALT_FUNCT_0 | GPIO_ALT_FUNCT_1 | GPIO_ALT_FUNCT_2 | GPIO_ALT_FUNCT_3) & 0x0f));
  /* Configure touch screen */
  PE811_Write(PE811_TSC_CFG, (TSC_CFG_AVE_8 | TSC_CFG_DELAY_5ms | TSC_CFG_SETTLING_50ms));
  /* Set threshold */
  PE811_Write(PE811_FIFO_TH, 0xf0);
  /* Reset FIFO */
  PE811_Write(PE811_FIFO_STA, FIFO_STA_RESET);
  PE811_Write(PE811_FIFO_STA, 0x00);
  /* Set well accuracy of a pressure measurement */
  PE811_Write(PE811_TSC_FRACTION_Z, TSC_FRACTION_Z_7_1); 
  /* Set current of the controller - 50 mA typical */
  PE811_Write(PE811_TSC_I_DRIVE, TSC_I_DRIVE_50);
  /* Enable touch screen with 3-axises and no-trakong */
  PE811_Write(PE811_TSC_CTRL, (TSC_CTRL_OP_MODE_XYZ | TSC_CTRL_TRACK_0 | TSC_CTRL_EN)); 
  /* Enable temperature sensor */
  PE811_Write(PE811_TEMP_CTRL, (TEMP_CTRL_EN | TEMP_CTRL_AQU | TEMP_CTRL_AQU_MOD)); 
  /* Clear interrups' statuses */
  PE811_Write(PE811_INT_STA, (INT_STA_TOUCH_DET | INT_STA_FIFO_TH));
  /* Set global interrupt */
  PE811_Write(PE811_INT_CTRL, INT_EN_GLOBAL);

  /* Display zeroes */
  uint8_t tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  DisplayCoordinates(tmp);
}





/**
  * @brief  Reads data into buffer
  * @param  reg: A register to be read.
  * @param  buf: A pointer of a buffer where the data to be write.
  * @param  buf: The length of a buffer.
  * @return  a received byte
  */
uint8_t PE811_Read(uint8_t reg, uint8_t *buf, uint16_t len) {
  return (I2C_Read(I2C3, PE_I2C_Addr, reg, buf, len));
}





/**
  * @brief  Writes a command
  * @param  reg: A register to be written
  * @param  cmd: A command to write into the register
  * @return None
  */
uint8_t PE811_Write(uint8_t reg, uint8_t cmd) {
  uint8_t tmp[2] = {reg, cmd};
  return (I2C_Write(I2C3, PE_I2C_Addr, tmp, 2));
}

