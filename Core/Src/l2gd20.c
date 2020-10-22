/**
  ******************************************************************************
  * File Name          : l3gd20.c
  * Description        : This file provides code for the configuration
  *                      of the L3GD20 Hyroscope.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "l3gd20.h"







////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initializes L3GD20 MEMS device
  * @param  None
  * @return None
  */
void L3GD20_Init(void) {
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  #define MEMS_INT_MASK     (MEMS_INT1_Pin_Mask | MEMS_INT2_Pin_Mask)
  /* Mode */
  MODIFY_REG(MEMS_INT_Port->MODER, MEMS_INT_MASK, (
        (_MODE_IN << MEMS_INT1_Pin_Pos * 2)
      | (_MODE_IN << MEMS_INT2_Pin_Pos * 2)
    )
  );
  /* Speed */
  /* Output type */
  /* Push mode */
  /* Alternate function */


  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* Mode */
  MODIFY_REG(MEMS_CS_Port->MODER, MEMS_CS_Pin_Mask, (_MODE_OUT << MEMS_CS_Pin_Pos * 2));
  /* Speed */
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MEMS_CS_H;
  Delay(1);


  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  uint8_t buf[12];
  
  /* Enable clock on all axises with the lowest frequency */
  buf[0] = L3GD20_CTRL_REG_1;
  buf[1] = (CTRL_REG_1_XEN | CTRL_REG_1_YEN | CTRL_REG_1_ZEN | CTRL_REG_1_PD | CTRL_REG_1_DR_BW_95_12_5);
  L3GD20_Write(buf, 1);
  
  /* Set scale to 2000dps */
  buf[0] = L3GD20_CTRL_REG_4;
  buf[1] = CTRL_REG_4_FS_2000;
  L3GD20_Write(buf, 1);
  
  /* Set High-pass filter mode and High-pass filter cut */
  buf[0] = L3GD20_CTRL_REG_2;
  buf[1] = CTRL_REG_2_HPM_NORM;
  L3GD20_Write(buf, 1);
  
  /* Enable High-pass filter and FIFO buffer */
  buf[0] = L3GD20_CTRL_REG_5;
  buf[1] = (CTRL_REG_5_HP_EN | CTRL_REG_5_FIFO_EN);
  L3GD20_Write(buf, 1);
  
  /* Setting Bypass mode clears FIFO data (for INT2 interrupt) */
  buf[0] = L3GD20_FIFO_CTRL_REG;
  buf[1] = 0x00;
  L3GD20_Write(buf, 1);

  /* Set FIFO mode and watermark level (for INT2 interrupt) */
  buf[0] = L3GD20_FIFO_CTRL_REG;
  buf[1] = (FIFO_CTRL_REG_FM_FIFO | MEMS_WATERMARK);
  L3GD20_Write(buf, 1);

  /* Set interrupts */
  buf[0] = L3GD20_CTRL_REG_3;
  buf[1] = CTRL_REG_3_I2_WTM; // FIFO watermark interrupt (compare FIFO with threshold)
  // buf[1] = CTRL_REG_3_I1_INT; // programmed INT1 interrupt set
  // buf[1] = (CTRL_REG_3_I1_INT | CTRL_REG_3_I2_WTM); // Multiple interrupt setup
  L3GD20_Write(buf, 1);

  /* Counfugire INT1 interrupts (INT1_CFG_LIR is the necessary attribute) */
  buf[0] = L3GD20_INT1_CFG;
  buf[1] = (INT1_CFG_XLIE | INT1_CFG_XHIE | INT1_CFG_YLIE | INT1_CFG_YHIE | INT1_CFG_ZLIE | INT1_CFG_ZHIE | INT1_CFG_LIR);
  L3GD20_Write(buf, 1);
  
  /* Reading status clears interrupt pendong bits (for INT1 interrupt) */
  buf[0] = L3GD20_INT1_SRC;
  L3GD20_Read(buf, 1);

}





/**
  * @brief  Receive data from the MEMS.
  * @param  buf: a link of a data buffer
  * @param  cnt: length of the buffer 
  * @return None
  */
void L3GD20_Read(uint8_t *buf, uint8_t cnt) {
  /* Set Read bit in address */
  buf[0] = buf[0] | 0x80; 
  /* Set continuous reading if necessary */
  if (cnt > 1) buf[0] = buf[0] | 0x40;
  cnt += 1;

  MEMS_CS_L;
  SPI_Transfer8bBuf(SPI5, buf, cnt);
  MEMS_CS_H;
}





/**
  * @brief  Send data to the MEMS.
  * @param  buf: a link of a data buffer
  * @param  cnt: length of thr buffer 
  * @return None
  */
void L3GD20_Write(uint8_t *buf, uint8_t cnt) {
  /* Set continuous writing if necessary */
  if (cnt > 1) buf[0] = buf[0] | 0x40;
  cnt += 1;

  MEMS_CS_L;
  SPI_Transfer8bBuf(SPI5, buf, cnt);
  MEMS_CS_H;
}
