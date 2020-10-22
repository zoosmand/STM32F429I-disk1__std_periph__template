/**
  ******************************************************************************
  * File Name          : i2c.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* Global variables ---------------------------------------------------------*/
uint32_t _I2CREG_;


/* -------------------------------------------------------------------------- */
/* Private function prototypes -----------------------------------------------*/
/* -------------------------------------------------------------------------- */
static uint8_t I2C_GenerateStart(I2C_TypeDef *I2Cx);
static void I2C_GenerateStop(I2C_TypeDef *I2Cx);
static uint8_t I2C_SendAddress(I2C_TypeDef *I2Cx, uint16_t slaveAddress, I2C_AddressMode addressMode, I2C_Direction dir);









////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initializes I2C3 peripheral
  * @param  None
  * @return None
  */
void I2C3_Init(void) {
  /* SDA */
  /* ----------------------------------------------------------------------------- */
  /* Mode */
  MODIFY_REG(I2C_3_SDA_Port->MODER, I2C_3_SDA_Pin_Mask, (_MODE_AF << I2C_3_SDA_Pin_Pos * 2));
  /* Speed */
  MODIFY_REG(I2C_3_SDA_Port->OSPEEDR, I2C_3_SDA_Pin_Mask, (_SPEED_M << I2C_3_SDA_Pin_Pos * 2));
  /* Output type */
  MODIFY_REG(I2C_3_SDA_Port->OTYPER, I2C_3_SDA_Pin, (_OTYPE_OD << I2C_3_SDA_Pin_Pos));
  /* Push mode */
  MODIFY_REG(I2C_3_SDA_Port->PUPDR, I2C_3_SDA_Pin_Mask, (_PUPD_PU << I2C_3_SDA_Pin_Pos * 2));
  /* Alternate function */
  MODIFY_REG(I2C_3_SDA_Port->AFR[1], 0x000000f0, (GPIO_AF_4 << (I2C_3_SDA_Pin_Pos - 8) * 4));

  /* SCL */
  /* Mode */
  MODIFY_REG(I2C_3_SCL_Port->MODER, I2C_3_SCL_Pin_Mask, (_MODE_AF << I2C_3_SCL_Pin_Pos * 2));
  /* Speed */
  MODIFY_REG(I2C_3_SCL_Port->OSPEEDR, I2C_3_SCL_Pin_Mask, (_SPEED_M << I2C_3_SCL_Pin_Pos * 2));
  /* Output type */
  MODIFY_REG(I2C_3_SCL_Port->OTYPER, I2C_3_SCL_Pin, (_OTYPE_OD << I2C_3_SCL_Pin_Pos));
  /* Push mode */
  MODIFY_REG(I2C_3_SCL_Port->PUPDR, I2C_3_SCL_Pin_Mask, (_PUPD_PU << I2C_3_SCL_Pin_Pos * 2));
  /* Alternate function */
  MODIFY_REG(I2C_3_SCL_Port->AFR[1], 0x0000000f, (GPIO_AF_4 << (I2C_3_SCL_Pin_Pos - 8) * 4));


  /* Due to a undocumented bug, the source clock should be start after the GPIO initialization */
  Delay(1);
  PREG_SET(RCC->APB1ENR, RCC_APB1ENR_I2C3EN_Pos);


  // I2C
  /* ----------------------------------------------------------------------------- */
  /* Switch off */
  PREG_CLR(I2C3->CR1, I2C_CR1_PE_Pos);
  __NOP();
  __NOP();
  // /* Switch on */
  // PREG_SET(I2C3->CR1, I2C_CR1_PE_Pos);
  /* Force reset I2C3 */
  PREG_SET(RCC->APB1RSTR, RCC_APB1ENR_I2C3EN_Pos);
  PREG_CLR(RCC->APB1RSTR, RCC_APB1ENR_I2C3EN_Pos);
  /* Set peripheral clock freq in MHz */
  MODIFY_REG(I2C3->CR2, 0x0000003f, 45);

  I2C3->OAR1 = 0x00004000;

  /* Analog filter off */
  PREG_SET(I2C3->FLTR, I2C_FLTR_ANOFF_Pos);
  /* Set TRISE */
  I2C3->TRISE = 0x2e; // 100kHz
  // I2C3->TRISE = 0x0c; // 400kHz
  /* Set master freq */
  /* 100kHz gives 5000ns rising of falling time, PCLK1_Freq (45MHz) gives 22.22ns period time of carruing freq, CCR <=> 5000 / 22.22 = 225 */
  /* 400kHz gives 1250ns rising of falling time, PCLK1_Freq (45MHz) gives 22.22ns period time of carruing freq, CCR <=> 1250 / 22.22 = 56 */
  MODIFY_REG(I2C3->CCR, 0x0fff, 0xe1); // 100kHz
  // MODIFY_REG(I2C3->CCR, 0x0fff, 0x38); // 400kHz
  /* Switch on */
  PREG_SET(I2C3->CR1, I2C_CR1_PE_Pos);

  /* Switch I2C on */
  PREG_SET(I2C3->CR1, I2C_CR1_PE_Pos);

}






/**
  * @brief  Writes 8 bit data via I2C
  * @param  I2Cx: pointer to an I2C instance
  * @param  data: a byte to send
  * @return None
  */
uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t *buf, uint16_t len) {
  uint8_t status = 0;

  if (I2C_GenerateStart(I2Cx)) {
    // if (I2C_Master_Transmit(I2Cx, slaveAddress, &reg, 1)) {
      status = I2C_Master_Transmit(I2Cx, slaveAddress, buf, len);
    // }
    I2C_GenerateStop(I2Cx);
  }

 return (status);
}




/**
  * @brief  Reads 8 bit data via I2C
  * @param  I2Cx: pointer to an I2C instance
  * @return  a received byte
  */
uint8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t reg, uint8_t *buf, uint16_t len) {
  uint8_t status = 0;

  if (I2C_GenerateStart(I2Cx)) {
    if (I2C_Master_Transmit(I2Cx, slaveAddress, &reg, 1)) {
      if (I2C_GenerateStart(I2Cx)) {
        status = I2C_Master_Receive(I2Cx, slaveAddress, buf, len);
      }
    }
    I2C_GenerateStop(I2Cx);
  }

  return (status);
}




/**
  * @brief  Transfers 8 bit data via I2C (read and write in one iteraction)
  * @param  I2Cx: pointer to an I2C instance
  * @param  data: a byte to send
  * @return a received byte
  */
uint8_t I2C_Transfer8b(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t data) {

  return (data);
}




/**
  * @brief  Transmits in master mode an amount of data.
  * @param  I2Cx Pointer to a I2C_TypeDef I2Cx.
  * @param  slaveAddress Target I2Cx address: The I2Cx 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  buf Pointer to data buffer
  * @param  len Amount of data to be sent
  * @return transmit status
  */
uint8_t I2C_Master_Transmit(I2C_TypeDef *I2Cx, uint16_t slaveAddress, uint8_t *buf, uint16_t len) {
  uint8_t status = 0;
  uint32_t timeout = 5000000;
  
  /* Disable Pos */
  PREG_CLR(I2Cx->CR1, I2C_CR1_POS_Pos);
  /* Disable Ack */
  PREG_CLR(I2Cx->CR1, I2C_CR1_ACK_Pos);


  if (I2C_SendAddress(I2Cx, slaveAddress, mode_7bit, transmit)) {
    while (len--) {
      I2Cx->DR = *buf++;
      while (!(PREG_CHECK(I2Cx->SR1, I2C_SR1_TXE_Pos))) {
        timeout--;
        if (!timeout) {
          status = 0;
          return (status);
        }
      }

      if ((PREG_CHECK(I2Cx->SR1, I2C_SR1_AF_Pos)) || (PREG_CHECK(I2Cx->SR1, I2C_SR1_ARLO_Pos)) || (PREG_CHECK(I2Cx->SR1, I2C_SR1_BERR_Pos))) {
        status = 0;
      } else {
        status = 1;
      }
    }
  }

  return (status);
}






/**
  * @brief  Receives in master mode an amount of data.
  * @param  I2Cx Pointer to a I2C_TypeDef I2Cx.
  * @param  slaveAddress Target I2Cx address: The I2Cx 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  buf Pointer to data buffer
  * @param  len Amount of data to be sent
  * @return receive status
  */
uint8_t I2C_Master_Receive(I2C_TypeDef *I2Cx, uint16_t slaveAddress, uint8_t *buf, uint16_t len) {
  uint8_t status = 0;
  uint32_t timeout = 5000000;
  
  /* Disable Pos */
  PREG_CLR(I2Cx->CR1, I2C_CR1_POS_Pos);
  /* Enable Ack */
  PREG_SET(I2Cx->CR1, I2C_CR1_ACK_Pos);
  // CLR_I2C_BIT(&I2Cx->CR1, I2C_CR1_ACK_Pos);

  if (I2C_SendAddress(I2Cx, slaveAddress, mode_7bit, receive)) {
    while (len--) {
      // last byte without Acking
      if (len == 0) {
        PREG_CLR(I2Cx->CR1, I2C_CR1_ACK_Pos);
      }

      while (!(PREG_CHECK(I2Cx->SR1, I2C_SR1_RXNE_Pos))) {
        timeout--;
        if (!timeout) {
          status = 0;
          return (status);
        }
      }
      *buf++ = I2Cx->DR;

      if ((PREG_CHECK(I2Cx->SR1, I2C_SR1_AF_Pos)) || (PREG_CHECK(I2Cx->SR1, I2C_SR1_ARLO_Pos)) || (PREG_CHECK(I2Cx->SR1, I2C_SR1_BERR_Pos))) {
        status = 0;
      } else {
        status = 1;
      }
    }
  }

  return (status);
}






static uint8_t I2C_SendAddress(I2C_TypeDef *I2Cx, uint16_t slaveAddress, I2C_AddressMode addressMode, I2C_Direction dir) {
  uint8_t status = 0;
  uint32_t timeout = 5000000;

  if (addressMode == mode_7bit) {
    I2Cx->DR = (slaveAddress | dir);
    while (!(PREG_CHECK(I2Cx->SR1, I2C_SR1_ADDR_Pos))) {
      timeout--;
      if (!timeout) {
        status = 0;
        return (status);
      }
    }
    if (PREG_CHECK(I2Cx->SR2, I2C_SR2_MSL_Pos)) {
      status = 1;
    }
  } else {
    // ToDo. 10-bit address handler
  }

  return (status);
}





uint8_t I2C_GenerateStart(I2C_TypeDef *I2Cx) {
  uint8_t status = 1;
  uint32_t timeout = 5000000;

   /* Generate Start or ReStart */
  PREG_SET(I2Cx->CR1, I2C_CR1_START_Pos);

  while (!(PREG_CHECK(I2Cx->SR1, I2C_SR1_SB_Pos))) {
    timeout--;
    if (!timeout) {
      status = 0;
      return (status);
    }
  }
  
  return (status);
}





void I2C_GenerateStop(I2C_TypeDef *I2Cx) {
  PREG_SET(I2Cx->CR1, I2C_CR1_STOP_Pos);
}