/**
  ******************************************************************************
  * File Name          : spi.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* Global variables ---------------------------------------------------------*/
uint32_t _SPIREG_ = 0;







////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initializes SPI5 peripheral 
  * @param  None
  * @return None
  */
void SPI5_Init(void) {
  /* Mode */
  #define SPI_5_MASK  (SPI_5_SCK_Pin_Mask | SPI_5_MISO_Pin_Mask | SPI_5_MOSI_Pin_Mask)
  /* Mode */
  MODIFY_REG(SPI_5_SPI_Port->MODER, SPI_5_MASK, (
      (_MODE_AF << SPI_5_SCK_Pin_Pos * 2)
    | (_MODE_AF << SPI_5_MISO_Pin_Pos * 2)
    | (_MODE_AF << SPI_5_MOSI_Pin_Pos * 2)
  ));
  /* Speed */
  MODIFY_REG(SPI_5_SPI_Port->OSPEEDR, SPI_5_MASK, (
      (_SPEED_V << SPI_5_SCK_Pin_Pos * 2)
    | (_SPEED_V << SPI_5_MISO_Pin_Pos * 2)
    | (_SPEED_V << SPI_5_MOSI_Pin_Pos * 2)
  ));
  /* Alternate function */
  MODIFY_REG(SPI_5_SPI_Port->AFR[0], 0xf0000000, (GPIO_AF_5 << SPI_5_SCK_Pin_Pos * 4));
  MODIFY_REG(SPI_5_SPI_Port->AFR[1], 0x000000ff, (
      (GPIO_AF_5 << (SPI_5_MISO_Pin_Pos - 8) * 4)
    | (GPIO_AF_5 << (SPI_5_MOSI_Pin_Pos - 8) * 4)
  ));
  
  
  /* Master enable */
  PREG_SET(SPI5->CR1, SPI_CR1_MSTR_Pos);
  /* Internal Slave Select enable */
  PREG_SET(SPI5->CR1, SPI_CR1_SSI_Pos);
  /* NSS managed internally. NSS pin not used and free */
  PREG_SET(SPI5->CR1, SPI_CR1_SSM_Pos);
  /* Bitrate divided to 16 */
  PREG_SET(SPI5->CR1, SPI_CR1_BR_Pos);
  PREG_SET(SPI5->CR1, (SPI_CR1_BR_Pos + 1));
  /* SPI5 enable */
  PREG_SET(SPI5->CR1, SPI_CR1_SPE_Pos);
}




/**
  * @brief  Writes 8 bit data via SPI
  * @param  SPIx: pointer to an SPI instance
  * @param  data: a byte to send
  * @return None
  */
void SPI_Write8b(SPI_TypeDef *SPIx, uint8_t data) {
  __IO uint8_t *spidr = (__IO uint8_t *)&SPIx->DR;
  *spidr = data;
  while (!(PREG_CHECK(SPIx->SR, SPI_SR_TXE_Pos)));
}




/**
  * @brief  Reads 8 bit data via SPI
  * @param  SPIx: pointer to an SPI instance
  * @return  a received byte
  */
uint8_t SPI_Read8b(SPI_TypeDef *SPIx) {
  uint8_t data_ret = (__IO uint8_t)(SPIx->DR);
  while (PREG_CHECK(SPIx->SR, SPI_SR_RXNE_Pos));
  return (data_ret);
}




/**
  * @brief  Transfers 8 bit data via SPI (read and write in one iteraction)
  * @param  SPIx: pointer to an SPI instance
  * @param  data: a byte to send
  * @return a received byte
  */
uint8_t SPI_Transfer8b(SPI_TypeDef *SPIx, uint8_t data) {
  __IO uint8_t *spidr = (__IO uint8_t *)&SPIx->DR;
  *spidr = data;
  while (!(PREG_CHECK(SPIx->SR, SPI_SR_TXE_Pos)));
  Delay(1);
  uint8_t data_ret = (__IO uint8_t)(SPIx->DR);
  while (PREG_CHECK(SPIx->SR, SPI_SR_RXNE_Pos));
  return (data_ret);
}






/**
  * @brief  Two-way transfer 8 bit data via SPI into/from a buffer
  * @param  SPIx: pointer to an SPI instance
  * @param  buf: pointer to a buffer to store data
  * @param  cnt: counter of bytes to get
  * @return  a received byte
  */
void SPI_Transfer8bBuf(SPI_TypeDef *SPIx, uint8_t *buf, uint16_t cnt) {
  while (cnt--) {
    __IO uint8_t *spidr = (__IO uint8_t *)&SPIx->DR;
    *spidr = *buf;
    while (!(PREG_CHECK(SPIx->SR, SPI_SR_TXE_Pos)));
    Delay(1);
    *buf = (__IO uint8_t)(SPIx->DR);
    while (PREG_CHECK(SPIx->SR, SPI_SR_RXNE_Pos));
    buf++;
  }
 
}
