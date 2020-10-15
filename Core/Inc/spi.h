/**
  ******************************************************************************
  * File Name          : spi.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
  extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Global variables ----------------------------------------------------------*/
extern uint32_t _SPIREG_;

/* Private defines -----------------------------------------------------------*/
// #define SPI_5_CS_Pin          GPIO_PIN_6
// #define SPI_5_C5_Pin_Pos      GPIO_PIN_6_Pos
// #define SPI_5_C5_Pin_Mask     GPIO_PIN_6_Mask

#define SPI_5_SCK_Pin         GPIO_PIN_7
#define SPI_5_SCK_Pin_Pos     GPIO_PIN_7_Pos
#define SPI_5_SCK_Pin_Mask    GPIO_PIN_7_Mask
#define SPI_5_MISO_Pin        GPIO_PIN_8
#define SPI_5_MISO_Pin_Pos    GPIO_PIN_8_Pos
#define SPI_5_MISO_Pin_Mask   GPIO_PIN_8_Mask
#define SPI_5_MOSI_Pin        GPIO_PIN_9
#define SPI_5_MOSI_Pin_Pos    GPIO_PIN_9_Pos
#define SPI_5_MOSI_Pin_Mask   GPIO_PIN_9_Mask
#define SPI_5_SPI_Port        GPIOF

// SPI Registry Flags
#define _SPI5_LBNEF_    0 // Local Buffer Not Empty Flag
#define _SPI5_RXAF_     1 // RX Action Flag


/* Exported functions prototypes ---------------------------------------------*/
void SPI5_Init(void);
void SPI_Write8b(SPI_TypeDef *SPIx, uint8_t data);
uint8_t SPI_Read8b(SPI_TypeDef *SPIx);
uint8_t SPI_Transfer8b(SPI_TypeDef *SPIx, uint8_t data);
void SPI_Transfer8bBuf(SPI_TypeDef *SPIx, uint8_t *buf, uint16_t cnt);


#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
