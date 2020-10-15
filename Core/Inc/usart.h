/**
  ******************************************************************************
  * File Name          : usart.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#ifndef __USART_H
#define __USART_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Global variables ----------------------------------------------------------*/
extern uint32_t _USARTREG_;


/* Private defines -----------------------------------------------------------*/
#define TX1_Pin             GPIO_PIN_9
#define TX1_Pin_Pos         GPIO_PIN_9_Pos
#define RX1_Pin             GPIO_PIN_10
#define RX1_Pin_Pos         GPIO_PIN_10_Pos
#define USART1_Port         GPIOA

#define RXBUF_LEN           64
#define RXBUF_MASK          (RXBUF_LEN - 1)


// USART Registry Flags
#define _USART1_LBNEF_      0 // Local Buffer Not Empty Flag
#define _USART1_RXAF_       1 // RX Action Flag



/* Private macros ------------------------------------------------------------*/
#define BAUDRATE        115200U
// #define BRR_Value       0x030d
#define BRR_Mantissa    (uint16_t)((RccClocks.PCLK2_Freq / (BAUDRATE * 16)) << 4)
#define BRR_Fractal     (uint16_t)((((RccClocks.PCLK2_Freq) / (BAUDRATE * 0.16)) - ((RccClocks.PCLK2_Freq / (BAUDRATE * 16)) * 100)) * 16 / 100)
#define BRR_Value       (uint16_t)((BRR_Mantissa & 0xfff0) | (BRR_Fractal & 0x000f))

/* Exported functions prototypes ---------------------------------------------*/
void USART1_Init(void);
void USART1_RX_Handler(void);
uint8_t USART1_RxBufferRead(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /*__ USART_H */

