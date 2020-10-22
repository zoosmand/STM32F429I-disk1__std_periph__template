/**
  ******************************************************************************
  * File Name          : usart.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* Global variables ---------------------------------------------------------*/
uint32_t _USARTREG_ = 0;

/* Private variables ---------------------------------------------------------*/
static uint8_t usartBuf[RXBUF_LEN];
static uint8_t usartBufPrtIn = 0;
static uint8_t usartBufPrtOut = 0;

/* Private function prototypes -----------------------------------------------*/










////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initializes USART1 peripheral 
  * @param  None
  * @return None
  */
void USART1_Init(void) {
  /* 
    PA9   ------> USART1_TX
    PA10  ------> USART1_RX 
  */

  /* Type [0] - push-pull */
  /* Speed [11] - very high speed */
  PREG_SET(USART1_Port->OSPEEDR, BIT_2_0(TX1_Pin_Pos));
  PREG_SET(USART1_Port->OSPEEDR, BIT_2_1(TX1_Pin_Pos));
  /* Pull [00] - no pull-up, no pull-down */
  /* Alternate [0111] - AF7 */
  PREG_SET(USART1_Port->AFR[1], BIT_4_0((TX1_Pin_Pos -8)));
  PREG_SET(USART1_Port->AFR[1], BIT_4_1((TX1_Pin_Pos - 8)));
  PREG_SET(USART1_Port->AFR[1], BIT_4_2((TX1_Pin_Pos - 8)));
  /* Mode [10] - alternative */
  PREG_SET(USART1_Port->MODER, BIT_2_1(TX1_Pin_Pos));

  /* Type [0] - push-pull */
  /* Speed [11] - very high speed */
  PREG_SET(USART1_Port->OSPEEDR, BIT_2_0(RX1_Pin_Pos));
  PREG_SET(USART1_Port->OSPEEDR, BIT_2_1(RX1_Pin_Pos));
  /* Pull [00] - no pull-up, no pull-down */
  /* Alternate [0111] - AF7 */
  PREG_SET(USART1_Port->AFR[1], BIT_4_0((RX1_Pin_Pos - 8)));
  PREG_SET(USART1_Port->AFR[1], BIT_4_1((RX1_Pin_Pos - 8)));
  PREG_SET(USART1_Port->AFR[1], BIT_4_2((RX1_Pin_Pos - 8)));
  /* Mode [10] - alternative */
  PREG_SET(USART1_Port->MODER, BIT_2_1(RX1_Pin_Pos));

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* Transmit enable */
  PREG_SET(USART1->CR1, USART_CR1_TE_Pos);
  /* Receive enable */
  PREG_SET(USART1->CR1, USART_CR1_RE_Pos);
  /* Asyncronos mode */
  PREG_CLR(USART1->CR2, USART_CR2_LINEN_Pos);
  PREG_CLR(USART1->CR2, USART_CR2_CLKEN_Pos);
  PREG_CLR(USART1->CR3, USART_CR3_SCEN);
  PREG_CLR(USART1->CR3, USART_CR3_IREN);
  PREG_CLR(USART1->CR3, USART_CR3_HDSEL);
  /* Set Baudrate */
  USART1->BRR = BRR_Value;
  /* Enable USART1 */
  PREG_SET(USART1->CR1, USART_CR1_UE_Pos);
  /* Enable RXNE Interrupt */
  PREG_SET(USART1->CR1, USART_CR1_RXNEIE_Pos);

}







void USART1_RX_Handler() {
  uint8_t tmp = USART1->DR;
  uint8_t preRxPtr = usartBufPrtIn - 1;
  preRxPtr &= RXBUF_MASK;

  usartBuf[(usartBufPrtIn++)] = tmp;
  usartBufPrtIn &= RXBUF_MASK;

  // the end of message
  if ((usartBuf[preRxPtr] == 0x0d) && (tmp == 0x0a)) {
  // if (tmp == '\n') { // 0x0a
    // if (usartBuf[preRxPtr] == '\r') { // 0x0d
      FLAG_SET(_USARTREG_, _USART1_LBNEF_);
    // }
  }
}





uint8_t USART1_RxBufferRead(uint8_t *buf, uint16_t len) {
  uint8_t payloadLen = 0;
  while (usartBufPrtOut != usartBufPrtIn) {
    buf[(payloadLen++)] = usartBuf[(usartBufPrtOut++)];
    usartBufPrtOut &= RXBUF_MASK;
  }
  return payloadLen;
}

