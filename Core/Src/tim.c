/**
  ******************************************************************************
  * File Name          : tim.c
  * Description        : This file provides code for the configuration
  *                      of timers instances.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* Global variables ---------------------------------------------------------*/
uint32_t _TIMREG_ = 0;









////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initialises Basic Timer7
  * @param  None
  * @return None
  */
void BasicTimer7_Init(void) {
  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  TIM7->PSC = 9000U - 1U; // APB1 runs on max freq of SystemCoreClock/2 (here 90 MHz)
  TIM7->ARR = 10000U - 1U;
  /* Autoreload enable */ 
  PREG_SET(TIM7->CR1, TIM_CR1_ARPE_Pos);

  /* Interrupt enabling passes */
  PREG_CLR(TIM7->SR, TIM_SR_UIF_Pos);
  PREG_SET(TIM7->DIER, TIM_DIER_UIE_Pos);
  PREG_SET(TIM7->CR1, TIM_CR1_CEN_Pos);

}



/**
  * @brief  Basic Timer Handler called from interrupt via flag's handler
  * @param  tim: pointer to a basic timer instance
  * @return None
  */
void Timer_Handler(TIM_TypeDef* tim) {
  if (tim == TIM7) {
    // printf("tim7\n");
  }
}
