/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of timers instances.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ---------------------------------------------------------*/
extern uint32_t _TIMREG_;



/* Private defines -----------------------------------------------------------*/
// TIM Registry Flags
#define _BT7F_        0 // Basic Timer7 Action Flag




/* Exported functions prototypes ---------------------------------------------*/
void BasicTimer7_Init(void);
void Timer_Handler(TIM_TypeDef* tim);



#ifdef __cplusplus
}
#endif
#endif /*__TIM_H */

