/**
  ******************************************************************************
  * File Name          : led.h
  * Description        : This file provides code for the configuration
  *                      of the GPIO instances for LED's indication.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
  
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Global variables ----------------------------------------------------------*/
extern uint32_t _LEDREG_;


/* Private defines -----------------------------------------------------------*/
#define LED0_Pin        GPIO_PIN_13
#define LED0_Pin_Pos    GPIO_PIN_13_Pos
#define LED0_Port       GPIOG
#define LED1_Pin        GPIO_PIN_14
#define LED1_Pin_Pos    GPIO_PIN_14_Pos
#define LED1_Port       GPIOG


// LED Registry Flags
#define _LED0UF_     0 // LED0 Up Flag
#define _LED0DF_     1 // LED0 Down Flag
#define _LED1UF_     2 // LED1 Up Flag
#define _LED1DF_     3 // LED1 Down Flag



/* Exported functions prototypes ---------------------------------------------*/
void LED_Init(void);
void LED_Blink(GPIO_TypeDef* port, uint16_t pinSource);



#ifdef __cplusplus
}
#endif
#endif /*__LED_H */
