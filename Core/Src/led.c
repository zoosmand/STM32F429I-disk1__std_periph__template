/**
  ******************************************************************************
  * File Name          : led.c
  * Description        : This file provides code for the configuration
  *                      of the GPIO instances for LEDs indication.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
  
/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* Global variables ---------------------------------------------------------*/
/**
  * @brief  A management restry keepeng flag states for the LED routines.
  */
uint32_t _LEDREG_ = 0;



/**
  * @brief  Initiates PIN 13 and 14 on GPIOG port as outputs to blink correspondent LEDs
  *  on the board. Pins are set on the lowest speed, push-pull, no pull-up neither pull-down
  * @param  None
  * @return None
  */
void LED_Init(void) {
    /* GPIO LED -  PG13, PG14 */
  /* Mode */
  MODIFY_REG(GPIOG->MODER, (GPIO_PIN_13_Mask | GPIO_PIN_14_Mask), ((_MODE_OUT << GPIO_PIN_13_Pos * 2) | (_MODE_OUT << GPIO_PIN_14_Pos * 2)));
  /* Speed */
  // MODIFY_REG(GPIOG->OSPEEDR, (GPIO_PIN_13_Mask | GPIO_PIN_14_Mask), ((_SPEED_L << GPIO_PIN_13_Pos * 2) | (_SPEED_L << GPIO_PIN_14_Pos * 2)));
  /* Output type */
  // MODIFY_REG(GPIOG->OTYPER, ((_OTYPE_PP << GPIO_PIN_13_Pos) | (_OTYPE_PP << GPIO_PIN_14_Pos)), ((_OTYPE_PP << GPIO_PIN_13_Pos) | (_OTYPE_PP << GPIO_PIN_14_Pos)));
  /* Push mode */
  // MODIFY_REG(GPIOG->PUPDR, (GPIO_PIN_13_Mask | GPIO_PIN_14_Mask), ((_PUPD_NO << GPIO_PIN_13_Pos * 2) | (_PUPD_NO << GPIO_PIN_14_Pos * 2)));
}


/**
  * @brief  Switches a certain PIN on a certain PORT on and off.
  * @param  port: a pointer to a GPIO port
  * @param  pinSource: a number of a pin in the port
  * @return None
  */
void LED_Blink(GPIO_TypeDef* port, uint16_t pinSource) {
  (PIN_LEVEL(port, pinSource)) ? PIN_L(port, pinSource) : PIN_H(port, pinSource);
}