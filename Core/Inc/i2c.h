/**
  ******************************************************************************
  * File Name          : i2c.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
  extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Global variables ----------------------------------------------------------*/
extern uint32_t _I2CREG_;

/* Private defines -----------------------------------------------------------*/
// PC9
#define I2C_3_SDA_Pin         GPIO_PIN_9
#define I2C_3_SDA_Pin_Pos     GPIO_PIN_9_Pos
#define I2C_3_SDA_Pin_Mask    GPIO_PIN_9_Mask
#define I2C_3_SDA_Port        GPIOC
// PA8
#define I2C_3_SCL_Pin         GPIO_PIN_8
#define I2C_3_SCL_Pin_Pos     GPIO_PIN_8_Pos
#define I2C_3_SCL_Pin_Mask    GPIO_PIN_8_Mask
#define I2C_3_SCL_Port        GPIOA

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  mode_7bit    = 0,
  mode_10bit
} I2C_AddressMode;

typedef enum {
  transmit    = 0,
  receive     = 1
} I2C_Direction;



/* Private macro -------------------------------------------------------------*/



/* Exported functions prototypes ---------------------------------------------*/
void I2C3_Init(void);
uint8_t I2C_Write(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t *buf, uint16_t len);
uint8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t I2C_Transfer8b(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t data);
uint8_t I2C_Master_Transmit(I2C_TypeDef *I2Cx, uint16_t slaveAddress, uint8_t *buf, uint16_t len);
uint8_t I2C_Master_Receive(I2C_TypeDef *I2Cx, uint16_t slaveAddress, uint8_t *buf, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
