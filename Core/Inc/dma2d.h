/**
  ******************************************************************************
  * File Name          : dma2d.h
  * Description        : This file provides code for the configuration
  *                      of the DMA2D instances.
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA2D_H
#define __DMA2D_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"



/* Private defines -----------------------------------------------------------*/
#define MODE_MM       0b00
#define MODE_MM_PFC   0b01
#define MODE_MM_B     0b10
#define MODE_RM       0b11


/* Exported functions prototypes ---------------------------------------------*/
void DMA2D_Init(void);


#ifdef __cplusplus
}
#endif
#endif /*__DMA2D_H */
