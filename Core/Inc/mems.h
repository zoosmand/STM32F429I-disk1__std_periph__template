/**
  ******************************************************************************
  * File Name          : mems.h
  * Description        : This file provides MEMS code for the Hyroscope.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMS_H
#define __MEMS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Global variables ----------------------------------------------------------*/
extern uint32_t _MEMSREG_;


/* Private defines -----------------------------------------------------------*/
#define _INT1F_      0 // L3DG20 INT1 Event Flag
#define _INT2F_      1 // L3DG20 INT2 Event Flag

/* Registers */



/* Exported functions prototypes ---------------------------------------------*/
void MEMS_Handler(void);


#ifdef __cplusplus
}
#endif
#endif /*__MEMS_H */
