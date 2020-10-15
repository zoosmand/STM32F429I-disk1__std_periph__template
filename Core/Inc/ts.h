/**
  ******************************************************************************
  * File Name          : ts.h
  * Description        : This file provides code for the configuration
  *                      of the TouchScreen instances.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __TS_H
#define __TS_H

#ifdef __cplusplus
  extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ----------------------------------------------------------*/
extern uint32_t _TSREG_;


/* Private defines -----------------------------------------------------------*/
#define _TEVF_      0 // Touch Event Flag


/* Exported functions prototypes ---------------------------------------------*/
void TS_Handler(void);




#ifdef __cplusplus
}
#endif

#endif /* __TS_H */