/**
  ******************************************************************************
  * File Name          : pe811.h
  * Description        : This file provides code for the configuration
  *                      of the STMPE811 Touch Screen for a Display.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PE811_H
#define __PE811_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Global variables ----------------------------------------------------------*/


/* Private defines -----------------------------------------------------------*/
/* I2C Address */
#define PE_I2C_Addr                   0x82

/* STMPE811 Registers definition */
#define PE811_CHIP_ID                 0x00	//PE811 Device identification
#define PE811_ID_VER					        0x02	//PE811 Revision number; 0x01 for engineering sample; 0x03 for final silicon
#define PE811_SYS_CTRL1				        0x03	//Reset control
#define PE811_SYS_CTRL2				        0x04	//Clock control
#define PE811_SPI_CFG				          0x08	//SPI interface configuration
#define PE811_INT_CTRL				        0x09	//Interrupt control register
#define PE811_INT_EN					        0x0a	//Interrupt enable register
#define PE811_INT_STA				          0x0b	//Interrupt status register
#define PE811_GPIO_EN				          0x0c	//GPIO interrupt enable register
#define PE811_GPIO_INT_STA		        0x0d	//GPIO interrupt status register
#define PE811_ADC_INT_EN			        0x0e	//ADC interrupt enable register
#define PE811_ADC_INT_STA			        0x0f	//ADC interface status register
#define PE811_GPIO_SET_PIN		        0x10	//GPIO set pin register
#define PE811_GPIO_CLR_PIN		        0x11	//GPIO clear pin register
#define PE811_MP_STA					        0x12	//GPIO monitor pin state register
#define PE811_GPIO_DIR				        0x13	//GPIO direction register
#define PE811_GPIO_ED				          0x14	//GPIO edge detect register
#define PE811_GPIO_RE				          0x15	//GPIO rising edge register
#define PE811_GPIO_FE				          0x16	//GPIO falling edge register
#define PE811_GPIO_AF				          0x17	//alternate function register
#define PE811_ADC_CTRL1				        0x20	//ADC control
#define PE811_ADC_CTRL2				        0x21	//ADC control
#define PE811_ADC_CAPT				        0x22	//To initiate ADC data acquisition
#define PE811_ADC_DATA_CHO		        0x30	//ADC channel 0
#define PE811_ADC_DATA_CH1		        0x32	//ADC channel 1
#define PE811_ADC_DATA_CH2		        0x34	//ADC channel 2
#define PE811_ADC_DATA_CH3		        0x36	//ADC channel 3
#define PE811_ADC_DATA_CH4		        0x38	//ADC channel 4
#define PE811_ADC_DATA_CH5		        0x3a	//ADC channel 5
#define PE811_ADC_DATA_CH6		        0x3c	//ADC channel 6
#define PE811_ADC_DATA_CH7		        0x3e	//ADC channel 7
#define PE811_TSC_CTRL				        0x40	//4-wire touchscreen controller setup
#define PE811_TSC_CFG				          0x41	//Touchscreen controller configuration
#define PE811_WDW_TR_X				        0x42	//Window setup for top right X
#define PE811_WDW_TR_Y				        0x44	//Window setup for top right Y
#define PE811_WDW_BL_X				        0x46	//Window setup for bottom left X
#define PE811_WDW_BL_Y				        0x48	//Window setup for bottom left Y
#define PE811_FIFO_TH				          0x4a	//FIFO level to generate interrupt
#define PE811_FIFO_STA				        0x4b	//Current status of FIFO
#define PE811_FIFO_SIZE				        0x4c	//Current filled level of FIFO
#define PE811_TSC_DATA_X			        0x4d	//Data port for touchscreen controller data access
#define PE811_TSC_DATA_Y			        0x4f	//Data port for touchscreen controller data access
#define PE811_TSC_DATA_Z			        0x51	//Data port for touchscreen controller data access
#define PE811_TSC_DATA_XYZ		        0x52	//Data port for touchscreen controller data access
#define PE811_TSC_FRACTION_Z	        0x56	//Touchscreen controller FRACTION_Z
#define PE811_TSC_DATA				        0x57	//Data port for touchscreen controller data access
#define PE811_TSC_I_DRIVE			        0x58	//Touchscreen controller drivel
#define PE811_TSC_SHIELD			        0x59	//Touchscreen controller shield
#define PE811_TEMP_CTRL				        0x60	//Temperature sensor setup
#define PE811_TEMP_DATA				        0x61	//Temperature data access port
#define PE811_TEMP_TH				          0x62	//Threshold for temperature controlled interrupt

/* SYS_CTRL1 Bits Definition */
#define SYS_CTRL1_HIBERNATE           0x01
#define SYS_CTRL1_SF_RESET            0x02

/* SYS_CTRL2 Bits Definition */
#define SYS_CTRL2_ADC_OFF             (uint8_t)0x01
#define SYS_CTRL2_TSC_OFF             (uint8_t)0x02
#define SYS_CTRL2_GPIO_OFF            (uint8_t)0x04
#define SYS_CTRL2_TS_OFF              (uint8_t)0x08

/* INT_EN Bits Definition */
#define INT_EN_GLOBAL                 0x01
#define INT_EN_TYPE                   0x02 // 0: Level interrupt, 1: Edge interrupt
#define INT_EN_POL                    0x04 // 0: Falling edge, 1: Rising edge

/* ADC_CTRL1 Bits Definition */
#define ADC_CTRL1_REF_SEL             0x02 // 0: Internal reference, 1: External reference
#define ADC_CTRL1_MOD_12B             0x04 // 0: 10-bit ADC, 1: 12-bit ADC
#define ADC_CTRL1_SAMPLE_TIME_Pos     4
#define ADC_CTRL1_SAMPLE_TIME_36      (0b000 << ADC_CTRL1_SAMPLE_TIME_Pos)
#define ADC_CTRL1_SAMPLE_TIME_44      (0b001 << ADC_CTRL1_SAMPLE_TIME_Pos)
#define ADC_CTRL1_SAMPLE_TIME_56      (0b010 << ADC_CTRL1_SAMPLE_TIME_Pos)
#define ADC_CTRL1_SAMPLE_TIME_64      (0b011 << ADC_CTRL1_SAMPLE_TIME_Pos)
#define ADC_CTRL1_SAMPLE_TIME_80      (0b100 << ADC_CTRL1_SAMPLE_TIME_Pos)
#define ADC_CTRL1_SAMPLE_TIME_96      (0b101 << ADC_CTRL1_SAMPLE_TIME_Pos)
#define ADC_CTRL1_SAMPLE_TIME_124     (0b110 << ADC_CTRL1_SAMPLE_TIME_Pos)

/* ADC_CTRL2 Bits Definition */
#define ADC_CTRL2_FREQ_1_625           0b00
#define ADC_CTRL2_FREQ_3_25            0b01
#define ADC_CTRL2_FREQ_6_5             0b10
#define ADC_CTRL2_FREQ_6_5_            0b11

/* GPIO_ALT_FUNCT Bits Definition */
#define GPIO_ALT_FUNCT_0              0x01
#define GPIO_ALT_FUNCT_1              0x02
#define GPIO_ALT_FUNCT_2              0x04
#define GPIO_ALT_FUNCT_3              0x08
#define GPIO_ALT_FUNCT_4              0x10
#define GPIO_ALT_FUNCT_5              0x20
#define GPIO_ALT_FUNCT_6              0x40
#define GPIO_ALT_FUNCT_7              0x80

/* TSC_CFG Bits Definition */
#define TSC_CFG_SETTLING_10us         0b000
#define TSC_CFG_SETTLING_100us        0b001
#define TSC_CFG_SETTLING_500us        0b010
#define TSC_CFG_SETTLING_1ms          0b011
#define TSC_CFG_SETTLING_5ms          0b100
#define TSC_CFG_SETTLING_10ms         0b101
#define TSC_CFG_SETTLING_50ms         0b110
#define TSC_CFG_SETTLING_100ms        0b111
#define TSC_CFG_DELAY_Pos             3
#define TSC_CFG_DELAY_10us            (0b000 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_50us            (0b001 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_100us           (0b010 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_500us           (0b011 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_1ms             (0b100 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_5ms             (0b101 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_10ms            (0b110 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_DELAY_50ms            (0b111 << TSC_CFG_DELAY_Pos)
#define TSC_CFG_AVE_Pos               6
#define TSC_CFG_AVE_1                 (0b00 << TSC_CFG_AVE_Pos)
#define TSC_CFG_AVE_2                 (0b01 << TSC_CFG_AVE_Pos)
#define TSC_CFG_AVE_4                 (0b10 << TSC_CFG_AVE_Pos)
#define TSC_CFG_AVE_8                 (0b11 << TSC_CFG_AVE_Pos)

/* FIFO_STA Bits Definition */
#define FIFO_STA_RESET                0x01
#define FIFO_STA_TH_TRIG              0x10 // 0: Below the threshold value, 1: At or beyond the threshold value
#define FIFO_STA_EMPTY                0x20
#define FIFO_STA_FULL                 0x40
#define FIFO_STA_OFLOW                0x80

/* TSC_FRACTION_Z Bits Definition */
#define TSC_FRACTION_Z_0_8            0b000
#define TSC_FRACTION_Z_1_7            0b001
#define TSC_FRACTION_Z_2_6            0b010
#define TSC_FRACTION_Z_3_5            0b011
#define TSC_FRACTION_Z_4_4            0b100
#define TSC_FRACTION_Z_5_3            0b101
#define TSC_FRACTION_Z_6_2            0b110
#define TSC_FRACTION_Z_7_1            0b111

/* TSC_I_DRIVE Bits Definition */
#define TSC_I_DRIVE_20                0b0 // 20mA
#define TSC_I_DRIVE_50                0b1 // 50mA

/* TSC_CTRL Bits Definition */
#define TSC_CTRL_EN                   0x01
#define TSC_CTRL_OP_MODE_Pos          1
#define TSC_CTRL_OP_MODE_XYZ          (0b000 << TSC_CTRL_OP_MODE_Pos)
#define TSC_CTRL_OP_MODE_XY           (0b001 << TSC_CTRL_OP_MODE_Pos)
#define TSC_CTRL_OP_MODE_X            (0b010 << TSC_CTRL_OP_MODE_Pos)
#define TSC_CTRL_OP_MODE_Y            (0b011 << TSC_CTRL_OP_MODE_Pos)
#define TSC_CTRL_OP_MODE_Z            (0b100 << TSC_CTRL_OP_MODE_Pos)
#define TSC_CTRL_TRACK_Pos            4
#define TSC_CTRL_TRACK_0              (0b000 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_4              (0b001 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_8              (0b010 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_16             (0b011 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_32             (0b100 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_64             (0b101 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_92             (0b110 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_TRACK_127            (0b111 << TSC_CTRL_TRACK_Pos)
#define TSC_CTRL_STA                  0x80

/* TEMP_CTRL Bits Definition */
#define TEMP_CTRL_EN                  0x01
#define TEMP_CTRL_AQU                 0x02
#define TEMP_CTRL_AQU_MOD             0x04 // 0: to acquire temperature for once only, 1: to acquire temperature every 10ms
#define TEMP_CTRL_THRES_EN            0x08
#define TEMP_CTRL_THRES_RANGE         0x10 // 0: assert interrupt if temperature is >= threshold, 1: assert interrupt if otherwise

/* INT_EN Bits Definition */
#define INT_EN_TOUCH_DET              0x01
#define INT_EN_FIFO_TH                0x02
#define INT_EN_FIFO_OFLOW             0x04
#define INT_EN_FIFO_FULL              0x08
#define INT_EN_FIFO_EMPTY             0x10
#define INT_EN_TEMP_SENS              0x20
#define INT_EN_ADC                    0x40
#define INT_EN_GPIO                   0x80

/* INT_STA Bits Definition */
#define INT_STA_TOUCH_DET             0x01
#define INT_STA_FIFO_TH               0x02
#define INT_STA_FIFO_OFLOW            0x04
#define INT_STA_FIFO_FULL             0x08
#define INT_STA_FIFO_EMPTY            0x10
#define INT_STA_TEMP_SENS             0x20
#define INT_STA_ADC                   0x40
#define INT_STA_GPIO                  0x80




/* Exported functions prototypes ---------------------------------------------*/
void PE811_Init(void);
uint8_t PE811_Read(uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t PE811_Write(uint8_t reg, uint8_t cmd);


#ifdef __cplusplus
}
#endif
#endif /*__PE811_H */
