/**
  ******************************************************************************
  * File Name          : l3dg20.h
  * Description        : This file provides code for the configuration
  *                      of the L3GD20 Hyroscope.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L3GD20_H
#define __L3GD20_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ----------------------------------------------------------*/



/* Private defines -----------------------------------------------------------*/
// PC1
#define MEMS_CS_Pin                   GPIO_PIN_1
#define MEMS_CS_Pin_Pos               GPIO_PIN_1_Pos
#define MEMS_CS_Pin_Mask              GPIO_PIN_1_Mask
#define MEMS_CS_Port                  GPIOC
// PA1
#define MEMS_INT1_Pin                 GPIO_PIN_1
#define MEMS_INT1_Pin_Pos             GPIO_PIN_1_Pos
#define MEMS_INT1_Pin_Mask            GPIO_PIN_1_Mask
// PA2
#define MEMS_INT2_Pin                 GPIO_PIN_2
#define MEMS_INT2_Pin_Pos             GPIO_PIN_2_Pos
#define MEMS_INT2_Pin_Mask            GPIO_PIN_2_Mask
#define MEMS_INT_Port                 GPIOA

#define MEMS_WATERMARK                (0x1f << FIFO_CTRL_REG_WTM_Pos)

/* L3GD20 Registers definition */
#define L3GD20_WHO_AM_I               0x0f
#define L3GD20_CTRL_REG_1             0x20
#define L3GD20_CTRL_REG_2             0x21
#define L3GD20_CTRL_REG_3             0x22
#define L3GD20_CTRL_REG_4             0x23
#define L3GD20_CTRL_REG_5             0x24
#define L3GD20_REFERENCE              0x25
#define L3GD20_OUT_TEMP               0x26 // Only read
#define L3GD20_STATUS_REG             0x27 // Only read
#define L3GD20_OUT_X_L                0x28 // Only read
#define L3GD20_OUT_X_H                0x29 // Only read
#define L3GD20_OUT_Y_L                0x2a // Only read
#define L3GD20_OUT_Y_H                0x2b // Only read
#define L3GD20_OUT_Z_L                0x2c // Only read
#define L3GD20_OUT_Z_H                0x2d // Only read
#define L3GD20_FIFO_CTRL_REG          0x2e
#define L3GD20_FIFO_SRC_REG           0x2f // Only read
#define L3GD20_INT1_CFG               0x30
#define L3GD20_INT1_SRC               0x31 // Only read
#define L3GD20_INT1_TSH_XH            0x32
#define L3GD20_INT1_TSH_XL            0x33
#define L3GD20_INT1_TSH_YH            0x34
#define L3GD20_INT1_TSH_YL            0x35
#define L3GD20_INT1_TSH_ZH            0x36
#define L3GD20_INT1_TSH_ZL            0x37
#define L3GD20_INT1_DURATION          0x38

/* L3GD20_CTRL_REG_1 Bits Definiion */
#define CTRL_REG_1_XEN                0x01
#define CTRL_REG_1_YEN                0x02
#define CTRL_REG_1_ZEN                0x04
#define CTRL_REG_1_PD                 0x08
#define CTRL_REG_1_DR_BW_Pos          4
#define CTRL_REG_1_DR_BW_95_12_5      (0b0000 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_95_25        (0b0001 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_95_25_       (0b0010 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_95_25__      (0b0011 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_190_12_5     (0b0100 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_190_25       (0b0101 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_190_50       (0b0110 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_190_70       (0b0111 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_380_20       (0b1000 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_380_25       (0b1001 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_380_50       (0b1010 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_380_100      (0b1011 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_760_30       (0b1100 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_760_35       (0b1101 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_760_50       (0b1110 << CTRL_REG_1_DR_BW_Pos)
#define CTRL_REG_1_DR_BW_760_100      (0b1111 << CTRL_REG_1_DR_BW_Pos)

/* L3GD20_CTRL_REG_2 Bits Definiion */
// Look data int datasheet
// Table 26. High-pass filter cut off frequency configuration [Hz]
#define CTRL_REG_2_HPM_Pos            4
#define CTRL_REG_2_HPM_NORM           (0b00 << CTRL_REG_2_HPM_Pos) // Normal mode (reset reading HP_RESET_FILTER)
#define CTRL_REG_2_HPM_REF            (0b01 << CTRL_REG_2_HPM_Pos) // Reference signal for filtering
#define CTRL_REG_2_HPM_NORM_          (0b10 << CTRL_REG_2_HPM_Pos) // Normal mode
#define CTRL_REG_2_HPM_AUTO           (0b11 << CTRL_REG_2_HPM_Pos) // Autoreset on interrupt event

/* L3GD20_CTRL_REG_3 Bits Definiion */
#define CTRL_REG_3_I2_EMPTY           0x01 // 0: disable, 1: enable
#define CTRL_REG_3_I2_ORUN            0x02 // 0: disable, 1: enable
#define CTRL_REG_3_I2_WTM             0x04 // 0: disable, 1: enable
#define CTRL_REG_3_I2_DRDY            0x08 // 0: disable, 1: enable
#define CTRL_REG_3_I2_PP_OD           0x10 // 0: push-pull; 1: open-drain
#define CTRL_REG_3_I2_H_LA            0x20 // 0: high, 1: low
#define CTRL_REG_3_I1_BOOT            0x40 // 0: disable, 1: enable
#define CTRL_REG_3_I1_INT             0x80 // 0: disable, 1: enable

/* L3GD20_CTRL_REG_4 Bits Definiion */
#define CTRL_REG_4_SIM                0x01 // 0: 4-wire; 1: 3-wire
#define CTRL_REG_4_FS_Pos             4
#define CTRL_REG_4_FS_250             (0b00 << CTRL_REG_4_FS_Pos) // 250dps
#define CTRL_REG_4_FS_500             (0b01 << CTRL_REG_4_FS_Pos) // 500dps
#define CTRL_REG_4_FS_2000            (0b10 << CTRL_REG_4_FS_Pos) // 2000dps
#define CTRL_REG_4_FS_2000_           (0b11 << CTRL_REG_4_FS_Pos) // 2000dps
#define CTRL_REG_4_BLE                0x40 // 0: Data LSb @ lower address, 1: Data MSb @ lower address
#define CTRL_REG_4_BDU                0x80 // 0: continuos update, 1: output registers not updated until MSb and LSb reading

/* L3GD20_CTRL_REG_5 Bits Definiion */
#define CTRL_REG_5_OUT_SEL_Pos        0
#define CTRL_REG_5_INT_SEL_Pos        2
#define CTRL_REG_5_HP_EN              0x10 // High-pass filter enable
#define CTRL_REG_5_FIFO_EN            0x40 // FIFO enable
#define CTRL_REG_5_BOOT               0x80 // Reboot memory content. 0: normal mode, 1: reboot memory content

/* L3GD20_STATUS_REG Bits Definiion */
#define STATUS_REG_XDA                0x01 // 0: new data for the X-axis is not yet available, 1: new data for the X-axis is available
#define STATUS_REG_YDA                0x02
#define STATUS_REG_ZDA                0x04
#define STATUS_REG_XYZDA              0x08
#define STATUS_REG_XOR                0x10 // 0: no overrun has occurred, 1: new data for the X-axis has overwritten the previous data
#define STATUS_REG_YOR                0x20
#define STATUS_REG_ZOR                0x40
#define STATUS_REG_XYZOR              0x80

/* FIFO_CTRL_REG Bits Definiion */
#define FIFO_CTRL_REG_WTM_Pos         0
#define FIFO_CTRL_REG_FM_Pos          5
#define FIFO_CTRL_REG_FM_BYPASS       (0b000 << FIFO_CTRL_REG_FM_Pos)
#define FIFO_CTRL_REG_FM_FIFO         (0b001 << FIFO_CTRL_REG_FM_Pos)
#define FIFO_CTRL_REG_FM_STREAM       (0b010 << FIFO_CTRL_REG_FM_Pos)
#define FIFO_CTRL_REG_FM_S2F          (0b011 << FIFO_CTRL_REG_FM_Pos) // Stream-to-FIFO mode
#define FIFO_CTRL_REG_FM_B2S          (0b100 << FIFO_CTRL_REG_FM_Pos) // Bypass-to-Stream mode

/* FIFO_CTRL_SRC Bits Definiion */
#define FIFO_SRC_REG_FSS_Pos          0
#define FIFO_SRC_REG_EMPTY            0x20
#define FIFO_SRC_REG_ORUN             0x40
#define FIFO_SRC_REG_WTM              0x80

/* L3GD20_INT1_CFG Bits Definiion */
#define INT1_CFG_XLIE                 0x01 // Enable interrupt generation on X low event. 0: disable interrupt request, 1: enable interrupt request on measured value lowerthan preset threshold
#define INT1_CFG_XHIE                 0x02
#define INT1_CFG_YLIE                 0x04
#define INT1_CFG_YHIE                 0x08
#define INT1_CFG_ZLIE                 0x10
#define INT1_CFG_ZHIE                 0x20
#define INT1_CFG_LIR                  0x40 // Latch interrupt request
#define INT1_CFG_AND_OR               0x80 //AND/OR combination of interrupt events. 0: OR, 1: AND

/* L3GD20_INT1_SRC Bits Definiion */
#define INT1_SRC_XL                   0x01 // X low. 0: no interrupt, 1: X Low event has occurred
#define INT1_SRC_XH                   0x02
#define INT1_SRC_YL                   0x04
#define INT1_SRC_YH                   0x08
#define INT1_SRC_ZL                   0x10
#define INT1_SRC_ZH                   0x20
#define INT1_SRC_IA                   0x40



/* Private macro -------------------------------------------------------------*/
#define MEMS_CS_H                     PIN_H(GPIOC, MEMS_CS_Pin_Pos)
#define MEMS_CS_L                     PIN_L(GPIOC, MEMS_CS_Pin_Pos)


/* Registers */


/* Exported functions prototypes ---------------------------------------------*/
void L3GD20_Init(void);
void L3GD20_Read(uint8_t *buf, uint8_t cnt);
void L3GD20_Write(uint8_t *buf, uint8_t cnt);


#ifdef __cplusplus
}
#endif
#endif /*__L3GD20_H */
