/**
  ******************************************************************************
  * File Name          : ltdc.c
  * Description        : This file provides code for the configuration
  *                      of the LTDC instances.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fmc.h"

/* Global variables ---------------------------------------------------------*/
uint32_t _LTDCREG_ = 0;

/* Private function prototypes -----------------------------------------------*/
static void LTDC_ConfigLayer1(void);
static void LTDC_ConfigLayer2(void);








////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Initializes LTDC peripheral 
  * @param  None
  * @return None
  */
void LTDC_Init(void) {
  /**LTDC GPIO Configuration    
  PC10    ------> LTDC_R2
  PA6     ------> LTDC_G2
  PD6     ------> LTDC_B2
  PB0     ------> LTDC_R3
  PG10    ------> LTDC_G3
  PG11    ------> LTDC_B3
  PA11    ------> LTDC_R4
  PB10    ------> LTDC_G4
  PG12    ------> LTDC_B4
  PA12    ------> LTDC_R5
  PB11    ------> LTDC_G5
  PA3     ------> LTDC_B5
  PB1     ------> LTDC_R6
  PC7     ------> LTDC_G6
  PB8     ------> LTDC_B6
  PG6     ------> LTDC_R7
  PD3     ------> LTDC_G7
  PB9     ------> LTDC_B7
  PA4     ------> LTDC_VSYNC
  PG7     ------> LTDC_CLK
  PF10    ------> LTDC_DE
  PD11    ------> TE
  PD12    ------> RDX
  PD13    ------> WRX
  PC2     ------> SPI_CS
  */


  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define F_MASK     LCD_DE_Pin_Mask

  /* Mode */
  MODIFY_REG(GPIOF->MODER, F_MASK, (_MODE_AF << LCD_DE_Pin_Pos * 2));
  /* Speed */
  MODIFY_REG(GPIOF->OSPEEDR, F_MASK, (_SPEED_V << LCD_DE_Pin_Pos * 2));
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MODIFY_REG(GPIOF->AFR[1], 0x00000f00, (GPIO_AF_14 << (LCD_DE_Pin_Pos - 8) * 4));



  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define A_MASK     (B5_Pin_Mask | VSYNC_Pin_Mask | G2_Pin_Mask | R4_Pin_Mask | R5_Pin_Mask)

  /* Mode */
  MODIFY_REG(GPIOA->MODER, A_MASK,
    (   (_MODE_AF << B5_Pin_Pos * 2)
      | (_MODE_AF << VSYNC_Pin_Pos * 2)
      | (_MODE_AF << G2_Pin_Pos * 2)
      | (_MODE_AF << R4_Pin_Pos * 2)
      | (_MODE_AF << R5_Pin_Pos * 2)
    )
  );
  /* Speed */
  MODIFY_REG(GPIOA->OSPEEDR, A_MASK, 
    (   (_SPEED_V << B5_Pin_Pos * 2)
      | (_SPEED_V << VSYNC_Pin_Pos * 2)
      | (_SPEED_V << G2_Pin_Pos * 2)
      | (_SPEED_V << R4_Pin_Pos * 2)
      | (_SPEED_V << R5_Pin_Pos * 2)
    )
  );
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MODIFY_REG(GPIOA->AFR[0], 0x0f0ff000, 
    (   (GPIO_AF_14 << B5_Pin_Pos * 4)
      | (GPIO_AF_14 << VSYNC_Pin_Pos * 4)
      | (GPIO_AF_14 << G2_Pin_Pos * 4)
    )
  );
  MODIFY_REG(GPIOA->AFR[1], 0x000ff000, 
    (   (GPIO_AF_14 << (R4_Pin_Pos - 8) * 4)
      | (GPIO_AF_14 << (R5_Pin_Pos - 8) * 4)
    )
  );



  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define B_MASK     (R3_Pin_Mask | R6_Pin_Mask | B6_Pin_Mask | B7_Pin_Mask | G4_Pin_Mask | G5_Pin_Mask)

  /* Mode */
  MODIFY_REG(GPIOB->MODER, B_MASK,
    (   (_MODE_AF << R3_Pin_Pos * 2)
      | (_MODE_AF << R6_Pin_Pos * 2)
      | (_MODE_AF << B6_Pin_Pos * 2)
      | (_MODE_AF << B7_Pin_Pos * 2)
      | (_MODE_AF << G4_Pin_Pos * 2)
      | (_MODE_AF << G5_Pin_Pos * 2)
    )
  );
  /* Speed */
  MODIFY_REG(GPIOB->OSPEEDR, B_MASK, 
    (   (_SPEED_V << R3_Pin_Pos * 2)
      | (_SPEED_V << R6_Pin_Pos * 2)
      | (_SPEED_V << B6_Pin_Pos * 2)
      | (_SPEED_V << B7_Pin_Pos * 2)
      | (_SPEED_V << G4_Pin_Pos * 2)
      | (_SPEED_V << G5_Pin_Pos * 2)
    )
  );
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MODIFY_REG(GPIOB->AFR[0], 0x000000ff, 
    (   (GPIO_AF_9 << R3_Pin_Pos * 4)
      | (GPIO_AF_9 << R6_Pin_Pos * 4)
    )
  );
  MODIFY_REG(GPIOB->AFR[1], 0x0000ffff, 
    (   (GPIO_AF_14 << (B6_Pin_Pos - 8) * 4)
      | (GPIO_AF_14 << (B7_Pin_Pos - 8) * 4)
      | (GPIO_AF_14 << (G4_Pin_Pos - 8) * 4)
      | (GPIO_AF_14 << (G5_Pin_Pos - 8) * 4)
    )
  );




  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define G_MASK     (R7_Pin_Mask | LCD_CLK_Pin_Mask | G3_Pin_Mask | B3_Pin_Mask | B4_Pin_Mask)

  /* Mode */
  MODIFY_REG(GPIOG->MODER, G_MASK,
    (   (_MODE_AF << R7_Pin_Pos * 2)
      | (_MODE_AF << LCD_CLK_Pin_Pos * 2)
      | (_MODE_AF << G3_Pin_Pos * 2)
      | (_MODE_AF << B3_Pin_Pos * 2)
      | (_MODE_AF << B4_Pin_Pos * 2)
    )
  );
  /* Speed */
  MODIFY_REG(GPIOG->OSPEEDR, G_MASK, 
    (   (_SPEED_V << R7_Pin_Pos * 2)
      | (_SPEED_V << LCD_CLK_Pin_Pos * 2)
      | (_SPEED_V << G3_Pin_Pos * 2)
      | (_SPEED_V << B3_Pin_Pos * 2)
      | (_SPEED_V << B4_Pin_Pos * 2)
    )
  );
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MODIFY_REG(GPIOG->AFR[0], 0xff000000, 
    (   (GPIO_AF_14 << R7_Pin_Pos * 4)
      | (GPIO_AF_14 << LCD_CLK_Pin_Pos * 4)
    )
  );
  MODIFY_REG(GPIOG->AFR[1], 0x0000f000, 
    (   (GPIO_AF_14 << (B3_Pin_Pos - 8) * 4)
    )
  );
  MODIFY_REG(GPIOG->AFR[1], 0x000f0f00, 
    (   (GPIO_AF_9 << (G3_Pin_Pos - 8) * 4)
      | (GPIO_AF_9 << (B4_Pin_Pos - 8) * 4)
    )
  );



  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define C_MASK     (HSYNC_Pin_Mask | G6_Pin_Mask | R2_Pin_Mask)

  /* Mode */
  MODIFY_REG(GPIOC->MODER, C_MASK,
    (   (_MODE_AF << HSYNC_Pin_Pos * 2)
      | (_MODE_AF << G6_Pin_Pos * 2)
      | (_MODE_AF << R2_Pin_Pos * 2)
    )
  );
  /* Speed */
  MODIFY_REG(GPIOC->OSPEEDR, C_MASK, 
    (   (_SPEED_V << HSYNC_Pin_Pos * 2)
      | (_SPEED_V << G6_Pin_Pos * 2)
      | (_SPEED_V << R2_Pin_Pos * 2)
    )
  );
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MODIFY_REG(GPIOC->AFR[0], 0xff000000, 
    (   (GPIO_AF_14 << HSYNC_Pin_Pos * 4)
      | (GPIO_AF_14 << G6_Pin_Pos * 4)
    )
  );
  MODIFY_REG(GPIOC->AFR[1], 0x00000f00, 
    (   (GPIO_AF_14 << (R2_Pin_Pos - 8) * 4)
    )
  );



  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define D_MASK     (G7_Pin_Mask | B2_Pin_Mask)

  /* Mode */
  MODIFY_REG(GPIOD->MODER, D_MASK,
    (   (_MODE_AF << G7_Pin_Pos * 2)
      | (_MODE_AF << B2_Pin_Pos * 2)
    )
  );
  /* Speed */
  MODIFY_REG(GPIOD->OSPEEDR, D_MASK, 
    (   (_SPEED_V << G7_Pin_Pos * 2)
      | (_SPEED_V << B2_Pin_Pos * 2)
    )
  );
  /* Output type */
  /* Push mode */
  /* Alternate function */
  MODIFY_REG(GPIOD->AFR[0], 0x0f00f000, 
    (   (GPIO_AF_14 << G7_Pin_Pos * 4)
      | (GPIO_AF_14 << B2_Pin_Pos * 4)
    )
  );



  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
#define D_MASK_     (LCD_RDX_Pin_Mask | LCD_WRX_Pin_Mask | LCD_TE_Pin_Mask)

  /* Mode */
  MODIFY_REG(GPIOD->MODER, D_MASK_, (
        (_MODE_OUT << LCD_RDX_Pin_Pos * 2)
      | (_MODE_OUT << LCD_WRX_Pin_Pos * 2)
      | (_MODE_IN << LCD_TE_Pin_Pos * 2)
    )
  );
  /* Speed */
  /* Output type */
  /* Push mode */
  /* Alternate function */



  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------------- */
  /* Mode */
  MODIFY_REG(LCD_CS_Port->MODER, LCD_CS_Pin_Mask, (_MODE_OUT << LCD_CS_Pin_Pos * 2));
  /* Speed */
  /* Output type */
  /* Push mode */
  /* Alternate function */



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  ILI9341_Init();


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* Configure the HS, VS, DE and PC polarity */
  MODIFY_REG(LTDC->GCR, (LTDC_GCR_HSPOL_Msk | LTDC_GCR_VSPOL_Msk | LTDC_GCR_DEPOL_Msk | LTDC_GCR_PCPOL_Msk), (
      HSPOL_AL
    | VSPOL_AL
    | DEPOL_AL
    | PCPOL_IIPC
  ));
  /* Set Synchronization size */
  LTDC->SSCR = (__IO uint32_t)(
      (1 << LTDC_SSCR_VSH_Pos)
    | (9 << LTDC_SSCR_HSW_Pos)
  );
  /* Set Accumulated Back porch */
  LTDC->BPCR = (__IO uint32_t)(
        (3 << LTDC_BPCR_AVBP_Pos)
      | (29 << LTDC_BPCR_AHBP_Pos)
    );

  /* Set Accumulated Active Width and Height */
  #ifdef _LANDSCAPE_
    LTDC->AWCR = (__IO uint32_t)(
        ((DISPLAY_HEIGHT + 29) << LTDC_AWCR_AAW_Pos)
      | ((DISPLAY_WIDTH + 3) << LTDC_AWCR_AAH_Pos)
    );
  #else
    LTDC->AWCR = (__IO uint32_t)(
        ((DISPLAY_WIDTH + 29) << LTDC_AWCR_AAW_Pos)
      | ((DISPLAY_HEIGHT + 3) << LTDC_AWCR_AAH_Pos)
    );
  #endif

  /* Set Total Width and Height */
  #ifdef _LANDSCAPE_
    LTDC->TWCR = (__IO uint32_t)(
        ((DISPLAY_HEIGHT + 29 + 10) << LTDC_TWCR_TOTALW_Pos)
      | ((DISPLAY_WIDTH + 3 + 4) << LTDC_TWCR_TOTALH_Pos)
    );
  #else
    LTDC->TWCR = (__IO uint32_t)(
        ((DISPLAY_WIDTH + 29 + 10) << LTDC_TWCR_TOTALW_Pos)
      | ((DISPLAY_HEIGHT + 3 + 4) << LTDC_TWCR_TOTALH_Pos)
    );
  #endif

  /* Set the background color value */
  LTDC->BCCR = (__IO uint32_t)(ARGB8888_Lightblue);
  /* Enable the Transfer Error and FIFO underrun interrupts */
  LTDC->IER |= (LTDC_IER_TERRIE | LTDC_IER_FUIE);


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  LTDC_ConfigLayer1();
  LTDC_ConfigLayer2();

  /* Enable Dither by setting DTEN bit */
  PREG_SET(LTDC->GCR, LTDC_GCR_DEN_Pos);

  /* Enable LTDC by setting LTDCEN bit */
  PREG_SET(LTDC->GCR, LTDC_GCR_LTDCEN_Pos);


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  FillLayer(L1, _WHITE_0);
  FillLayer(L2, _WHITE_0);
}







/**
  * @brief  Setup an LTDC Layer 1.
  * @param  None
  * @return None
  */
static void LTDC_ConfigLayer1(void) {
  LTDC_Layer_TypeDef* layer = (LTDC_Layer_TypeDef*)LTDC_Layer1_BASE;

  /* Configure the horizontal start and stop position */
  #ifdef _LANDSCAPE_
    layer->WHPCR = (
        ((L1_PADDING_BOTTOM + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) << LTDC_LxWHPCR_WHSTPOS_Pos)
      | (((L1_HEIGHT + L1_PADDING_BOTTOM) + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U)) << LTDC_LxWHPCR_WHSPPOS_Pos)
    );
  #else
    layer->WHPCR = (
        ((L1_PADDING_LEFT + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) << LTDC_LxWHPCR_WHSTPOS_Pos)
      | (((L1_WIDTH + L1_PADDING_LEFT) + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U)) << LTDC_LxWHPCR_WHSPPOS_Pos)
    );
  #endif

  /* Configure the vertical start and stop position */
  #ifdef _LANDSCAPE_
    layer->WVPCR = (
        ((L1_PADDING_LEFT + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1U) << LTDC_LxWVPCR_WVSTPOS_Pos)
      | (((L1_WIDTH + L1_PADDING_LEFT) + (LTDC->BPCR & LTDC_BPCR_AVBP)) << LTDC_LxWVPCR_WVSPPOS_Pos)
    );
  #else
    layer->WVPCR = (
        ((L1_PADDING_TOP + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1U) << LTDC_LxWVPCR_WVSTPOS_Pos)
      | (((L1_HEIGHT + L1_PADDING_TOP) + (LTDC->BPCR & LTDC_BPCR_AVBP)) << LTDC_LxWVPCR_WVSPPOS_Pos)
    );
  #endif

  /* Specifies the pixel format */
  layer->PFCR = (PIXEL_FORMAT_ARGB8888);
  /* Configure the default color values */
  layer->DCCR = (ARGB8888_Orange | 0xff000000);
  /* Specifies the constant alpha value */
  layer->CACR = 0xff;
  /* Specifies the blending factors */
  layer->BFCR = (
      BLENDING_FACTOR1_PAxCA
    | BLENDING_FACTOR2_PAxCA
  );
  /* Configure the color frame buffer start address */
  layer->CFBAR = L1_ADDR;

  /* Configure the color frame buffer pitch in byte */
  #ifdef _LANDSCAPE_
    layer->CFBLR = (
        (((L1_HEIGHT * 4) + 3U) << LTDC_LxCFBLR_CFBLL_Pos)
      | ((L1_HEIGHT * 4) << LTDC_LxCFBLR_CFBP_Pos)
    );
  #else
    layer->CFBLR = (
        (((L1_WIDTH * 4) + 3U) << LTDC_LxCFBLR_CFBLL_Pos)
      | ((L1_WIDTH * 4) << LTDC_LxCFBLR_CFBP_Pos)
    );
  #endif

  /* Configure the frame buffer line number */
  #ifdef _LANDSCAPE_
    layer->CFBLNR = L1_WIDTH;
  #else
    layer->CFBLNR = L1_HEIGHT;
  #endif

  /* Enable LTDC_Layer by setting LEN bit */
  PREG_SET(layer->CR, LTDC_LxCR_LEN_Pos);
  PREG_SET(LTDC->SRCR, LTDC_SRCR_IMR_Pos);
}




/**
  * @brief  Setup an LTDC Layer 2.
  * @param  None
  * @return None
  */
static void LTDC_ConfigLayer2(void) {
  LTDC_Layer_TypeDef* layer = (LTDC_Layer_TypeDef*)LTDC_Layer2_BASE;

  #ifdef _LANDSCAPE_
    layer->WHPCR = (
        ((L2_PADDING_BOTTOM + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) << LTDC_LxWHPCR_WHSTPOS_Pos)
      | (((L2_HEIGHT + L2_PADDING_BOTTOM) + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U)) << LTDC_LxWHPCR_WHSPPOS_Pos)
    );
  #else
    layer->WHPCR = (
        ((L2_PADDING_LEFT + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) << LTDC_LxWHPCR_WHSTPOS_Pos)
      | (((L2_WIDTH + L2_PADDING_LEFT) + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U)) << LTDC_LxWHPCR_WHSPPOS_Pos)
    );
  #endif

  /* Configure the vertical start and stop position */
  #ifdef _LANDSCAPE_
    layer->WVPCR = (
        ((L2_PADDING_LEFT + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1U) << LTDC_LxWVPCR_WVSTPOS_Pos)
      | (((L2_WIDTH + L2_PADDING_LEFT) + (LTDC->BPCR & LTDC_BPCR_AVBP)) << LTDC_LxWVPCR_WVSPPOS_Pos)
    );
  #else
    layer->WVPCR = (
        ((L2_PADDING_TOP + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1U) << LTDC_LxWVPCR_WVSTPOS_Pos)
      | (((L2_HEIGHT + L2_PADDING_TOP) + (LTDC->BPCR & LTDC_BPCR_AVBP)) << LTDC_LxWVPCR_WVSPPOS_Pos)
    );
  #endif

  /* Specifies the pixel format */
  layer->PFCR = (__IO uint8_t)(PIXEL_FORMAT_ARGB8888);
  /* Configure the default color values */
  layer->DCCR = (__IO uint32_t)(ARGB8888_Apple | 0x00000000);
  /* Specifies the constant alpha value */
  layer->CACR = (__IO uint8_t)(0xff);
  /* Specifies the blending factors */
  layer->BFCR = (__IO uint16_t)(
      BLENDING_FACTOR1_PAxCA
    | BLENDING_FACTOR2_PAxCA
  );
  /* Configure the color frame buffer start address */
  layer->CFBAR = (__IO uint32_t)(L2_ADDR);

  /* Configure the color frame buffer pitch in byte */
  #ifdef _LANDSCAPE_
    layer->CFBLR = (
        (((L2_HEIGHT * 4) + 3U) << LTDC_LxCFBLR_CFBLL_Pos)
      | ((L2_HEIGHT * 4) << LTDC_LxCFBLR_CFBP_Pos)
    );
  #else
    layer->CFBLR = (
        (((L2_WIDTH * 4) + 3U) << LTDC_LxCFBLR_CFBLL_Pos)
      | ((L2_WIDTH * 4) << LTDC_LxCFBLR_CFBP_Pos)
    );
  #endif

  /* Configure the frame buffer line number */
  #ifdef _LANDSCAPE_
    layer->CFBLNR = L2_WIDTH;
  #else
    layer->CFBLNR = L2_HEIGHT;
  #endif


  /* Enable LTDC_Layer by setting LEN bit */
  PREG_SET(layer->CR, LTDC_LxCR_LEN_Pos);
  PREG_SET(LTDC->SRCR, LTDC_SRCR_IMR_Pos);
}