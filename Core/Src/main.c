/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ---------------------------------------------------------*/
uint32_t sysQuantum           = 0;
uint32_t millis               = 0;
uint32_t seconds              = 0;
uint32_t minutes              = 0;
uint32_t _GLOBALREG_          = 0;
uint32_t delay_tmp            = 0;
uint32_t SystemCoreClock      = 16000000;
RCC_ClocksTypeDef RccClocks;

/* Private variables ---------------------------------------------------------*/
static uint32_t millis_tmp    = 100;
static uint32_t seconds_tmp   = 1000;
static uint32_t minutes_tmp   = 60;

/* Private function prototypes -----------------------------------------------*/
static void CronSysQuantum_Handler(void);
static void CronMillis_Handler(void);
static void CronSeconds_Handler(void);
static void CronMinutes_Handler(void);
static void Flags_Handler(void);









////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  The application entry point.
  * @return int
  */
int main(void) {

  // Delay(500);

  while (1) {
    Delay_Handler(0);
    Cron_Handler();
    Flags_Handler();
  }
}








/********************************************************************************/
/*                                     CRON                                     */
/********************************************************************************/
void Cron_Handler(void) {
  $CronStart:
  if (SysTick->CTRL & (1 << SysTick_CTRL_COUNTFLAG_Pos)) { 
    sysQuantum++;
    CronSysQuantum_Handler();
  }

  if (sysQuantum >= millis_tmp) {
    millis++;
    millis_tmp = sysQuantum + 100;
    CronMillis_Handler();
  }
  
  if (millis >= seconds_tmp) {
    seconds++;
    seconds_tmp += 1000;
    CronSeconds_Handler();
  }
  
  if (seconds >= minutes_tmp) {
    minutes++;
    minutes_tmp += 60;
    CronMinutes_Handler();
  }

  while (sysQuantum < delay_tmp) {
    goto $CronStart;
  }
  // !!!!!!!!! The bug!!!!!!!!
  delay_tmp = 0;
  FLAG_CLR(_GLOBALREG_, _DELAYF_);
}






/********************************************************************************/
/*                             CRON EVENTS HANDLERS                             */
/********************************************************************************/
// ---- System Quantum ---- //
static void CronSysQuantum_Handler(void) {
  //
}

// ---- Milliseconds ---- //
static void CronMillis_Handler(void) {
  //
}

// ---- Seconds ---- //
static void CronSeconds_Handler(void) {
  /* Refresh independent watchdog */
  WRITE_REG(IWDG->KR, IWDG_KEY_RELOAD);
  /* Let the Date and Tine to be displayed */
  FLAG_SET(_GLOBALREG_, _DDTF_);
}

// ---- Minutes ---- //
static void CronMinutes_Handler(void) {
  //
}






/********************************************************************************/
/*                                     FLAGS                                    */
/********************************************************************************/
void Flags_Handler(void){
  /* TIM7 action */
  if (FLAG_CHECK(_TIMREG_, _BT7F_)) {
    Timer_Handler(TIM7);
    FLAG_CLR(_TIMREG_, _BT7F_);
  }

  /* USART1 action */
  if (FLAG_CHECK(_USARTREG_, _USART1_RXAF_)) {
    USART1_RX_Handler();
    FLAG_CLR(_USARTREG_, _USART1_RXAF_);
  }

  /* USART1 Local Buffer not empty action */
  if (FLAG_CHECK(_USARTREG_, _USART1_LBNEF_)) {
    // uint8_t bbb[4];
    // USART1_RxBufferRead(bbb, 4);
    FLAG_CLR(_USARTREG_, _USART1_LBNEF_);
  }

  /* RTC Alarm A action */
  if (FLAG_CHECK(_RTCREG_, _ALAF_)) {
    RTC_Alarm_Handler('A');
    FLAG_CLR(_RTCREG_, _ALAF_);
  }

  /* RTC Alarm B action */
  if (FLAG_CHECK(_RTCREG_, _ALBF_)) {
    RTC_Alarm_Handler('B');
    FLAG_CLR(_RTCREG_, _ALBF_);
  }

  /* TouchScreen interrupt event  */
  if (FLAG_CHECK(_TSREG_, _TEVF_)) {
    TS_Handler();
    FLAG_CLR(_TSREG_, _TEVF_);
  }

  /* Gyroscope INT1 interrupt event  */
  if (FLAG_CHECK(_MEMSREG_, _INT1F_)) {
    MEMS_Handler();
    FLAG_CLR(_MEMSREG_, _INT1F_);
  }

  /* Gyroscope INT2 interrupt event  */
  if (FLAG_CHECK(_MEMSREG_, _INT2F_)) {
    MEMS_Handler();
    FLAG_CLR(_MEMSREG_, _INT2F_);
  }

  /* Display Date and Time  */
  if (FLAG_CHECK(_GLOBALREG_, _DDTF_)) {
    DisplayDateAndTime_Handler();
    printf("test\n");
    FLAG_CLR(_GLOBALREG_, _DDTF_);
  }

}











/**
  * @brief  Initializes clocks and peripherals.
  * @param  None
  * @return None
  */
void SystemInit(void) {

  #if (INSTRUCTION_CACHE_ENABLE != 0U)
    PREG_SET(FLASH->ACR, FLASH_ACR_ICEN_Pos);
  #endif /* INSTRUCTION_CACHE_ENABLE */

  #if (DATA_CACHE_ENABLE != 0U)
    PREG_SET(FLASH->ACR, FLASH_ACR_DCEN_Pos);
  #endif /* DATA_CACHE_ENABLE */

  #if (PREFETCH_ENABLE != 0U)
    PREG_SET(FLASH->ACR, FLASH_ACR_PRFTEN_Pos);
  #endif /* PREFETCH_ENABLE */

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick */
  SET_BIT(SysTick->CTRL, (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk));
  SysTick->LOAD = 1800U - 1U;
  SysTick->VAL = 0;
  SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);

  /* SysTick interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SysTick_IRQn);

  /* SysCfg */
  PREG_SET(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN_Pos);
  while (!(PREG_CHECK(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN_Pos)));

  /* PWR */
  PREG_SET(RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);
  while (!(PREG_CHECK(RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos)))

  /* FLASH_IRQn interrupt configuration */
  NVIC_SetPriority(FLASH_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(FLASH_IRQn);

  /* RCC_IRQn interrupt configuration */
  NVIC_SetPriority(RCC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(RCC_IRQn);

  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

  if (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) {
    Error_Handler();
  }
  
  /* Enable overdrive */
  MODIFY_REG(PWR->CR, PWR_CR_VOS, (PWR_CR_VOS_0 | PWR_CR_VOS_1));
  PREG_SET(PWR->CR, PWR_CR_ODEN_Pos);  
  
  /* HSE enable and wait till it's ready */
  PREG_SET(RCC->CR, RCC_CR_HSEON_Pos);
  while(!(PREG_CHECK(RCC->CR, RCC_CR_HSERDY_Pos)));
  
  /* LSI enable and wait till it's ready */
  PREG_SET(RCC->CSR, RCC_CSR_LSION_Pos);
  while(!(PREG_CHECK(RCC->CSR, RCC_CSR_LSIRDY_Pos)));

  /* Backup registers enable access */
  PREG_SET(PWR->CR, PWR_CR_DBP_Pos);

  /* force backup domain reset */
  PREG_SET(RCC->BDCR, RCC_BDCR_BDRST_Pos);
  PREG_CLR(RCC->BDCR, RCC_BDCR_BDRST_Pos);
  
  /* LSE enable and wait till it's ready */
  PREG_SET(RCC->BDCR, RCC_BDCR_LSEON_Pos);
  // PREG_CLR(RCC->BDCR, RCC_BDCR_LSEON_Pos);
  while(!(PREG_CHECK(RCC->BDCR, RCC_BDCR_LSERDY_Pos)));

  /* RTC source is LSE */
  MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL, RCC_BDCR_RTCSEL_0);
  /* RTC source is LSI */
  // MODIFY_REG(RCC->BDCR, RCC_BDCR_RTCSEL_Msk, RCC_BDCR_RTCSEL_1);

  // /* Enable RTC */
  PREG_SET(RCC->BDCR, RCC_BDCR_RTCEN_Pos);

  /* Configure PLL domain end prescaller */
  MODIFY_REG(RCC->PLLCFGR, (RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN), (RCC_PLLCFGR_PLLSRC_HSE | RCC_PLLCFGR_PLLM_2 | 180 << RCC_PLLCFGR_PLLN_Pos));
  /* PLL divided into two */
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, 0U);
  
  /* PLL enable and wait till it's ready */
  PREG_SET(RCC->CR, RCC_CR_PLLON_Pos);
  while (!(PREG_CHECK(RCC->CR, RCC_CR_PLLRDY_Pos)));


  /* AHB isn't divided */
  // MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, LL_RCC_SYSCLK_DIV_1);
  /* APB1 divided by 4 */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4);
  /* APB2 divided by 2 */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2);

  /* Enable PLL as sysclock and wait till it's ready */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  while(!(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL));

  SystemCoreClock = 180000000U;
  RccClocks.HCLK_Freq = SystemCoreClock;
  RccClocks.PCLK1_Freq = 45000000U;
  RccClocks.PCLK1_Freq_Tim = 90000000U;
  RccClocks.PCLK2_Freq = 90000000U;
  RccClocks.PCLK2_Freq_Tim = 180000000U;


  /* Set timer prescaler */
  MODIFY_REG(RCC->DCKCFGR, RCC_DCKCFGR_TIMPRE, 0);

  /* Enable PLL SAI Domain */
  MODIFY_REG(RCC->PLLCFGR, (RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM), (RCC_PLLCFGR_PLLSRC_HSE | RCC_PLLCFGR_PLLM_2));
  MODIFY_REG(RCC->PLLSAICFGR, RCC_PLLSAICFGR_PLLSAIN | RCC_PLLSAICFGR_PLLSAIR, (60U << RCC_PLLSAICFGR_PLLSAIN_Pos | RCC_PLLSAICFGR_PLLSAIR_2 | RCC_PLLSAICFGR_PLLSAIR_0)); /*!< PLLSAI division factor for PLLSAIR output by 5 */
  /* PLLSAI division factor for PLLSAIDIVR output by 4 */
  MODIFY_REG(RCC->DCKCFGR, RCC_DCKCFGR_PLLSAIDIVR, RCC_DCKCFGR_PLLSAIDIVR_0);

  /* PLL SAI wait till it's ready */
  PREG_SET(RCC->CR, RCC_CR_PLLSAION_Pos);
  while (!(PREG_CHECK(RCC->CR, RCC_CR_PLLSAIRDY_Pos)));


  /*****************************************************************************************/
  /*****************************************************************************************/
  /*****************************************************************************************/
  /* Freeze some peripherals for the debug purpose */
  #ifdef DEBUG
  SET_BIT(DBGMCU->APB1FZ, (
      DBGMCU_APB1_FZ_DBG_TIM7_STOP
    | DBGMCU_APB1_FZ_DBG_IWDG_STOP
    | DBGMCU_APB1_FZ_DBG_WWDG_STOP
  ));
  #endif

  /*****************************************************************************************/
  /* IWDG */
  WRITE_REG(IWDG->KR, IWDG_KEY_ENABLE);
  WRITE_REG(IWDG->KR, IWDG_KEY_WR_ACCESS_ENABLE);
  WRITE_REG(IWDG->PR, IWDG_PR_PR & (IWDG_PR_PR_2 | IWDG_PR_PR_0)); /*!< Divider by 128 */
  WRITE_REG(IWDG->RLR, IWDG_RLR_RL & 6624U);
  while (!(PREG_CHECK(IWDG->SR, IWDG_SR_PVU_Pos)));
  WRITE_REG(IWDG->KR, IWDG_KEY_RELOAD);

  /*****************************************************************************************/
  /* Peripheral clock */
  /* APB1 */
  SET_BIT(RCC->APB1ENR, (
      RCC_APB1ENR_TIM7EN
    // | RCC_APB1ENR_I2C3EN
  ));

  /* AHB1 */
  SET_BIT(RCC->AHB1ENR, ( 
      RCC_AHB1ENR_GPIOAEN
    | RCC_AHB1ENR_GPIOBEN
    | RCC_AHB1ENR_GPIOCEN
    | RCC_AHB1ENR_GPIODEN
    | RCC_AHB1ENR_GPIOEEN
    | RCC_AHB1ENR_GPIOFEN
    | RCC_AHB1ENR_GPIOGEN
    | RCC_AHB1ENR_GPIOHEN
    | RCC_AHB1ENR_CRCEN
    | RCC_AHB1ENR_DMA2DEN
  ));

  /* APB2 */
  SET_BIT(RCC->APB2ENR, (
      RCC_APB2ENR_SPI5EN
    | RCC_APB2ENR_USART1EN
    | RCC_APB2ENR_LTDCEN
  ));

  /* AHB3 */
  SET_BIT(RCC->AHB3ENR, (
      RCC_AHB3ENR_FMCEN
  ));
  /* Wait for FMC starts */
  while (!(PREG_CHECK(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN_Pos)));



  /************************************************************************************************/
  /************************************************************************************************/
  /************************************************************************************************/
  Delay(500);
  
  /* LED */
  LED_Init();

  /* EXTI */
  EXTI_Init();

  /* USART1 */ 
  USART1_Init();

  /* SPI5 */
  SPI5_Init();

  /* I2C3 */
  I2C3_Init();

  /* Basic Timer7 */
  BasicTimer7_Init();

  /* RTC */
  RTC_Init();

  /* FMC */
  FMC_Init(_SDRAM);

  /* DMA2D */
  DMA2D_Init();
  
  /* LTDC */
  LTDC_Init();

  /* Touchscreen STMPE811 */
  PE811_Init();

  /* MEMS Hyroscope */
  L3GD20_Init();

}


