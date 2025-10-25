/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    b_l475e_iot01a1.c
  * @author  MCD Application Team (Modified)
  * @brief   Source file for the BSP Common driver
  ******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "b_l475e_iot01a1.h"
#include "stm32l4xx_hal_exti.h"
#include <stdio.h> // Include for printf

/** @addtogroup BSP
 * @{
 */

/** @addtogroup B_L475E_IOT01A1
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL B_L475E_IOT01A1 LOW LEVEL
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Variables B_L475E_IOT01A1 LOW LEVEL Private Variables
 * @{
 */
/* Led */
typedef void (* BSP_LED_GPIO_Init) (void);
static GPIO_TypeDef* LED_PORT[LEDn] = {LED2_GPIO_PORT};
static const uint16_t LED_PIN[LEDn]  = {LED2_PIN};
static void LED_USER_GPIO_Init(void); // Prototype for local init function

/* Button */
typedef void (* BSP_BUTTON_GPIO_Init) (void);
static GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
static const uint16_t  BUTTON_PIN[BUTTONn]  = {USER_BUTTON_PIN};
static const IRQn_Type BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};
EXTI_HandleTypeDef hpb_exti[BUTTONn] = {{.Line = USER_BUTTON_EXTI_LINE}};
static void BUTTON_USER_GPIO_Init(void); // Prototype for local init function

/* Com */
#if (USE_BSP_COM_FEATURE > 0)
USART_TypeDef* COM_USART[COMn] = {COM1_UART};
UART_HandleTypeDef hcom_uart[COMn];
#if (USE_COM_LOG > 0)
static COM_TypeDef COM_ActiveLogPort = COM1;
#endif
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
static uint32_t IsUsart1MspCbValid = 0;
#endif
__weak HAL_StatusTypeDef MX_USART1_UART_Init(UART_HandleTypeDef* huart);
#endif /* (USE_BSP_COM_FEATURE > 0) */

/* I2C */
extern I2C_HandleTypeDef hi2c2;
#define hI2cHandler hi2c2   // simple alias so BSP uses same instance
/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_FunctionPrototypes B_L475E_IOT01A1 LOW LEVEL Private Function Prototypes
 * @{
 */
// --- FIX: Add the missing typedef ---
typedef void (* BSP_EXTI_LineCallback) (void);
// ------------------------------------

static void BUTTON_USER_EXTI_Callback(void);

#if (USE_BSP_COM_FEATURE > 0)
static void USART1_MspInit(UART_HandleTypeDef *huart);
static void USART1_MspDeInit(UART_HandleTypeDef *huart);
#endif

/* I2C Functions (adapted from reference) */
static void     I2Cx_MspInit(I2C_HandleTypeDef *i2c_handler);
static void     I2Cx_MspDeInit(I2C_HandleTypeDef *i2c_handler);
static void     I2Cx_Init(I2C_HandleTypeDef *i2c_handler);
static void     I2Cx_DeInit(I2C_HandleTypeDef *i2c_handler);
static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials);
static void              I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr);

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Functions B_L475E_IOT01A1 LOW LEVEL Private Functions
 * @{
 */

int32_t BSP_GetVersion(void)
{
  return (int32_t)__B_L475E_IOT01A1_BSP_VERSION;
}

//------------------------------------------------------------------------------
// LED Functions
//------------------------------------------------------------------------------
int32_t BSP_LED_Init(Led_TypeDef Led)
{
  LED2_GPIO_CLK_ENABLE();
  return BSP_ERROR_NONE;
}
int32_t BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);
  return BSP_ERROR_NONE;
}
int32_t BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT [Led], LED_PIN [Led], GPIO_PIN_SET);
  return BSP_ERROR_NONE;
}
int32_t BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT [Led], LED_PIN [Led], GPIO_PIN_RESET);
  return BSP_ERROR_NONE;
}
int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
  return BSP_ERROR_NONE;
}
int32_t BSP_LED_GetState(Led_TypeDef Led)
{
  return (int32_t)HAL_GPIO_ReadPin (LED_PORT [Led], LED_PIN [Led]);
}
static void LED_USER_GPIO_Init(void) {
  LED2_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
}


//------------------------------------------------------------------------------
// BUTTON Functions
//------------------------------------------------------------------------------
int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  int32_t ret = BSP_ERROR_NONE;
  static const BSP_EXTI_LineCallback ButtonCallback[BUTTONn] = {BUTTON_USER_EXTI_Callback}; // This line is now valid
  static const uint32_t BSP_BUTTON_PRIO [BUTTONn] = {BSP_BUTTON_USER_IT_PRIORITY};
  static const uint32_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE};

  BUTTON_USER_GPIO_Init(); // Call the specific init function

  if (ButtonMode == BUTTON_MODE_EXTI)
  {
    if(HAL_EXTI_GetHandle(&hpb_exti[Button], BUTTON_EXTI_LINE[Button]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_EXTI_RegisterCallback(&hpb_exti[Button], HAL_EXTI_COMMON_CB_ID, ButtonCallback[Button]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      HAL_NVIC_SetPriority((BUTTON_IRQn[Button]), BSP_BUTTON_PRIO[Button], 0x00);
      HAL_NVIC_EnableIRQ((BUTTON_IRQn[Button]));
    }
  }

  return ret;
}
int32_t BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;
  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
  return BSP_ERROR_NONE;
}
int32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return (HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]) == GPIO_PIN_RESET) ? BUTTON_PRESSED : BUTTON_RELEASED;
}
void BSP_PB_IRQHandler (Button_TypeDef Button)
{
  HAL_EXTI_IRQHandler( &hpb_exti[Button] );
}
__weak void BSP_PB_Callback(Button_TypeDef Button)
{
  UNUSED(Button);
}
static void BUTTON_USER_EXTI_Callback(void)
{
  BSP_PB_Callback(BUTTON_USER);
}
static void BUTTON_USER_GPIO_Init(void) {
	BUS_BSP_BUTTON_GPIO_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = USER_BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Use FALLING edge
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);
}


//------------------------------------------------------------------------------
// COM Functions
//------------------------------------------------------------------------------
#if (USE_BSP_COM_FEATURE > 0)
int32_t BSP_COM_Init(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;
  if(COM >= COMn) { ret = BSP_ERROR_WRONG_PARAM; }
  else
  {
     hcom_uart[COM].Instance = COM_USART[COM];
#if (USE_HAL_UART_REGISTER_CALLBACKS == 0U)
    USART1_MspInit(&hcom_uart[COM]);
#else
    if(IsUsart1MspCbValid == 0U)
    { if(BSP_COM_RegisterDefaultMspCallbacks(COM) != BSP_ERROR_NONE) { return BSP_ERROR_MSP_FAILURE; } }
#endif
    if (MX_USART1_UART_Init(&hcom_uart[COM]) != HAL_OK) { ret = BSP_ERROR_PERIPH_FAILURE; }
  }
  return ret;
}
int32_t BSP_COM_DeInit(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;
  if(COM >= COMn) { ret = BSP_ERROR_WRONG_PARAM; }
  else
  {
    hcom_uart[COM].Instance = COM_USART[COM];
#if (USE_HAL_UART_REGISTER_CALLBACKS == 0U)
      USART1_MspDeInit(&hcom_uart[COM]);
#endif
    if(HAL_UART_DeInit(&hcom_uart[COM]) != HAL_OK) { ret = BSP_ERROR_PERIPH_FAILURE; }
  }
  return ret;
}
__weak HAL_StatusTypeDef MX_USART1_UART_Init(UART_HandleTypeDef* huart)
{
  HAL_StatusTypeDef ret = HAL_OK;
  huart->Instance = COM1_UART;
  huart->Init.BaudRate = 115200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart) != HAL_OK) { ret = HAL_ERROR; }
  return ret;
}
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;
  if(COM >= COMn) { ret = BSP_ERROR_WRONG_PARAM; }
  else
  {
    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, USART1_MspInit) != HAL_OK) { ret = BSP_ERROR_PERIPH_FAILURE; }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, USART1_MspDeInit) != HAL_OK) { ret = BSP_ERROR_PERIPH_FAILURE; }
    else { IsUsart1MspCbValid = 1U; }
  }
  return ret;
}
int32_t BSP_COM_RegisterMspCallbacks (COM_TypeDef COM , BSP_COM_Cb_t *Callback)
{
  int32_t ret = BSP_ERROR_NONE;
  if(COM >= COMn) { ret = BSP_ERROR_WRONG_PARAM; }
  else
  {
    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, Callback->pMspInitCb) != HAL_OK) { ret = BSP_ERROR_PERIPH_FAILURE; }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, Callback->pMspDeInitCb) != HAL_OK) { ret = BSP_ERROR_PERIPH_FAILURE; }
    else { IsUsart1MspCbValid = 1U; }
  }
  return ret;
}
#endif
#if (USE_COM_LOG > 0)
int32_t BSP_COM_SelectLogPort(COM_TypeDef COM)
{
  if(COM_ActiveLogPort != COM) { COM_ActiveLogPort = COM; }
  return BSP_ERROR_NONE;
}
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  (void)HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t *)&ch, 1, COM_POLL_TIMEOUT);
  return ch;
}
#endif
static void USART1_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
  COM1_CLK_ENABLE();
  COM1_TX_GPIO_CLK_ENABLE();
  COM1_RX_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = COM1_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = COM1_TX_AF;
  HAL_GPIO_Init(COM1_TX_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = COM1_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = COM1_RX_AF;
  HAL_GPIO_Init(COM1_RX_GPIO_PORT, &GPIO_InitStruct);
}
static void USART1_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  COM1_CLK_DISABLE();
  HAL_GPIO_DeInit(COM1_TX_GPIO_PORT, COM1_TX_PIN);
  HAL_GPIO_DeInit(COM1_RX_GPIO_PORT, COM1_RX_PIN);
}
#endif /* (USE_BSP_COM_FEATURE > 0) */


//------------------------------------------------------------------------------
// BUS Operations (I2C for Sensors) - Copied from Reference File
//------------------------------------------------------------------------------

/******************************* I2C Routines *********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_MspInit(I2C_HandleTypeDef *i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;

  if (i2c_handler->Instance == BUS_I2C2_INSTANCE)
  {
    /* Configure the GPIOs */
    /* Enable GPIO clock */
    BUS_I2C2_SCL_GPIO_CLK_ENABLE();
    BUS_I2C2_SDA_GPIO_CLK_ENABLE();

    /* Configure I2C SCL, SDA as alternate function */
    gpio_init_structure.Pin = BUS_I2C2_SCL_GPIO_PIN | BUS_I2C2_SDA_GPIO_PIN;
    gpio_init_structure.Mode = GPIO_MODE_AF_OD; // Open Drain for I2C
    gpio_init_structure.Pull = GPIO_PULLUP; // External pull-ups are usually present
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_structure.Alternate = BUS_I2C2_SCL_GPIO_AF; // Use AF for SCL
    HAL_GPIO_Init(BUS_I2C2_SCL_GPIO_PORT, &gpio_init_structure);

    gpio_init_structure.Alternate = BUS_I2C2_SDA_GPIO_AF; // Use AF for SDA
    HAL_GPIO_Init(BUS_I2C2_SDA_GPIO_PORT, &gpio_init_structure);

    /* Configure the I2C peripheral */
    /* Enable I2C clock */
    BUS_I2C2_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    BUS_I2C2_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    BUS_I2C2_RELEASE_RESET();

    /* Enable and set I2Cx Interrupt to a lower priority */
    HAL_NVIC_SetPriority(BUS_I2C2_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BUS_I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(BUS_I2C2_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BUS_I2C2_ER_IRQn);
  }
}

/**
  * @brief  DeInitializes I2C MSP.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_MspDeInit(I2C_HandleTypeDef *i2c_handler)
{
  if (i2c_handler->Instance == BUS_I2C2_INSTANCE)
  {
    /* Disable NVIC IRQs */
    HAL_NVIC_DisableIRQ(BUS_I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(BUS_I2C2_ER_IRQn);

    /* DeInit GPIO pins */
    HAL_GPIO_DeInit(BUS_I2C2_SCL_GPIO_PORT, BUS_I2C2_SCL_GPIO_PIN);
    HAL_GPIO_DeInit(BUS_I2C2_SDA_GPIO_PORT, BUS_I2C2_SDA_GPIO_PIN);

    /* Disable I2C clock */
    BUS_I2C2_CLK_DISABLE();
  }
}

/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_Init(I2C_HandleTypeDef *i2c_handler)
{
  if(HAL_I2C_GetState(i2c_handler) == HAL_I2C_STATE_RESET)
  {
    i2c_handler->Instance              = BUS_I2C2_INSTANCE;
    i2c_handler->Init.Timing           = BUS_I2C2_TIMING;
    i2c_handler->Init.OwnAddress1      = 0;
    i2c_handler->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c_handler->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c_handler->Init.OwnAddress2      = 0;
    i2c_handler->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c_handler->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c_handler->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2Cx_MspInit(i2c_handler);
    if (HAL_I2C_Init(i2c_handler) != HAL_OK)
    {
       Error_Handler();
    }

    /** Configure Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(i2c_handler, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
       Error_Handler();
    }

    /** Configure Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(i2c_handler, 0) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

/**
  * @brief  DeInitializes I2C HAL.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_DeInit(I2C_HandleTypeDef *i2c_handler)
{
   if (HAL_I2C_GetState(i2c_handler) != HAL_I2C_STATE_RESET)
   {
     if (HAL_I2C_DeInit(i2c_handler) != HAL_OK)
     {
         Error_Handler();
     }
     I2Cx_MspDeInit(i2c_handler);
   }
}

/**
  * @brief  Reads multiple data.
  * @param  i2c_handler  I2C handler
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @param  MemAddressSize I2C Memory Address Size
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddressSize, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  // Use increased timeout for robustness
  status = HAL_I2C_Mem_Read(i2c_handler, Addr, Reg, MemAddressSize, Buffer, Length, 5000);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("I2Cx_ReadMultiple: HAL_I2C_Mem_Read failed with status %d\r\n", status);
    /* I2C error occurred */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Writes multiple data.
  * @param  i2c_handler  I2C handler
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @param  MemAddressSize I2C Memory Address Size
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddressSize, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  // Use increased timeout for robustness
  status = HAL_I2C_Mem_Write(i2c_handler, Addr, Reg, MemAddressSize, Buffer, Length, 5000);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("I2Cx_WriteMultiple: HAL_I2C_Mem_Write failed with status %d\r\n", status);
    /* Re-Initialize the I2C Bus */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  i2c_handler  I2C handler
  * @param  DevAddress  Target device address
  * @param  Trials  Number of trials
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials)
{
  return (HAL_I2C_IsDeviceReady(i2c_handler, DevAddress, Trials, BUS_I2C2_TIMEOUT_MAX));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  i2c_handler  I2C handler
  * @param  Addr        I2C Address
  * @retval None
  */
static void I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr)
{
  printf("!!! I2C Error Occurred for Addr 0x%02X !!!\r\n", Addr);
  /* De-initialize the I2C communication bus */
  if (HAL_I2C_DeInit(i2c_handler) != HAL_OK)
  {
    Error_Handler();
  }
  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
}

//------------------------------------------------------------------------------
// LINK Operations (Sensor IO) - Copied from Reference File
//------------------------------------------------------------------------------

/**
  * @brief  Initializes Sensors low level I2C.
  * @retval None
  */
void SENSOR_IO_Init(void)
{
  I2Cx_Init(&hI2cHandler);
}

/**
  * @brief  DeInitializes Sensors low level I2C.
  * @retval None
  */
void SENSOR_IO_DeInit(void)
{
  I2Cx_DeInit(&hI2cHandler);
}

/**
  * @brief  Writes a single data byte.
  * @param  Addr    Device address on BUS
  * @param  Reg     The target register address to write
  * @param  Value   The target register value to be written
  * @retval None
  */
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1);
}

/**
  * @brief  Reads a single data byte.
  * @param  Addr    Device address on BUS
  * @param  Reg     The target register address to read
  * @retval Data read from the register
  */
uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;
  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hI2cHandler);
  if (state != HAL_I2C_STATE_READY) {
     printf("SENSOR_IO_Read: I2C State not READY (%d) before read\r\n", state);
  }

  if (I2Cx_ReadMultiple(&hI2cHandler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &read_value, 1) != HAL_OK)
  {
      printf("!!! SENSOR_IO_Read: I2Cx_ReadMultiple failed (Addr 0x%02X, Reg 0x%02X) !!!\r\n", Addr, Reg);
      read_value = 0xFF; // Indicate read error
  }
  return read_value;
}

/**
  * @brief  Reads multiple data bytes.
  * @param  Addr    Device address on BUS
  * @param  Reg     The target register address to read
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval 0 if success, non-zero if failure (matches lsm6dsl.h)
  */
uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 /* NOTE: Removed (Reg | 0x80) as it was causing I2C failures. */
 if(I2Cx_ReadMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length) != HAL_OK)
 {
   return 1; // Indicate error
 }
 return 0; // Indicate success
}

/**
  * @brief  Writes multiple data bytes.
  * @param  Addr    Device address on BUS
  * @param  Reg     The target register address to write
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval None
  */
void SENSOR_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  /* NOTE: Removed (Reg | 0x80) as it was causing I2C failures. */
  I2Cx_WriteMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Checks if target device is ready for communication.
  * @param  DevAddress  Target device address (I2C 8-bit)
  * @param  Trials      Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef SENSOR_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
  return I2Cx_IsDeviceReady(&hI2cHandler, DevAddress, Trials);
}

/**
  * @brief  Delay function used in Sensor low level driver.
  * @param  Delay   Delay in milliseconds
  * @retval None
  */
void SENSOR_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/* ... (NFC functions - omitted) ... */


/**
 * @}
 */ /* End of B_L475E_IOT01A1_LOW_LEVEL_Private_Functions */

/**
 * @}
 */ /* End of B_L475E_IOT01A1_LOW_LEVEL */

/**
 * @}
 */ /* End of B_L475E_IOT01A1 */

/**
 * @}
 */ /* End of BSP */
