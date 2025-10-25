/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __B_L475E_IOT01A1_H
#define __B_L475E_IOT01A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "b_l475e_iot01a1_conf.h"
#include "b_l475e_iot01a1_errno.h"
#include "main.h" // For HAL_Delay
#include "stm32l4xx_hal_i2c.h" // Include I2C HAL driver

#if (USE_BSP_COM_FEATURE > 0)
  #if (USE_COM_LOG > 0)
    #if defined(__ICCARM__) || defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
      #include <stdio.h>
    #endif
  #endif
#endif
/** @addtogroup BSP
 * @{
 */

/** @defgroup B_L475E_IOT01A1
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Constants LOW LEVEL Exported Constants
  * @{
  */
#define __B_L475E_IOT01A1_BSP_VERSION_MAIN   (uint32_t)(0x01)
#define __B_L475E_IOT01A1_BSP_VERSION_SUB1   (uint32_t)(0x01)
#define __B_L475E_IOT01A1_BSP_VERSION_SUB2   (uint32_t)(0x08)
#define __B_L475E_IOT01A1_BSP_VERSION_RC     (uint32_t)(0x00)
#define __B_L475E_IOT01A1_BSP_VERSION        ((__B_L475E_IOT01A1_BSP_VERSION_MAIN << 24)\
                                                    |(__B_L475E_IOT01A1_BSP_VERSION_SUB1 << 16)\
                                                    |(__B_L475E_IOT01A1_BSP_VERSION_SUB2 << 8 )\
                                                    |(__B_L475E_IOT01A1_BSP_VERSION_RC))

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Types B_L475E_IOT01A1 LOW LEVEL Exported Types
 * @{
 */
#if !defined (USE_B_L475E_IOT01A1)
 #define USE_B_L475E_IOT01A1
#endif
#ifndef USE_BSP_COM_FEATURE
   #define USE_BSP_COM_FEATURE                  1U
#endif

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_LED B_L475E_IOT01A1 LOW LEVEL LED
 * @{
 */
#define LEDn                              1U
#define LED2_PIN                          GPIO_PIN_14
#define LED2_GPIO_PORT                    GPIOB
#define LED2_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

#define BUS_GPIO_INSTANCE GPIO
#define BUS_BSP_LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_BSP_LED_GPIO_PORT GPIOB
#define BUS_BSP_LED_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_BSP_LED_GPIO_PIN GPIO_PIN_14

typedef enum
{
  LED2 = 0,
  LED_GREEN = LED2,
}Led_TypeDef;
/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_BUTTON B_L475E_IOT01A1 LOW LEVEL BUTTON
 * @{
 */
#define BUTTON_RELEASED                   0U
#define BUTTON_PRESSED                    1U
#define BUTTONn                           1U

#define BUS_GPIO_INSTANCE GPIO
#define BUS_BSP_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define BUS_BSP_BUTTON_GPIO_PIN GPIO_PIN_13
#define BUS_BSP_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()
#define BUS_BSP_BUTTON_GPIO_PORT GPIOC

#define USER_BUTTON_PIN                   GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT              GPIOC
#define USER_BUTTON_EXTI_IRQn              EXTI15_10_IRQn
#define USER_BUTTON_EXTI_LINE              EXTI_LINE_13
#define H_EXTI_13             hpb_exti[BUTTON_USER]

typedef enum
{
  BUTTON_USER = 0U,
}Button_TypeDef;
#define BUTTON_KEY BUTTON_USER

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;
#ifndef BSP_BUTTON_USER_IT_PRIORITY
  #define BSP_BUTTON_USER_IT_PRIORITY            15U
#endif
/**
 * @}
 */
/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_COM B_L475E_IOT01A1 LOW LEVEL COM
 * @{
 */
#if (USE_BSP_COM_FEATURE > 0)
#define COMn                             1U
#define COM1_UART                        USART1
#define COM1_CLK_ENABLE()                __HAL_RCC_USART1_CLK_ENABLE()
#define COM1_CLK_DISABLE()               __HAL_RCC_USART1_CLK_DISABLE()

#define COM1_TX_PIN                      GPIO_PIN_6
#define COM1_TX_GPIO_PORT                GPIOB
#define COM1_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define COM1_TX_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define COM1_TX_AF                       GPIO_AF7_USART1

#define COM1_RX_PIN                      GPIO_PIN_7
#define COM1_RX_GPIO_PORT                GPIOB
#define COM1_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define COM1_RX_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define COM1_RX_AF                       GPIO_AF7_USART1

#define BUS_USART1_INSTANCE USART1
#define BUS_USART1_TX_GPIO_PIN GPIO_PIN_6
#define BUS_USART1_TX_GPIO_PORT GPIOB
#define BUS_USART1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_USART1_TX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_USART1_TX_GPIO_AF GPIO_AF7_USART1
#define BUS_USART1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_USART1_RX_GPIO_PORT GPIOB
#define BUS_USART1_RX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_USART1_RX_GPIO_PIN GPIO_PIN_7
#define BUS_USART1_RX_GPIO_AF GPIO_AF7_USART1

#define COM_POLL_TIMEOUT                 1000

typedef enum
{
  COM1 = 0U
}COM_TypeDef;

typedef enum { COM_WORDLENGTH_8B = UART_WORDLENGTH_8B, COM_WORDLENGTH_9B = UART_WORDLENGTH_9B } COM_WordLengthTypeDef;
typedef enum { COM_STOPBITS_1 = UART_STOPBITS_1, COM_STOPBITS_2 = UART_STOPBITS_2 } COM_StopBitsTypeDef;
typedef enum { COM_PARITY_NONE = UART_PARITY_NONE, COM_PARITY_EVEN = UART_PARITY_EVEN, COM_PARITY_ODD = UART_PARITY_ODD } COM_ParityTypeDef;
typedef enum { COM_HWCONTROL_NONE = UART_HWCONTROL_NONE, COM_HWCONTROL_RTS = UART_HWCONTROL_RTS, COM_HWCONTROL_CTS = UART_HWCONTROL_CTS, COM_HWCONTROL_RTS_CTS = UART_HWCONTROL_RTS_CTS } COM_HwFlowCtlTypeDef;

typedef struct
{
  uint32_t             BaudRate;
  COM_WordLengthTypeDef  WordLength;
  COM_StopBitsTypeDef  StopBits;
  COM_ParityTypeDef    Parity;
  COM_HwFlowCtlTypeDef HwFlowCtl;
}COM_InitTypeDef;

#define MX_UART_InitTypeDef          COM_InitTypeDef

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
typedef struct
{
  void (* pMspInitCb)(UART_HandleTypeDef *);
  void (* pMspDeInitCb)(UART_HandleTypeDef *);
} BSP_COM_Cb_t;
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1U) */

extern UART_HandleTypeDef hcom_uart[COMn];
#define huart1 hcom_uart[COM1]

#endif /* (USE_BSP_COM_FEATURE > 0) */

#ifndef USE_COM_LOG
  #define USE_COM_LOG                           1U
#endif
/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_BUS B_L475E_IOT01A1 LOW LEVEL BUS
 * @{
 */
/*############################### I2C2 #######################################*/
/* I2C2 used for MOTION and ENV sensors */
#define BUS_I2C2_INSTANCE                 I2C2
#define BUS_I2C2_SCL_GPIO_PIN             GPIO_PIN_10
#define BUS_I2C2_SCL_GPIO_PORT            GPIOB
#define BUS_I2C2_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_I2C2_SCL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C2_SCL_GPIO_AF              GPIO_AF4_I2C2

#define BUS_I2C2_SDA_GPIO_PIN             GPIO_PIN_11
#define BUS_I2C2_SDA_GPIO_PORT            GPIOB
#define BUS_I2C2_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_I2C2_SDA_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C2_SDA_GPIO_AF              GPIO_AF4_I2C2

#define BUS_I2C2_CLK_ENABLE()             __HAL_RCC_I2C2_CLK_ENABLE()
#define BUS_I2C2_CLK_DISABLE()            __HAL_RCC_I2C2_CLK_DISABLE()
#define BUS_I2C2_FORCE_RESET()            __HAL_RCC_I2C2_FORCE_RESET()
#define BUS_I2C2_RELEASE_RESET()          __HAL_RCC_I2C2_RELEASE_RESET()

#define BUS_I2C2_EV_IRQn                  I2C2_EV_IRQn
#define BUS_I2C2_ER_IRQn                  I2C2_ER_IRQn
#define BUS_I2C2_TIMING                   0x00000E14 // Timing for 400kHz @80MHz (check if SystemClock is 80MHz)
#define BUS_I2C2_TIMEOUT_MAX              0x3000

/**
  * @}
  */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Variables
  * @{
  */
extern EXTI_HandleTypeDef hpb_exti[BUTTONn];
extern I2C_HandleTypeDef hI2cHandler;
/**
  * @}
  */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Functions B_L475E_IOT01A1 LOW LEVEL Exported Functions
 * @{
 */
/* Basic BSP functions */
int32_t  BSP_GetVersion(void);
int32_t  BSP_LED_Init(Led_TypeDef Led);
int32_t  BSP_LED_DeInit(Led_TypeDef Led);
int32_t  BSP_LED_On(Led_TypeDef Led);
int32_t  BSP_LED_Off(Led_TypeDef Led);
int32_t  BSP_LED_Toggle(Led_TypeDef Led);
int32_t  BSP_LED_GetState(Led_TypeDef Led);
int32_t  BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
int32_t  BSP_PB_DeInit(Button_TypeDef Button);
int32_t  BSP_PB_GetState(Button_TypeDef Button);
void     BSP_PB_Callback(Button_TypeDef Button);
void     BSP_PB_IRQHandler(Button_TypeDef Button);

/* COM functions */
#if (USE_BSP_COM_FEATURE > 0)
int32_t  BSP_COM_Init(COM_TypeDef COM);
int32_t  BSP_COM_DeInit(COM_TypeDef COM);
#if (USE_COM_LOG > 0)
int32_t  BSP_COM_SelectLogPort(COM_TypeDef COM);
#endif
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM);
int32_t BSP_COM_RegisterMspCallbacks(COM_TypeDef COM , BSP_COM_Cb_t *Callback);
#endif
HAL_StatusTypeDef MX_USART1_UART_Init(UART_HandleTypeDef* huart);
#endif

/* Sensor IO functions (From Reference File) */
void    SENSOR_IO_Init(void);
void    SENSOR_IO_DeInit(void);
void    SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg);
// --- FIX: Change return type to uint16_t to match lsm6dsl.h ---
uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
// ----------------------------------------------------------------
void    SENSOR_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
HAL_StatusTypeDef SENSOR_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
void    SENSOR_IO_Delay(uint32_t Delay);
/* NFC IO functions (If needed) */
// (Declarations omitted for clarity)

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __B_L475E_IOT01A1__H */
