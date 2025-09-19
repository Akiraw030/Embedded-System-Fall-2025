/**
  ******************************************************************************
  * @file    Wifi/WiFi_Client_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lsm6dsl.h"

/* Private defines -----------------------------------------------------------*/

#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */
#define SSID     "Akiraw3"
#define PASSWORD "0910993640"

uint8_t RemoteIP[] = {10,89,11,238};
#define RemotePORT	8002

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

#define FUNC_CFG_ACCESS_FUNC_EN    0x80   /* set to access embedded function bank (FUNC_CFG_ACCESS) */
#define CTRL10_C_FUNC_EN           0x04   /* CTRL10_C: enable embedded func block */
#define CTRL10_C_SIGN_MOTION_EN    0x01   /* CTRL10_C: enable significant motion function */
#define INT1_CTRL_INT1_SIGN_MOT    0x40   /* INT1_CTRL: route sign motion to INT1 */
#define FUNC_SRC1_SIGN_MOTION_IA   0x40   /* FUNC_SRC1: SIGN_MOTION_IA bit */

#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
static uint8_t RxData [500];
int32_t Socket = -1;

volatile uint8_t significant_motion_flag = 0;
static void Sensor_SignificantMotion_Init(void);

/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
void Configure_INT1_Pin(void);
void Sensor_SignificantMotion_Init(void);
void SendSignificantMotionEvent(void);


extern  SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t  MAC_Addr[6] = {0};
  uint8_t  IP_Addr[4] = {0};
  uint8_t TxData[] = "STM32 : Hello!\n";
  uint16_t Datalen;
  int32_t ret;
  int16_t Trials = CONNECTION_TRIAL_MAX;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Configure LED2 */
  BSP_LED_Init(LED2);

#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);
  BSP_ACCELERO_Init();
  Sensor_SignificantMotion_Init();
#endif /* TERMINAL_USE */

  Configure_INT1_Pin();

  TERMOUT("****** WIFI Module in TCP Client mode demonstration ****** \n\n");
  TERMOUT("TCP Client Instructions :\n");
  TERMOUT("1- Make sure your Phone is connected to the same network that\n");
  TERMOUT("   you configured using the Configuration Access Point.\n");
  TERMOUT("2- Create a server by using the android application TCP Server\n");
  TERMOUT("   with port(8002).\n");
  TERMOUT("3- Get the Network Name or IP Address of your Android from the step 2.\n\n");



  /*Initialize  WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    TERMOUT("> WIFI Module Initialized.\n");
    if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]);
    }
    else
    {
      TERMOUT("> ERROR : CANNOT get MAC address\n");
      BSP_LED_On(LED2);
    }

    if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi module connected \n");
      if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
      {
        TERMOUT("> es-wifi module got IP Address : %d.%d.%d.%d\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]);

        TERMOUT("> Trying to connect to Server: %d.%d.%d.%d:%d ...\n",
               RemoteIP[0],
               RemoteIP[1],
               RemoteIP[2],
               RemoteIP[3],
							 RemotePORT);

        while (Trials--)
        {
          if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
          {
            TERMOUT("> TCP Connection opened successfully.\n");
            Socket = 0;
            break;
          }
        }
        if(Socket == -1)
        {
          TERMOUT("> ERROR : Cannot open Connection\n");
          BSP_LED_On(LED2);
        }
      }
      else
      {
        TERMOUT("> ERROR : es-wifi module CANNOT get IP address\n");
        BSP_LED_On(LED2);
      }
    }
    else
    {
      TERMOUT("> ERROR : es-wifi module NOT connected\n");
      BSP_LED_On(LED2);
    }
  }
  else
  {
    TERMOUT("> ERROR : WIFI Module cannot be initialized.\n");
    BSP_LED_On(LED2);
  }

  int16_t pDataXYZ[3];
  char msg[128];
  uint16_t sent_len;
  uint16_t rcv_len;
  int32_t r;

  while(1)
  {
	  BSP_LED_Toggle(LED2);
      /* Read accelerometer */
      BSP_ACCELERO_AccGetXYZ(pDataXYZ);
      TERMOUT("ACC X=%d, Y=%d, Z=%d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);

      /* If significant-motion was flagged by EXTI, notify host immediately */
      /* If significant-motion was flagged by EXTI, handle it here (non-ISR) */
      if (significant_motion_flag)
      {
        /* Consume the flag immediately */
        significant_motion_flag = 0;

        /*
         * CRITICAL STEP: Read the FUNC_SRC register to acknowledge the event.
         * This makes the sensor's INT1 pin go low, clearing the physical interrupt condition.
         * This MUST be done before re-enabling the EXTI interrupt.
         */
        uint8_t func_src = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FUNC_SRC);
        TERMOUT("> FUNC_SRC read = 0x%02X\n", func_src);

        /* Only proceed if the source was actually a significant motion event */
        if (1/*func_src & FUNC_SRC1_SIGN_MOTION_IA*/)
        {
        	/* Now that the interrupt is cleared, we can safely perform software debouncing */
        	static uint32_t last_sig_tick = 0;
        	const uint32_t debounce_ms = 500; /* Increased debounce for stability */
	          uint32_t now = HAL_GetTick();

	          if (now - last_sig_tick < debounce_ms)
	          {
	            TERMOUT(">> SIGN_MOTION ignored (debounce)\n");
	          }
	          else
	          {
	            last_sig_tick = now;
	            TERMOUT(">> Significant motion detected! Sending event to host...\n");

	            /* Prepare and send the JSON event */
	            char outbuf[96];
	            int n = snprintf(outbuf, sizeof(outbuf),
	                             "{\"event\":\"significant_motion\",\"ts\":%lu}\n",
	                             (unsigned long)HAL_GetTick());

	            if (n > 0 && n < (int)sizeof(outbuf))
	            {
	              uint16_t sent = 0;
	              int32_t rr = WIFI_STATUS_ERROR;
	              const int send_retries = 3;
	              for (int retry = 0; retry < send_retries; ++retry)
	              {
	                /* Check socket and try to reconnect if needed */
	                if (Socket == -1) {
	                    TERMOUT("> Socket closed, attempting reconnect...\n");
	                    if (WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK) {
	                        Socket = 0;
	                        TERMOUT("> Reconnect success.\n");
	                    } else {
	                        TERMOUT("> Reconnect failed.\n");
	                        HAL_Delay(200);
	                        continue;
	                    }
	                }

	                rr = WIFI_SendData(Socket, (uint8_t*)outbuf, (uint16_t)n, &sent, WIFI_WRITE_TIMEOUT);
	                if (rr == WIFI_STATUS_OK) {
	                  TERMOUT("> Sent SIGNIFICANT_MOTION event (%u bytes)\n", sent);
	                  break;
	                } else {
	                  TERMOUT("> WARN: WIFI_SendData failed (err=%ld), retrying...\n", (long)rr);
	                  Socket = -1; /* Assume connection is lost on failure */
	                  HAL_Delay(100);
	                }
	              }

	              if (rr != WIFI_STATUS_OK) {
	                TERMOUT("> ERROR: Failed to send event after retries.\n");
	              }
	            }
	          }
        }
        else
        {
          TERMOUT("> Spurious interrupt on INT1, event ignored.\n");
        }

        /*
         * FINAL STEP: Now that the sensor's INT1 pin is low and we've handled the event,
         * it is safe to re-enable the EXTI interrupt for future events.
         */
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
      }

      /* If connected, send accel data to server as newline-terminated JSON */
      if (Socket != -1)
      {
          int n = snprintf(msg, sizeof(msg),
                           "{\"type\":\"accel\",\"ts\":%lu,\"x\":%d,\"y\":%d,\"z\":%d}\n",
                           (unsigned long)HAL_GetTick(),
                           pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);

          if (n > 0 && n < (int)sizeof(msg))
          {
              r = WIFI_SendData(Socket, (uint8_t*)msg, (uint16_t)n, &sent_len, WIFI_WRITE_TIMEOUT);
              if (r != WIFI_STATUS_OK)
              {
                  TERMOUT("> ERROR : Failed to send accel data (err=%ld)\n", (long)r);
                  /* Optionally set Socket = -1 and try to reconnect */
              }
              else
              {
                  TERMOUT("> Sent %u bytes\n", sent_len);
              }
          }
      }

      /* Short, non-blocking check for incoming data (100 ms timeout) */
      if (Socket != -1)
      {
          r = WIFI_ReceiveData(Socket, RxData, sizeof(RxData)-1, &rcv_len, 100);
          if (r == WIFI_STATUS_OK && rcv_len > 0)
          {
              RxData[rcv_len] = 0;
              TERMOUT("Received: %s\n", RxData);
          }
          else if (r != WIFI_STATUS_OK && r != WIFI_STATUS_TIMEOUT)
          {
              TERMOUT("> WARNING: WIFI_ReceiveData returned error %ld\n", (long)r);
              /* Optionally handle reconnect here */
          }
      }

      HAL_Delay(500); /* sample interval */
  }

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library TERMOUT function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */

void Configure_INT1_Pin(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Ensure GPIOD clock enabled */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* PD11 configured as input + EXTI on rising edge (LSM6DSL INT1 is active-high) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI IRQ is EXTI15_10 for pins 10..15 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ---------- ADDED: Simplified sender helper used by EXTI callback --------- */
void SendSignificantMotionEvent(void)
{
  if (Socket == -1) return; /* not connected */

  /* Build a small JSON line. Adjust length if you want timestamp. */
  const char msg[] = "{\"event\":\"significant_motion\"}\n";
  uint16_t sent_len = 0;
  if (WIFI_SendData(Socket, (uint8_t *)msg, sizeof(msg)-1, &sent_len, WIFI_WRITE_TIMEOUT) != WIFI_STATUS_OK)
  {
    TERMOUT("> ERROR : Failed to send motion event\n");
  }
  else
  {
    TERMOUT("> Sent motion event to host\n");
  }
}

/* ---------- ADDED: initialize the LSM6DSL significant motion feature ----- */
static void Sensor_SignificantMotion_Init(void)
{
  uint8_t reg;

  /* Initialize sensor IO (I2C/SPI) - SENSOR_IO_Init() is provided by the sensor driver
     If BSP_ACCELERO_Init already called SENSOR_IO_Init() you can omit this, harmless if repeated. */
  SENSOR_IO_Init();

  /* 1) Set SM_THS in embedded function bank
     - enable access to embedded registers bank A by writing FUNC_CFG_ACCESS = 0x80
     - write to SM_THS (0x13) the threshold you want (e.g. 0x06 or 0x08). The value
       is the number of steps/threshold used by the algorithm (experiment).
     - disable embedded register access (write 0x00 to FUNC_CFG_ACCESS)
  */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS, FUNC_CFG_ACCESS_FUNC_EN);
  /* SM_THS register address in driver is defined as LSM6DSL_ACC_GYRO_SM_STEP_THS (0x13) */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_SM_STEP_THS, 0x02); /* try 0x06 - tune as needed */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS, 0x00);

  /* 2) Make sure accelerometer ODR is >= 26 Hz (significant motion works at 26 Hz or higher).
     The BSP ACC init may already have configured CTRL1_XL; to be sure, set ODR to 26Hz or 52Hz:
     CTRL1_XL (0x10) upper bits are ODR. We'll set 52 Hz as an example (LSM6DSL_ODR_52Hz = 0x30).
     Keep FS (full scale) bits; read/modify/write. */
  reg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);
  reg &= ~(0xF0); /* clear ODR bits */
  reg |= LSM6DSL_ODR_52Hz; /* or LSM6DSL_ODR_104Hz etc. */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, reg);

  /* 3) Enable embedded function and sign-motion in CTRL10_C:
     set FUNC_EN and SIGN_MOTION_EN bits in CTRL10_C (0x19)
     -> CTRL10_C |= 0x04 | 0x01
  */
  reg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C);
  reg |= (CTRL10_C_FUNC_EN | CTRL10_C_SIGN_MOTION_EN);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, reg);

  /* 4) Route the significant motion event to INT1 pin:
     Set the INT1_SIGN_MOT bit in INT1_CTRL (0x0D) */
  reg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);
  reg |= INT1_CTRL_INT1_SIGN_MOT;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, reg);

  /* Optionally: read back/print registers for debug */
  reg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C);
  TERMOUT("> CTRL10_C after config = 0x%02X\n", reg);
  reg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);
  TERMOUT("> INT1_CTRL after config = 0x%02X\n", reg);
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: TERMOUT("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1)
  {
    SPI_WIFI_ISR();
    return;
  }

  /* Check if the interrupt is from the motion sensor's pin */
  if (GPIO_Pin == GPIO_PIN_11)
  {
    /*
     * Immediately disable the interrupt to prevent re-entry.
     * The main loop will re-enable it after handling the event.
    */
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    significant_motion_flag = 1;
  }
}


void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}
