/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ad7730.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t tx_buffer_3[BUFFER_SIZE_SPI3];
uint8_t rx_buffer_3[BUFFER_SIZE_SPI3];
uint8_t shadow_buffer_3_0[BUFFER_SIZE_SPI3];
uint8_t shadow_buffer_3_1[BUFFER_SIZE_SPI3];
uint8_t shadow_buffer_3_2[BUFFER_SIZE_SPI3];
uint8_t tx_buffer_1[BUFFER_SIZE_SPI1];
uint8_t rx_buffer_1[BUFFER_SIZE_SPI1];

Controller_State slave_state = READ_CONFIG;
uint8_t togg = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static inline void cp_clean_buffer(uint8_t dest_buffer[], uint8_t dest_size, uint8_t src_buffer[], uint8_t src_size) {
  for (uint8_t idx = 0; idx < src_size; idx++) {
    dest_buffer[idx] = src_buffer[idx];
  }
  /*
  for (uint8_t idx = src_size; idx < dest_size; idx++) {
    dest_buffer[idx] = 0x00;
  }*/
}

static inline void set_state(uint8_t command_idx) {
  switch (command_idx) {
    case 0xFD:
      slave_state = MONITOR;
      break;

    case 0xFC:
      slave_state = PROVIDE_DATA;
      break;

    default:
      slave_state = READ_CONFIG;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  tx_buffer_3[63] = slave_state;
  shadow_buffer_3_0[63] = slave_state;
  shadow_buffer_3_1[63] = slave_state;

  if (hspi->Instance == SPI3) {
    switch (0){//togg % 3) {
      case 1:
        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_0, rx_buffer_3, BUFFER_SIZE_SPI3);
        break;
      case 0:
        HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buffer_3, rx_buffer_3, BUFFER_SIZE_SPI3);
        break;
      case 2:
        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_1, rx_buffer_3, BUFFER_SIZE_SPI3);
        break;
    }
    set_state(rx_buffer_3[0]);
    togg++;
  }

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();

  static struct Transducer_SS_Info device_infos[8] = {
      {CS0_Pin,CS0_GPIO_Port},{CS1_Pin,CS1_GPIO_Port},
      {CS2_Pin,CS2_GPIO_Port},{CS3_Pin,CS3_GPIO_Port},
      {CS4_Pin,CS4_GPIO_Port},{CS5_Pin,CS5_GPIO_Port},
      {CS6_Pin,CS6_GPIO_Port},{CS7_Pin,CS7_GPIO_Port}
  };

  for (unsigned int idx = 0; idx < TRANSDUCER_NUMBER; idx++) {
    HAL_GPIO_WritePin(device_infos[idx].ss_port, device_infos[idx].ss_pin, GPIO_PIN_SET);
  }

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ad7730_softreset(0, device_infos);
  ad7730_setup_device(0, device_infos);
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buffer_3, rx_buffer_3, BUFFER_SIZE_SPI3);
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (slave_state) {

      case READ_CONFIG:
        break;

      case PROVIDE_DATA:
        break;

      case MONITOR:
        ad7730_read_all_inputs(shadow_buffer_3_0, device_infos);
        cp_clean_buffer(tx_buffer_3, BUFFER_SIZE_SPI3, shadow_buffer_3_0, BUFFER_SIZE_SPI3);

//        ad7730_read_input(0, rx_buffer_1, device_infos, CHANNEL_A2);
//        cp_clean_buffer(tx_buffer_3, BUFFER_SIZE_SPI3, rx_buffer_1, BUFFER_SIZE_SPI1);
        break;

    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
