/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* RX_BUFFER FORMAT
 * shadow_buffer[0:slave_state, 1:update_mode_params, 2-3:mode_params{w/o channel}, 4:update_filter_params, 5-7:filter_params]
 */
uint8_t rx_buffer_3[BUFFER_SIZE_SPI3] = {READ_CONFIG,0,0x51,0xB4,0,0x08,0x43,0x00, //Configuration row
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0};

/* SHADOW_BUFFER FORMAT
 * shadow_buffer[0...47:channel_1-16, 63:slave_state]
 */
uint8_t shadow_buffer_3_0[BUFFER_SIZE_SPI3];
uint8_t shadow_buffer_3_1[BUFFER_SIZE_SPI3];

uint8_t tx_buffer_1[BUFFER_SIZE_SPI1];
uint8_t rx_buffer_1[BUFFER_SIZE_SPI1];

/*
 * DEBUG_BUFFER
 */
uint8_t shadow_buffer_3_2[BUFFER_SIZE_SPI3] = {0,0,1,0,0,1,0,0,
											   1,0,0,1,0,0,1,0,
											   0,1,0,0,1,0,0,1,
											   0,0,1,0,0,1,0,0,
											   1,0,0,1,0,0,1,0,
											   0,1,0,0,1,0,0,1,
											   0,0,1,0,0,1,0,0,
											   1,0,0,1,0,0,0,0};

Controller_State slave_state = READ_CONFIG;
uint8_t active_buffer = 0;
uint8_t buffer_updated = 0;

// Parameters
uint8_t AD7730_REGISTER_SIZE[TRANSDUCER_NUMBER] = {1, 3, 2, 3, 1, 3, 3, 3};
uint8_t mode_register[2] = {0x51, 0xB4};
uint8_t filter_register[3] = {0x08,0x43, 0x00};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static inline void copy_buffer(uint8_t dest_buffer[], uint8_t dest_size, uint8_t src_buffer[], uint8_t src_size) {
  for (uint8_t idx = 0; idx < src_size; idx++) {
    dest_buffer[idx] = src_buffer[idx];
  }
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

  if (hspi->Instance == SPI3) {
    switch (active_buffer) {
      case 0:
        shadow_buffer_3_1[63] = slave_state;
        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_1, rx_buffer_3, BUFFER_SIZE_SPI3);
        buffer_updated = 1;
        break;

      case 1:
        shadow_buffer_3_0[63] = slave_state;
        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_0, rx_buffer_3, BUFFER_SIZE_SPI3);
        buffer_updated = 1;
        break;
    }

    set_state(rx_buffer_3[0]);
    if (rx_buffer_3[1] != 0)
    {
    	mode_register[0] = rx_buffer_3[2];
    	mode_register[1] = rx_buffer_3[3];
    }
    if (rx_buffer_3[4] != 0)
    {
    	filter_register[0] = rx_buffer_3[5];
    	filter_register[1] = rx_buffer_3[6];
    	filter_register[2] = rx_buffer_3[7];
    }
  }

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  static struct Transducer_SS_Info slave_infos[8] = { { CS0_Pin, CS0_GPIO_Port }, { CS1_Pin, CS1_GPIO_Port }, { CS2_Pin,
      CS2_GPIO_Port }, { CS3_Pin, CS3_GPIO_Port }, { CS4_Pin, CS4_GPIO_Port }, { CS5_Pin, CS5_GPIO_Port }, { CS6_Pin,
      CS6_GPIO_Port }, { CS7_Pin, CS7_GPIO_Port } };

  static struct Transducer_RDY_Info ready_infos[8] = { { RDY0_Pin, RDY0_GPIO_Port }, { RDY1_Pin, RDY1_GPIO_Port }, {
      RDY2_Pin, RDY2_GPIO_Port }, { RDY3_Pin, RDY3_GPIO_Port }, { RDY4_Pin, RDY4_GPIO_Port },
      { RDY5_Pin, RDY5_GPIO_Port }, { RDY6_Pin, RDY6_GPIO_Port }, { RDY7_Pin, RDY7_GPIO_Port } };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
    ad7730_softreset(dev_idx, slave_infos);
  }
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

        /*
         * Reason for using six instead of one big loop:
         *
         * To give load cell controllers as much time
         * as possible for processing the input without waiting
         * data is send to the controllers one by one.
         * Doing so the time used for sending data to the
         * remaining controllers gives the first one time to process the input.
         *
         */
        for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
          ad7730_set_filter(dev_idx, slave_infos);
        }

        for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
          uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A1};
          ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
        }

        //TODO Make do while loop
        for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
          while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
        	  //uint8_t dbg = dev_idx;
          }
          ad7730_read_register(dev_idx, REG_DATA_REGISTER, &tx_buffer_3[dev_idx * 6], slave_infos);
        }



        for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
          ad7730_set_filter(dev_idx, slave_infos);
        }

        for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
          uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A2};
          ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
        }

        for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
          while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
        	  //uint8_t dbg = dev_idx;
          }
          ad7730_read_register(dev_idx, REG_DATA_REGISTER, &tx_buffer_3[(dev_idx * 6) + 3], slave_infos);
        }

        /*
         * Switch shadow buffer when new data is available
         * and set indicator flag for SPI-slave callback
         */
        if (buffer_updated == 1) {
          switch (active_buffer) {
            case 0:
              copy_buffer(shadow_buffer_3_0, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
              break;

            case 1:
              copy_buffer(shadow_buffer_3_1, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
              break;
          }
          active_buffer ^= 1;
          buffer_updated = 0;
        }

        break;

    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
