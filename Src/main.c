
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32l0xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);



uint8_t write_byte(uint8_t data)
{

	uint8_t data_out;
    uint8_t read_data;

	// wait for spi transmitter readiness
	while ((SPI2->SR & SPI_SR_TXE) == RESET );
	data_out = data;
    SPI2->DR = data_out;
    // wait while a transmission complete
	while ((SPI2->SR & SPI_SR_RXNE) == RESET );
    read_data = SPI2->DR;
	
	return read_data;

	
}







/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(void)
{

	int i,j,k;

	char message[256];


  	/* MCU Configuration----------------------------------------------------------*/

  	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  	HAL_Init();

  	/* Configure the system clock */
  	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_SPI1_Init();
	// enable spi1
	SPI1->CR1 |= SPI_CR1_SPE;
	MX_SPI2_Init();
	SPI2->CR2 &= ~SPI_CR2_TXEIE;   // disable txe interrupt
	SPI2->CR2 &= ~SPI_CR2_ERRIE;   // disable error interrupt
	SPI2->CR2 |= SPI_CR2_RXNEIE;   // enable rxne interrupt
	// enable spi2
	SPI2->CR1 |= SPI_CR1_SPE;

  	MX_USART1_UART_Init();
  	
  
  	HAL_GPIO_WritePin(led_out_GPIO_Port, led_out_Pin, GPIO_PIN_RESET); // turn led on

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	uint8_t spi2_out_data_buffer[128];
	uint8_t spi2_in_data_buffer[128];

	uint8_t data_out;
    uint8_t read_data;


	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//                 RESET
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// reset spi2 cs pin
    spi2cs_out_GPIO_Port->BRR = spi2cs_out_Pin ; 	
	// transmit 0x1e                             	
	write_byte( 0x1e);                         	
	// set spi2 cs pin                           	
    spi2cs_out_GPIO_Port->BSRR = spi2cs_out_Pin ;	
	HAL_Delay(3);
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	uint16_t sensor_prom[7];

	for(i=1; i<7; i++)
	{
		//send read prom command
		// reset spi2 cs pin
    	spi2cs_out_GPIO_Port->BRR = spi2cs_out_Pin ;
		// transmit command with address 
		write_byte( 0xa0 + (((uint8_t)i)<<1));

		// read ms byte
		sensor_prom[i] = write_byte(0x55);
		sensor_prom[i] <<= 8;
		// read ls byte
		sensor_prom[i] += write_byte(0x55);

		// set spi2 cs pin
    	spi2cs_out_GPIO_Port->BSRR = spi2cs_out_Pin ;
	}


	uint32_t pressure;
	uint32_t temperature;
	int32_t dT;
	int32_t actual_temperature;
	int32_t OFF;
	int32_t SENS;
	int32_t P;

//*
	while(1)
	{

		//send start conversion D1 OSR 1024 command
		// reset spi2 cs pin
    	spi2cs_out_GPIO_Port->BRR = spi2cs_out_Pin ;
		// transmit command  
		write_byte(0x44);
		// set spi2 cs pin
    	spi2cs_out_GPIO_Port->BSRR = spi2cs_out_Pin ;

		// pause 3 mS
		HAL_Delay(3);
		

		//send read adc command
		// reset spi2 cs pin
    	spi2cs_out_GPIO_Port->BRR = spi2cs_out_Pin ;
		// transmit command 
		write_byte(0x00);

		// read ms byte
		write_byte(0x55);
		pressure = 0;
		// read ls byte
		pressure += write_byte(0x55);
		pressure <<= 8;
		// read ls byte
		pressure += write_byte(0x55);
		pressure <<= 8;
		// read ls byte
		pressure += write_byte(0x55);

		// set spi2 cs pin
    	spi2cs_out_GPIO_Port->BSRR = spi2cs_out_Pin ;
		
		//send start conversion D2 OSR 1024 command
		// reset spi2 cs pin
    	spi2cs_out_GPIO_Port->BRR = spi2cs_out_Pin ;
		// transmit command  
		write_byte(0x54);
		// set spi2 cs pin
    	spi2cs_out_GPIO_Port->BSRR = spi2cs_out_Pin ;

		// pause 3 mS
		HAL_Delay(3);
		

		//send read adc command
		// reset spi2 cs pin
    	spi2cs_out_GPIO_Port->BRR = spi2cs_out_Pin ;
		// transmit command 
		write_byte(0x00);

		// read ms byte
		write_byte(0x55);
		temperature = 0;
		// read ls byte
		temperature += write_byte(0x55);
		temperature <<= 8;
		// read ls byte
		temperature += write_byte(0x55);
		temperature <<= 8;
		// read ls byte
		temperature += write_byte(0x55);

		// set spi2 cs pin
    	spi2cs_out_GPIO_Port->BSRR = spi2cs_out_Pin ;

		dT = temperature - sensor_prom[5]*((int32_t)0x100);
		actual_temperature = 2000 + dT*sensor_prom[6]/((uint32_t)0x800000);

		OFF = sensor_prom[2]*((int32_t)0x40000) + sensor_prom[4]*dT/((uint32_t)0x20);
		SENS = sensor_prom[1]*((int32_t)0x20000) + sensor_prom[3]*dT/((uint32_t)0x80);
		P = (pressure*SENS/((int32_t)0x200000) - OFF)/((int32_t)0x8000);


		sprintf(message, "press = %d;   temp = %d;\r\n", P, actual_temperature);
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);

		// pause 1 S
		HAL_Delay(1000);
		
	}
//*/
	

/*
  	while (1)
  	{
	  	HAL_Delay(1500);
	  	HAL_GPIO_TogglePin(led_out_GPIO_Port, led_out_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led_out_GPIO_Port, led_out_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led_out_GPIO_Port, led_out_Pin); //
		sprintf(message, "led off\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
	  	HAL_Delay(500);
	  	HAL_GPIO_TogglePin(led_out_GPIO_Port, led_out_Pin); //
		sprintf(message, "led on\r\n");
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
  	}
//*/

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
	while(1)
	{
	
  
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
