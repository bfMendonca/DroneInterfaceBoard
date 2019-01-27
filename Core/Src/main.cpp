/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6500/MPU6500.h"
#include "HMC5983/HMC5983.h"

#include "USARTInterface.h"

#include "mpu6500.pb.h"
#include "hmc5983.pb.h"
#include "JoystickReadings.pb.h"
#include "motorsinterface.pb.h"

#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Sensors::MPU6500 *m_mpu6500 = nullptr;
Sensors::HMC5983 *m_hmc5983 = nullptr;
Comm::USARTInterface<100, 100> *m_interface = nullptr;

uint8_t mpuPrescalerSender = 0x00;
uint8_t magPrescalerSender = 0x00;
uint8_t joystickPrescalerSender = 0x00;


uint8_t gyroBuffer[120];
pb_ostream_t mpuStream = pb_ostream_from_buffer(gyroBuffer, 120);
MPU6500Readings mpu6500message = MPU6500Readings_init_zero;
Sensors::ThreeAxisReadingsStamped<float> accel;
Sensors::ThreeAxisReadingsStamped<float> gyro;
Sensors::SingleValueReadingStamped< float > mpuTemperature;

uint8_t magBuffer[120];
pb_ostream_t magStream = pb_ostream_from_buffer(magBuffer, 120);
HMC5983Readings hmc5983message = HMC5983Readings_init_zero;
Sensors::ThreeAxisReadingsStamped<float> mag;
Sensors::SingleValueReadingStamped< float > magTemperature;


uint8_t motorInputBuffer[50];
pb_istream_t motorInterface = pb_istream_from_buffer( motorInputBuffer, 50 );
MotorsInterfaceMessage motorsInterfaceMessage = MotorsInterfaceMessage_init_zero;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(HCM5883L_CS_GPIO_Port, HCM5883L_CS_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET );

  HAL_Delay( 10 );

  m_mpu6500 = new Sensors::MPU6500( hspi1,  MPU6500_CS_GPIO_Port, MPU6500_CS_Pin );
  m_hmc5983 = new Sensors::HMC5983( hspi1, HCM5883L_CS_GPIO_Port, HCM5883L_CS_Pin );

  m_interface = new Comm::USARTInterface<100, 100>( &huart2 );

  m_mpu6500->startReading();
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4 );

  HAL_TIM_Base_Start( &htim1 );
  HAL_TIM_Base_Start_IT( &htim14 );

  while(true) {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void usartReceiveTransferDMACallBack( DMA_HandleTypeDef *hdma) {
	__IO uint8_t teste = 0x00;

	++teste;


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if( htim == &htim14 ) {

		if( m_interface == nullptr ) {
			return;
		}

		++mpuPrescalerSender;
		++magPrescalerSender;
		++joystickPrescalerSender;

		if( mpuPrescalerSender == 11 ) {
			size_t message_length = 0x00;

			mpuPrescalerSender = 0x00;

			accel = m_mpu6500->getAccel();
			gyro = m_mpu6500->getGyro();
			mpuTemperature = m_mpu6500->getTemp();

			mpu6500message.accelerometer.x = accel.x();
			mpu6500message.accelerometer.y = accel.y();
			mpu6500message.accelerometer.z = accel.z();
			mpu6500message.gyroscope.x = gyro.x();
			mpu6500message.gyroscope.y = gyro.y();
			mpu6500message.gyroscope.z = gyro.z();
			mpu6500message.temperature = mpuTemperature.value();
			mpu6500message.timestamp = gyro.timeStamp();

			message_length = 0;
			mpuStream = pb_ostream_from_buffer(gyroBuffer+4, 120);
			pb_encode(&mpuStream, MPU6500Readings_fields, &mpu6500message);
			message_length = mpuStream.bytes_written;

			gyroBuffer[ 0 ] = 0xAA;
			gyroBuffer[ 1 ] = 0xBB;
			gyroBuffer[ 2 ] = 0xCC;
			gyroBuffer[ 3 ] = 0x01;	//Mensagem de mpu
			message_length += 4;

			gyroBuffer[ message_length++ ] = '*';
			gyroBuffer[ message_length++ ] = 'F';
			gyroBuffer[ message_length++ ] = 'I';
			gyroBuffer[ message_length++ ] = 'M';
			gyroBuffer[ message_length++ ] = '\0';

			m_interface->appendDataToSend( gyroBuffer, message_length );
		}

		if( magPrescalerSender == 50 ) {
			size_t message_length = 0x00;
			magPrescalerSender = 0x00;

			mag = m_hmc5983->getMag();
			magTemperature = m_hmc5983->getTemp();

			hmc5983message.magnetometer.x = mag.x();
			hmc5983message.magnetometer.y = mag.y();
			hmc5983message.magnetometer.z = mag.z();
			hmc5983message.timestamp = mag.timeStamp();
			hmc5983message.temperature = magTemperature.value();

			message_length = 0;
			magStream = pb_ostream_from_buffer(magBuffer+4, 120);
			pb_encode(&magStream, HMC5983Readings_fields, &hmc5983message );
			message_length = magStream.bytes_written;

			magBuffer[ 0 ] = 0xAA;
			magBuffer[ 1 ] = 0xBB;
			magBuffer[ 2 ] = 0xCC;
			magBuffer[ 3 ] = 0x02;
			message_length += 4;

			magBuffer[ message_length++ ] = '*';
			magBuffer[ message_length++ ] = 'F';
			magBuffer[ message_length++ ] = 'I';
			magBuffer[ message_length++ ] = 'M';
			magBuffer[ message_length++ ] = '\0';

			m_interface->appendDataToSend( magBuffer, message_length );
		}

		if( joystickPrescalerSender == 220 ) {

		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if( GPIO_Pin == MPU6500_INT_Pin ) {
		if( m_mpu6500 != nullptr ) {

			hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
			if (HAL_SPI_Init(&hspi1) != HAL_OK) {
				Error_Handler();
			}

			m_mpu6500->intCallback();
		}
	}else if( GPIO_Pin == HMC5883L_INT_Pin ) {
		if( m_hmc5983 != nullptr ) {

			hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
			if (HAL_SPI_Init(&hspi1) != HAL_OK) {
				Error_Handler();
			}

			m_hmc5983->intCallback();
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if( m_interface != nullptr ) {
		m_interface->txCallback();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t *received = huart->pRxBuffPtr;

	bool status = false;

	if( ( received[0] == 0xAA ) && ( received[1] == 0xBB ) && ( received[2] == 0xCC ) ) {
		if( received[3] == 0x99 ) {
			motorInterface = pb_istream_from_buffer( received+4, 20 );
			status = pb_decode( &motorInterface, MotorsInterfaceMessage_fields, &motorsInterfaceMessage );
		}
	}

	if( status ) {
		++teste;
		__IO uint16_t m1, m2, m3, m4;

		m1 = motorsInterfaceMessage.motor1DC;
		m2 = motorsInterfaceMessage.motor2Dc;
		m3 = motorsInterfaceMessage.motor3DC;
		m4 = motorsInterfaceMessage.motor4DC;
	}else {
		++teste;
	}

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
