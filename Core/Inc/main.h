/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define JOYSTICK_CHANNEL_1_Pin GPIO_PIN_0
#define JOYSTICK_CHANNEL_1_GPIO_Port GPIOA
#define JOYSTICK_CHANNEL_2_Pin GPIO_PIN_1
#define JOYSTICK_CHANNEL_2_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define MPU6500_CS_Pin GPIO_PIN_0
#define MPU6500_CS_GPIO_Port GPIOB
#define HCM5883L_CS_Pin GPIO_PIN_1
#define HCM5883L_CS_GPIO_Port GPIOB
#define M1_PWM_OUTPUT_Pin GPIO_PIN_8
#define M1_PWM_OUTPUT_GPIO_Port GPIOA
#define M2_PWM_OUTPUT_Pin GPIO_PIN_9
#define M2_PWM_OUTPUT_GPIO_Port GPIOA
#define M3_PWM_OUTPUT_Pin GPIO_PIN_10
#define M3_PWM_OUTPUT_GPIO_Port GPIOA
#define M4_PWM_OUTPUT_Pin GPIO_PIN_11
#define M4_PWM_OUTPUT_GPIO_Port GPIOA
#define MPU6500_INT_Pin GPIO_PIN_12
#define MPU6500_INT_GPIO_Port GPIOA
#define MPU6500_INT_EXTI_IRQn EXTI4_15_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define BMP280_CS_Pin GPIO_PIN_3
#define BMP280_CS_GPIO_Port GPIOB
#define JOYSTICK_CHANNEL_4_Pin GPIO_PIN_4
#define JOYSTICK_CHANNEL_4_GPIO_Port GPIOB
#define JOYSTICK_CHANNEL_3_Pin GPIO_PIN_5
#define JOYSTICK_CHANNEL_3_GPIO_Port GPIOB
#define HMC5883L_INT_Pin GPIO_PIN_6
#define HMC5883L_INT_GPIO_Port GPIOB
#define HMC5883L_INT_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
