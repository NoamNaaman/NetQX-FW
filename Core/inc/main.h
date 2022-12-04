/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#define DIP2_Pin GPIO_PIN_2
#define DIP2_GPIO_Port GPIOE
#define DIP3_Pin GPIO_PIN_3
#define DIP3_GPIO_Port GPIOE
#define DIP4_Pin GPIO_PIN_4
#define DIP4_GPIO_Port GPIOE
#define DIP5_Pin GPIO_PIN_5
#define DIP5_GPIO_Port GPIOE
#define DIP6_Pin GPIO_PIN_6
#define DIP6_GPIO_Port GPIOE
#define DIP7_Pin GPIO_PIN_13
#define DIP7_GPIO_Port GPIOC
#define DIP8_Pin GPIO_PIN_9
#define DIP8_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define R2D0_Pin GPIO_PIN_2
#define R2D0_GPIO_Port GPIOF
#define R2D0_EXTI_IRQn EXTI2_3_IRQn
#define R4D0_Pin GPIO_PIN_7
#define R4D0_GPIO_Port GPIOE
#define R4D0_EXTI_IRQn EXTI4_15_IRQn
#define D1DPOS_Pin GPIO_PIN_9
#define D1DPOS_GPIO_Port GPIOE
#define D1RTE_Pin GPIO_PIN_4
#define D1RTE_GPIO_Port GPIOA
#define D2RTE_Pin GPIO_PIN_10
#define D2RTE_GPIO_Port GPIOE
#define D3DPOS_Pin GPIO_PIN_11
#define D3DPOS_GPIO_Port GPIOE
#define D2DPOS_Pin GPIO_PIN_12
#define D2DPOS_GPIO_Port GPIOE
#define R3RTE_Pin GPIO_PIN_13
#define R3RTE_GPIO_Port GPIOE
#define R3D0_Pin GPIO_PIN_14
#define R3D0_GPIO_Port GPIOE
#define R3D0_EXTI_IRQn EXTI4_15_IRQn
#define R1D0_Pin GPIO_PIN_15
#define R1D0_GPIO_Port GPIOE
#define R1D0_EXTI_IRQn EXTI4_15_IRQn
#define R1D1_Pin GPIO_PIN_10
#define R1D1_GPIO_Port GPIOB
#define R1D1_EXTI_IRQn EXTI4_15_IRQn
#define R3D1_Pin GPIO_PIN_13
#define R3D1_GPIO_Port GPIOB
#define R3D1_EXTI_IRQn EXTI4_15_IRQn
#define D4RTE_Pin GPIO_PIN_14
#define D4RTE_GPIO_Port GPIOB
#define RBUZ1_Pin GPIO_PIN_8
#define RBUZ1_GPIO_Port GPIOD
#define RELAY10_Pin GPIO_PIN_9
#define RELAY10_GPIO_Port GPIOD
#define RELAY9_Pin GPIO_PIN_10
#define RELAY9_GPIO_Port GPIOD
#define SRAM_CS_Pin GPIO_PIN_11
#define SRAM_CS_GPIO_Port GPIOD
#define MEM_PRESENT_Pin GPIO_PIN_12
#define MEM_PRESENT_GPIO_Port GPIOD
#define RBUZ2_Pin GPIO_PIN_13
#define RBUZ2_GPIO_Port GPIOD
#define RBUZ3_Pin GPIO_PIN_14
#define RBUZ3_GPIO_Port GPIOD
#define FLASH_CS_Pin GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOD
#define ETH_CS_Pin GPIO_PIN_6
#define ETH_CS_GPIO_Port GPIOC
#define SF_RST_Pin GPIO_PIN_7
#define SF_RST_GPIO_Port GPIOC
#define SF_WP_Pin GPIO_PIN_8
#define SF_WP_GPIO_Port GPIOC
#define R4D1_Pin GPIO_PIN_9
#define R4D1_GPIO_Port GPIOC
#define R4D1_EXTI_IRQn EXTI4_15_IRQn
#define R2D1_Pin GPIO_PIN_8
#define R2D1_GPIO_Port GPIOA
#define R2D1_EXTI_IRQn EXTI4_15_IRQn
#define RS485EN_Pin GPIO_PIN_11
#define RS485EN_GPIO_Port GPIOA
#define D4DPOS_Pin GPIO_PIN_12
#define D4DPOS_GPIO_Port GPIOA
#define RBUZ4_Pin GPIO_PIN_6
#define RBUZ4_GPIO_Port GPIOF
#define RLED4_Pin GPIO_PIN_15
#define RLED4_GPIO_Port GPIOA
#define RLED3_Pin GPIO_PIN_6
#define RLED3_GPIO_Port GPIOB
#define RLED2_Pin GPIO_PIN_7
#define RLED2_GPIO_Port GPIOB
#define RLED1_Pin GPIO_PIN_12
#define RLED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOD
#define SDA_Pin GPIO_PIN_1
#define SDA_GPIO_Port GPIOD
#define SCL_Pin GPIO_PIN_2
#define SCL_GPIO_Port GPIOD
#define ETH_RST_Pin GPIO_PIN_3
#define ETH_RST_GPIO_Port GPIOD
#define RELAY5_Pin GPIO_PIN_5
#define RELAY5_GPIO_Port GPIOD
#define RELAY4_Pin GPIO_PIN_6
#define RELAY4_GPIO_Port GPIOD
#define RELAY3_Pin GPIO_PIN_7
#define RELAY3_GPIO_Port GPIOD
#define RELAY1_Pin GPIO_PIN_4
#define RELAY1_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_5
#define RELAY2_GPIO_Port GPIOB
#define RELAY6_Pin GPIO_PIN_8
#define RELAY6_GPIO_Port GPIOB
#define RELAY7_Pin GPIO_PIN_9
#define RELAY7_GPIO_Port GPIOB
#define RELAY8_Pin GPIO_PIN_0
#define RELAY8_GPIO_Port GPIOE
#define DIP1_Pin GPIO_PIN_1
#define DIP1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
