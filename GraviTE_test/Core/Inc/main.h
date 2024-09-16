/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define ValveGate13_Pin GPIO_PIN_2
#define ValveGate13_GPIO_Port GPIOE
#define ValveGate14_Pin GPIO_PIN_3
#define ValveGate14_GPIO_Port GPIOE
#define ValveGate16_Pin GPIO_PIN_4
#define ValveGate16_GPIO_Port GPIOE
#define ValveGate16E5_Pin GPIO_PIN_5
#define ValveGate16E5_GPIO_Port GPIOE
#define DRV_I2C_SDA_Pin GPIO_PIN_0
#define DRV_I2C_SDA_GPIO_Port GPIOF
#define DRV_I2C_SCL_Pin GPIO_PIN_1
#define DRV_I2C_SCL_GPIO_Port GPIOF
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define ADC_12V_Pin GPIO_PIN_2
#define ADC_12V_GPIO_Port GPIOC
#define ADC_5V_Pin GPIO_PIN_3
#define ADC_5V_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define BOOT1__CHECK_THIS__Pin GPIO_PIN_2
#define BOOT1__CHECK_THIS__GPIO_Port GPIOB
#define PHY_EN_CLK_Pin GPIO_PIN_7
#define PHY_EN_CLK_GPIO_Port GPIOE
#define PHY_SIGDET_Pin GPIO_PIN_8
#define PHY_SIGDET_GPIO_Port GPIOE
#define PHY_INT_Pin GPIO_PIN_9
#define PHY_INT_GPIO_Port GPIOE
#define PHY_REF_CLK_Pin GPIO_PIN_10
#define PHY_REF_CLK_GPIO_Port GPIOE
#define PHY_RESET_Pin GPIO_PIN_11
#define PHY_RESET_GPIO_Port GPIOE
#define ValveGate1_Pin GPIO_PIN_12
#define ValveGate1_GPIO_Port GPIOE
#define ValveGate2_Pin GPIO_PIN_13
#define ValveGate2_GPIO_Port GPIOE
#define ValveGate3_Pin GPIO_PIN_14
#define ValveGate3_GPIO_Port GPIOE
#define ValveGate4_Pin GPIO_PIN_15
#define ValveGate4_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define ValveGate5_Pin GPIO_PIN_14
#define ValveGate5_GPIO_Port GPIOB
#define ValveGate6_Pin GPIO_PIN_15
#define ValveGate6_GPIO_Port GPIOB
#define ValveGate7_Pin GPIO_PIN_10
#define ValveGate7_GPIO_Port GPIOD
#define ValveGate8_Pin GPIO_PIN_11
#define ValveGate8_GPIO_Port GPIOD
#define VALVE_PWR_EN1_Pin GPIO_PIN_12
#define VALVE_PWR_EN1_GPIO_Port GPIOD
#define HEATER_PWM1_Pin GPIO_PIN_15
#define HEATER_PWM1_GPIO_Port GPIOD
#define PHY_RESET_CHECK_THIS___Pin GPIO_PIN_6
#define PHY_RESET_CHECK_THIS___GPIO_Port GPIOC
#define HEATER_PWM2_Pin GPIO_PIN_8
#define HEATER_PWM2_GPIO_Port GPIOC
#define HEAT_I2C_SDA_1_Pin GPIO_PIN_9
#define HEAT_I2C_SDA_1_GPIO_Port GPIOC
#define HEAT_I2C_SCL_1_Pin GPIO_PIN_8
#define HEAT_I2C_SCL_1_GPIO_Port GPIOA
#define EN_PUMP1_Pin GPIO_PIN_9
#define EN_PUMP1_GPIO_Port GPIOA
#define EN_PUMP2_Pin GPIO_PIN_10
#define EN_PUMP2_GPIO_Port GPIOA
#define EN_PUMP_PWR_Pin GPIO_PIN_11
#define EN_PUMP_PWR_GPIO_Port GPIOA
#define mem__reset_Pin GPIO_PIN_15
#define mem__reset_GPIO_Port GPIOA
#define mem_sck_Pin GPIO_PIN_10
#define mem_sck_GPIO_Port GPIOC
#define mem_miso_Pin GPIO_PIN_11
#define mem_miso_GPIO_Port GPIOC
#define mem_mosi_Pin GPIO_PIN_12
#define mem_mosi_GPIO_Port GPIOC
#define mem__wp_Pin GPIO_PIN_0
#define mem__wp_GPIO_Port GPIOD
#define mem__ce_Pin GPIO_PIN_1
#define mem__ce_GPIO_Port GPIOD
#define VALVE_PWR_EN2_Pin GPIO_PIN_15
#define VALVE_PWR_EN2_GPIO_Port GPIOG
#define ValveGate9_Pin GPIO_PIN_4
#define ValveGate9_GPIO_Port GPIOB
#define ValveGate10_Pin GPIO_PIN_5
#define ValveGate10_GPIO_Port GPIOB
#define ST_LINK_TX_Pin GPIO_PIN_6
#define ST_LINK_TX_GPIO_Port GPIOB
#define ST_LINK_RX_Pin GPIO_PIN_7
#define ST_LINK_RX_GPIO_Port GPIOB
#define HEAT_I2C_SDA2_Pin GPIO_PIN_8
#define HEAT_I2C_SDA2_GPIO_Port GPIOB
#define HEAT_I2C_SDA2B9_Pin GPIO_PIN_9
#define HEAT_I2C_SDA2B9_GPIO_Port GPIOB
#define ValveGate11_Pin GPIO_PIN_0
#define ValveGate11_GPIO_Port GPIOE
#define ValveGate12_Pin GPIO_PIN_1
#define ValveGate12_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
