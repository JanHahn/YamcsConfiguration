/**
  ******************************************************************************
  * @file    mcp.h
  * @brief   This driver is tailored for specific use case and can be only operated in comparator mode
  *          with tCrirical temperature alert and Active Low output. Can be developed in the future to cover 
  *          all of the sensor features.
  * @todo    Consider implementing observer pattern instead of proxy
  ******************************************************************************
  */

#ifndef MCP9808_H
#define MCP9808_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"

// MCP9808 register addresses
#define CONFIG_REG        0x01
#define tUPPER_REG        0x02
#define tLOWER_REG        0x03
#define tCRITICAL_REG     0x04
#define tAMBIENT_REG      0x05
#define MANUFACTURER_REG  0x06
#define DEVICE_REG        0x07
#define RESOLUTION_REG    0x08
#define N_REGS            0x08

// MCP9808 base I2C address
#define MCP9808_BASE_ADDR 0x18 

// MCP9808 hysterisis values
#define MCP9808_HYST_0   0x00
#define MCP9808_HYST_1_5 (1 << 9)
#define MCP9808_HYST_3   (1 << 10)
#define MCP9808_HYST_6   (1 << 9 | 1 << 10)

// MCP9808 resolution values
#define MCP9808_RESOLUTION_0_5 0x00
#define MCP9808_RESOLUTION_0_25 0x01
#define MCP9808_RESOLUTION_0_125 0x02
#define MCP9808_RESOLUTION_0_0625 0x03

// MCP9808 sign values
#define MCP9808_tSIGN_POSITIVE 0x00
#define MCP9808_tSIGN_NEGATIVE (1 << 12)

typedef struct {
    uint16_t config;
    int16_t tCritical;
    float tAmbient;
    bool alertCritical;
    uint8_t resolution;
    uint8_t m_devAddr;
    I2C_HandleTypeDef *hi2c;
    osSemaphoreId_t i2cSemaphore;
} MCP9808;

void MCP9808_Init(MCP9808 *sensor, uint8_t LS_ADDR_BITS, I2C_HandleTypeDef *hi2c, osSemaphoreId_t i2cSemaphore);
void MCP9808_Enable(MCP9808 *sensor);
void MCP9808_Disable(MCP9808 *sensor);
void MCP9808_Configure(MCP9808 *sensor, uint16_t hyst, uint8_t resolution, uint8_t tCritical, uint16_t sign);
void MCP9808_Read(MCP9808 *sensor);
void MCP9808_Write(MCP9808 *sensor);
float MCP9808_GetTemperature(MCP9808 *sensor);
bool MCP9808_GetAlertCritical(MCP9808 *sensor);

#endif // MCP9808_H
