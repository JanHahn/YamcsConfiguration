#include "mcp.h"

/**
* @brief Function initializing all MCP9808's fields with deafult values.
* @param sensor: MCP9808 sensor pointer
* @param LS_ADDR_BITS: Least significant address bits
* @param hi2c: I2C handler
* @param I2C_SemaphoreHandle: I2C semaphore handler
* @retval None
*/
void MCP9808_Init(MCP9808 *sensor, uint8_t LS_ADDR_BITS, I2C_HandleTypeDef *hi2c, osSemaphoreId_t I2C_SemaphoreHandle) {
    sensor->config |= (1 << 2) + (1 << 8);                              //tCritical alert only
    sensor->tCritical = 0x0000;
    sensor->tAmbient = 0.0;
    sensor->alertCritical = false;
    sensor->resolution = 0x03;                                          //0.0625 Celsius degrees
    sensor->m_devAddr = MCP9808_BASE_ADDR | (LS_ADDR_BITS & 0x07);      
    sensor->hi2c = hi2c;
    sensor->i2cSemaphore = I2C_SemaphoreHandle;
}

/**
 * @brief Function enabling MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @retval None
 */
void MCP9808_Enable(MCP9808 *sensor) {
    sensor->config = (sensor->config & 0x06FF); //clearing SHDN bit
    MCP9808_Write(sensor);
}

/**
 * @brief Function disabling MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @retval None
 */
void MCP9808_Disable(MCP9808 *sensor) {
    sensor->config = (sensor->config | (1 << 8)) & 0x7FFF; //setting SHDN bit
    MCP9808_Write(sensor);
}

/**
 * @brief Function configuring MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @param hyst: Hysteresis value
 * @param resolution: Resolution value
 * @param tCritical: Critical temperature value in range of -256 to 256 Celsius degrees
 * @param sign: Sign of the critical temperature value
 * @retval None
 */
void MCP9808_Configure(MCP9808 *sensor, uint16_t hyst, uint8_t resolution, uint8_t tCritical, uint16_t sign) {
    sensor->config = (sensor->config & 0x06FF) | hyst; 
    sensor->resolution = resolution;
    sensor->tCritical = (((uint16_t)tCritical << 4) | sign);  //losing precision here for the sake of using Celsius degrees in the API
    MCP9808_Write(sensor);
}

/**
 * @brief Function for getting temperature value from MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @retval Temperature value
 */
float MCP9808_GetTemperature(MCP9808 *sensor) {
    MCP9808_Read(sensor);
    return sensor->tAmbient;
}

/**
 * @brief Function for getting critical temperature alert flag from MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @retval Critical temperature alert flag
 */
bool MCP9808_GetAlertCritical(MCP9808 *sensor) {
    MCP9808_Read(sensor);
    return sensor->alertCritical;
}

/**
 * @brief Private function for reading from MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @retval None
 */
void MCP9808_Read(MCP9808 *sensor) {
    uint8_t data[2];
    HAL_StatusTypeDef status;

    osSemaphoreAcquire(sensor->i2cSemaphore, portMAX_DELAY);
    status = HAL_I2C_Mem_Read_DMA(sensor->hi2c, (sensor->m_devAddr << 1), tAMBIENT_REG, I2C_MEMADD_SIZE_8BIT, data, 2);
    if (status != HAL_OK) {
        printf("Error reading from MCP9808 at address: 0x%x\n", sensor->m_devAddr);
    }
    osSemaphoreAcquire(sensor->i2cSemaphore, portMAX_DELAY); //A bit ugly, but prevents race condition in saving data to sensor fields
    osSemaphoreRelease(sensor->i2cSemaphore);

    if (data[0] & (1 << 7)) {
        sensor->alertCritical = true;
    } else {
        sensor->alertCritical = false;
    }
    
    data[0] = data[0] & 0x1F;
    if(data[0] & (1 << 4)) {
        data[0] = data[0] & 0x0F;
        sensor->tAmbient = 256 - ((float)data[0] * 16) - ((float)data[1] / 16);
    } else {
        sensor->tAmbient = ((float)data[0] * 16) + ((float)data[1] / 16);
    }
}

/**
 * @brief Private function for writing to MCP9808 sensor
 * @param sensor: MCP9808 sensor pointer
 * @retval None
 */
void MCP9808_Write(MCP9808 *sensor) {
    uint8_t data[2];
    HAL_StatusTypeDef status;

    osSemaphoreAcquire(sensor->i2cSemaphore, portMAX_DELAY);
    data[0] = sensor->config >> 8;
    data[1] = sensor->config & 0xFF;
    status = HAL_I2C_Mem_Write_DMA(sensor->hi2c, sensor->m_devAddr << 1, CONFIG_REG, I2C_MEMADD_SIZE_8BIT, data, 2);
    if (status != HAL_OK) {
        printf("Error writing to MCP9808 at address: 0x%x\n", sensor->m_devAddr);
    };

    osSemaphoreAcquire(sensor->i2cSemaphore, portMAX_DELAY);
    data[0] = (sensor->tCritical >> 8) & 0x1F;
    data[1] = sensor->tCritical & 0xFC;
    status = HAL_I2C_Mem_Write_DMA(sensor->hi2c, sensor->m_devAddr << 1, tCRITICAL_REG, I2C_MEMADD_SIZE_8BIT, data, 2);
    if (status != HAL_OK) {
        printf("Error writing to MCP9808 at address: 0x%x\n", sensor->m_devAddr);
    }

    osSemaphoreAcquire(sensor->i2cSemaphore, portMAX_DELAY);
    data[0] = sensor->resolution & 0x03;
    status = HAL_I2C_Mem_Write_DMA(sensor->hi2c, sensor->m_devAddr << 1, RESOLUTION_REG, I2C_MEMADD_SIZE_8BIT, data, 1);
    if (status != HAL_OK) {
        printf("Error writing to MCP9808 at address: 0x%x\n", sensor->m_devAddr);
    }
}
