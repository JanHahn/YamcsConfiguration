/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "mcp.h"
#include <stdbool.h>
#include "i2c.h"
#include <stdio.h>
#include "usart.h"
#include "dma.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

#define TM_PORT 10015 //TM - sending
#define TC_PORT 10025 //TC- receiving
#define TM_PACKET_SIZE 40

extern DMA_HandleTypeDef hdma_usart3_rx;

typedef struct {

	uint8_t incubationStage;
	float heater1Temp;
	float heater2Temp;
	float valveDriver1Current;
	float valveDriver2Current;
	uint8_t valve1_8;
	uint8_t valve9_16;
	bool pump1;
	bool pump2;
	float uptime;
	uint8_t resetCounter;
	float  supplyVoltage12;
	float supplyVoltage5;

} __attribute__((packed)) TELEMETRY_DATA;



typedef struct {
    int tm_counter;
    int tc_counter;
    uint8_t last_tc[TM_PACKET_SIZE];
    osThreadId tm_thread;
    osThreadId tc_thread;
} YAMCS;





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
TELEMETRY_DATA rx_tele;
bool udp_yamcs_enabled;
YAMCS yamcs = {0};
uint8_t rx_uart_buffer[15];
MCP9808 tempSensor;

/* USER CODE END Variables */
/* Definitions for sendYAMCS */
osThreadId_t sendYAMCSHandle;
const osThreadAttr_t sendYAMCS_attributes = {
  .name = "sendYAMCS",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for receiveYAMCS */
osThreadId_t receiveYAMCSHandle;
const osThreadAttr_t receiveYAMCS_attributes = {
  .name = "receiveYAMCS",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tempControl */
osThreadId_t tempControlHandle;
const osThreadAttr_t tempControl_attributes = {
  .name = "tempControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for telemetrySafety */
osThreadId_t telemetrySafetyHandle;
const osThreadAttr_t telemetrySafety_attributes = {
  .name = "telemetrySafety",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for comm_control */
osThreadId_t comm_controlHandle;
const osThreadAttr_t comm_control_attributes = {
  .name = "comm_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempQueue */
osMessageQueueId_t TempQueueHandle;
const osMessageQueueAttr_t TempQueue_attributes = {
  .name = "TempQueue"
};
/* Definitions for CommToActQueue */
osMessageQueueId_t CommToActQueueHandle;
const osMessageQueueAttr_t CommToActQueue_attributes = {
  .name = "CommToActQueue"
};
/* Definitions for TelemetryToCommQueue */
osMessageQueueId_t TelemetryToCommQueueHandle;
const osMessageQueueAttr_t TelemetryToCommQueue_attributes = {
  .name = "TelemetryToCommQueue"
};
/* Definitions for RxUartBuff */
osMessageQueueId_t RxUartBuffHandle;
const osMessageQueueAttr_t RxUartBuff_attributes = {
  .name = "RxUartBuff"
};
/* Definitions for CommToTempQueue */
osMessageQueueId_t CommToTempQueueHandle;
const osMessageQueueAttr_t CommToTempQueue_attributes = {
  .name = "CommToTempQueue"
};
/* Definitions for I2C1_Semaphore */
osSemaphoreId_t I2C1_SemaphoreHandle;
osStaticSemaphoreDef_t I2C1_SemaphoreControlBlock;
const osSemaphoreAttr_t I2C1_Semaphore_attributes = {
  .name = "I2C1_Semaphore",
  .cb_mem = &I2C1_SemaphoreControlBlock,
  .cb_size = sizeof(I2C1_SemaphoreControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int actCommandConversion(char command[4], uint8_t *result);
int tempCommandConversion(char temp[4],float *result);
/* USER CODE END FunctionPrototypes */

void SendYAMCS(void *argument);
void ReceiveYAMCS(void *argument);
void TempControl(void *argument);
void TelemetrySafetyControl(void *argument);
void CommControl(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of I2C1_Semaphore */
  I2C1_SemaphoreHandle = osSemaphoreNew(1, 1, &I2C1_Semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of TempQueue */
  TempQueueHandle = osMessageQueueNew (16, sizeof(bool), &TempQueue_attributes);

  /* creation of CommToActQueue */
  CommToActQueueHandle = osMessageQueueNew (4, sizeof(uint8_t), &CommToActQueue_attributes);

  /* creation of TelemetryToCommQueue */
  TelemetryToCommQueueHandle = osMessageQueueNew (2, sizeof(TELEMETRY_DATA), &TelemetryToCommQueue_attributes);

  /* creation of RxUartBuff */
  RxUartBuffHandle = osMessageQueueNew (3, 5, &RxUartBuff_attributes);

  /* creation of CommToTempQueue */
  CommToTempQueueHandle = osMessageQueueNew (2, sizeof(float), &CommToTempQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sendYAMCS */
  sendYAMCSHandle = osThreadNew(SendYAMCS, NULL, &sendYAMCS_attributes);

  /* creation of receiveYAMCS */
  receiveYAMCSHandle = osThreadNew(ReceiveYAMCS, NULL, &receiveYAMCS_attributes);

  /* creation of tempControl */
  tempControlHandle = osThreadNew(TempControl, NULL, &tempControl_attributes);

  /* creation of telemetrySafety */
  telemetrySafetyHandle = osThreadNew(TelemetrySafetyControl, NULL, &telemetrySafety_attributes);

  /* creation of comm_control */
  comm_controlHandle = osThreadNew(CommControl, NULL, &comm_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_SendYAMCS */

/**
  * @brief  Function implementing the sendYAMCS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SendYAMCS */
void SendYAMCS(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN SendYAMCS */

	int sock;
	struct sockaddr_in dest_addr;

	//tworzenie socketu
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printf("Failed to create socket\n");
		return;
	}

	//konfiguracja adresu do ktorego wysylamy
	memset(&dest_addr, 0, sizeof(dest_addr));
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(TM_PORT);
	dest_addr.sin_addr.s_addr = inet_addr("192.168.8.101");

  /* Infinite loop */
  for(;;)
  {
	  	if (udp_yamcs_enabled == true){  //

	  		uint8_t packet[TM_PACKET_SIZE];

	  		//example CCSDS header (only for tests)
	  		uint8_t header[6] = {0x39, 0xA5, 0xC0, 0x00, 0x00, 0x0A};

	  		memcpy(packet, header, 6);
	  		memcpy(packet + 6, &rx_tele, sizeof(TELEMETRY_DATA));

	  		if (sendto(sock, packet, sizeof(packet), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
	  			printf("Failed to send packet\n");
	  		}
	  		udp_yamcs_enabled = false;
	  	}
	    osDelay(500);
  }
  /* USER CODE END SendYAMCS */
}

/* USER CODE BEGIN Header_ReceiveYAMCS */
/**
* @brief Function implementing the receiveYAMCS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveYAMCS */
void ReceiveYAMCS(void *argument)
{
  /* USER CODE BEGIN ReceiveYAMCS */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END ReceiveYAMCS */
}

/* USER CODE BEGIN Header_TempControl */
/**
* @brief Function implementing the tempControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TempControl */
void TempControl(void *argument)
{
  /* USER CODE BEGIN TempControl */
  MCP9808_Init(&tempSensor, 0x00, &hi2c1, I2C1_SemaphoreHandle);
  MCP9808_Configure(&tempSensor, MCP9808_HYST_1_5, MCP9808_RESOLUTION_0_0625, 40, MCP9808_tSIGN_POSITIVE);
  MCP9808_Enable(&tempSensor);
  /* Infinite loop */
  for(;;)
  {
    MCP9808_GetTemperature(&tempSensor);
    printf("Test temperature: %.2f\n", tempSensor.tAmbient);
    osDelay(1000);
  }
  /* USER CODE END TempControl */
}

/* USER CODE BEGIN Header_TelemetrySafetyControl */
/**
* @brief Function implementing the telemetrySafety thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TelemetrySafetyControl */
void TelemetrySafetyControl(void *argument)
{
  /* USER CODE BEGIN TelemetrySafetyControl */
  /* Infinite loop */
  TELEMETRY_DATA dataToSend;
  for(;;)
  {
    //zbieranie danych telemetrycznych (na razie wpisane na sztywno)
	  dataToSend.heater1Temp = 12.12;
	  dataToSend.heater2Temp = 30.8;
	  dataToSend.incubationStage = 2;
	  dataToSend.pump1 = 1;
	  dataToSend.pump2 = 0;
	  dataToSend.resetCounter = 1;
	  dataToSend.supplyVoltage12 = 11.98;
	  dataToSend.supplyVoltage5 = 4.95;
	  dataToSend.valve1_8 = 0b01110110;
	  dataToSend.valve9_16 = 0b10100110;
	  dataToSend.valveDriver1Current = 2.345;
	  dataToSend.valveDriver2Current = 3.234;
	  dataToSend.uptime = 40.88;

	  //wysylanie danych bufora
	  osMessageQueuePut(TelemetryToCommQueueHandle, &dataToSend, 0, 0);

    osDelay(4000);
  }
  /* USER CODE END TelemetrySafetyControl */
}

/* USER CODE BEGIN Header_CommControl */

/**
* @brief Function implementing the comm_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommControl */
void CommControl(void *argument)
{
  /* USER CODE BEGIN CommControl */

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)rx_uart_buffer, sizeof(rx_uart_buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

  char rx_buff[5];
  uint8_t act_data;
  float temp_data;

  //buffor do wysyłania telemtrii przez UART
  char tx_buff[100];

  /* Infinite loop */
  for(;;)
  {
     //odbiór danych UART - sprawdzamy kolejke czy przyszly jakies dane
	  osStatus_t queue_status_1 = osMessageQueueGet(RxUartBuffHandle, &rx_buff, NULL, 0);
	  if (queue_status_1 == osOK){
		  char rx_command_type = rx_buff[0];
		  char actual_data[4];
		  memcpy(actual_data, rx_buff + 1, 4);

		  switch (rx_command_type){

		  	  //wysyłamy dane do act_control
		  	  case '1':
		  		  if (actCommandConversion(rx_buff, &act_data) == 1){
		  			  osMessageQueuePut(CommToActQueueHandle, &act_data, 0, 0);
		  		  }
		  		  break;

		  	  //wysyłamy dane do temp_control
		  	  case '2':
		  		  tempCommandConversion(rx_buff, &temp_data);
		  		  osMessageQueuePut(CommToTempQueueHandle, &temp_data, 0, 0);
		  		  break;
		  }
	  }



	//odbieramy dane telemetryczne
	osStatus_t queue_status_2 = osMessageQueueGet(TelemetryToCommQueueHandle, &rx_tele, NULL, 0);
	if (queue_status_2 == osOK){
		udp_yamcs_enabled = true;
		//obsługa danych z kolekji oraz zamiana na odpowiedni format
		int len = sprintf(tx_buff, "incub: %d\nheat1: %.2f\nheat2: %.2f\npump1: %d\npump2: %d\n", rx_tele.incubationStage, // @suppress("Float formatting support")
				rx_tele.heater1Temp, rx_tele.heater2Temp, (uint8_t)rx_tele.pump1, (uint8_t)rx_tele.pump2);

		//WYSYŁAMY DANE PRZEZ UART
		if (HAL_UART_Transmit(&huart3, (uint8_t*)tx_buff, len, HAL_MAX_DELAY) != HAL_OK)
		    {
		        printf("Uart transmit error");
		    }

	}
	osDelay(100);

  }
  /* USER CODE END CommControl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if (huart->Instance == USART3){

		if (Size % 5 == 0){
			uint8_t command[5];
			for (int i = 0; i < Size; i++){
				command[i%5] = rx_uart_buffer[i];
				if (i%5 == 4){
					osMessageQueuePut(RxUartBuffHandle, command ,0,0);
				}
			}
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)rx_uart_buffer, sizeof(rx_uart_buffer));
		  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

int actCommandConversion(char command[4], uint8_t *result){
	*result = 0b00000000;
	switch (command[0])
	{
		case 'v':
			break;
		case 'p':
			*result |= (0b00000001 << 6);
			break;
		case 's':
			*result |= (0b00000001 << 7);
			break;
		case 'f':
			*result |= (0b00000011 << 6);
			break;
		default:
			printf("command conversion error");
			return 0;
	}

	switch (command[1])
	{
		case '0':
			break;
		case '1':
			*result |= (0b00000001 << 5);
			break;
		default:
			printf("command conversion error");
			return 0;
	}

	uint8_t addres = (command[2]-'0')*10 + command[3]-'0';
	if (addres > 31){
		printf("command conversion error");
		return 0;

	}
	else{
		*result |= addres;
		return 1;
	}
}

int tempCommandConversion(char temp[4],float *result) {

    *result = 10*(temp[0]-'0');
    *result += temp[1]-'0';
    *result += 0.1*(temp[2]-'0');
    *result += 0.01*(temp[3]-'0');
    return 1;
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    osSemaphoreRelease(I2C1_SemaphoreHandle);
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    osSemaphoreRelease(I2C1_SemaphoreHandle);
  }
}
/* USER CODE END Application */

