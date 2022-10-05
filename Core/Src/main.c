/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "lwip.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "lwip/udp.h"
//#include "app_ethernet.h"
#include "lwip/tcpip.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
#define APP_ADDRESS (0x08040000)

#define APP_START_ADDRESS (0x08100000)
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct netif gnetif;
/* USER CODE END PM */
typedef void (application_t)(void);

typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

/* Private variables ---------------------------------------------------------*/
void jumpToApp(const uint32_t address);
void Flash_init();
void deinitEverything();
void write_flash_memory();
static void Netif_Config(void);
void udp_echoclient_connect(void);
void udp_echoclient_send(void);
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
struct udp_pcb *upcb;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buffer=0;
uint32_t write_address = APP_START_ADDRESS;
/* USER CODE END 0 */

void jumpToApp(const uint32_t address)
{
	const JumpStruct* vector_p = (JumpStruct*)address;

	deinitEverything();

	/* let's do The Jump! */
    /* Jump, used asm to avoid stack optimization */
    asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
    //asm("nop");
}

void deinitEverything()
{
	//-- reset peripherals to guarantee flawless start of user application
	HAL_GPIO_DeInit(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin);
	//HAL_GPIO_DeInit(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin);
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//HAL_GPIO_DeInit(USB_ENABLE_GPIO_Port, USB_ENABLE_Pin);
	//USBD_DeInit(&hUsbDeviceFS);
	//__disable_irq();
	  __HAL_RCC_GPIOC_CLK_DISABLE();
	  __HAL_RCC_GPIOD_CLK_DISABLE();
	  __HAL_RCC_GPIOB_CLK_DISABLE();
	  __HAL_RCC_GPIOA_CLK_DISABLE();
	//MX_LWIP_Init();
	//netif_set_down(&gnetif);
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
uint32_t flash_read(uint32_t address)
{

   return *(uint32_t*)address;

}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_flash_memory()
{
	//Flash_init(APP_START_ADDRESS);
	uint32_t write_address = APP_START_ADDRESS;
	uint32_t read_address = APP_ADDRESS;
	uint32_t data=0;
	for(int i=0;i<8000;i++)
	{
		data=flash_read(read_address);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,write_address, data) == HAL_OK)
		{
			write_address += 1;
			read_address +=1;
		}
	}

	HAL_FLASH_Lock();
}

void Flash_init()
{
	uint32_t status=0;
	static FLASH_EraseInitTypeDef EraseInitStruct;

	  /* Unlock the Flash to enable the flash control register access *************/
	while(1)
	{
	   HAL_FLASH_Unlock();
	   uint32_t PAGEError;
	   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
	   /* Erase the user Flash area*/

	  //uint32_t StartPage = GetPage(StartPageAddress);
	  //uint32_t EndPageAdress = StartPageAddress + 204*4;
	  //uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/

	   EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	   EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	   EraseInitStruct.Sector=FLASH_SECTOR_12;
	   //EraseInitStruct.PageAddress = StartPage;
	   EraseInitStruct.NbSectors=3;

	   status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	   if(status == HAL_OK)
	   {
		   break;
	   }
	}
	   /* Program the user Flash area word by word*/

}

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
  //MX_LWIP_Init();
  Netif_Config();

  //udpServer_init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  udp_echoclient_connect();

  //HAL_FLASH_Unlock();
  //udp_echoclient_send();
  //HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
  //HAL_Delay(1000);
  Flash_init();
  //write_flash_memory();
  //HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
  //HAL_Delay(1000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //MX_LWIP_Init();

		  if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
		  {
			  HAL_FLASH_Lock();
			  jumpToApp(APP_START_ADDRESS);
		  }
		  else
		  {
			    ethernetif_input(&gnetif);
			    sys_check_timeouts();
		  }
	    //udp_recv(upcb, udp_receive_callback, NULL);
	    //HAL_Delay(1000);
	    //udp_echoclient_send();

	    /* Handle timeouts */

  }
  /* USER CODE END 3 */
}

void udp_echoclient_connect(void)
{
  ip_addr_t DestIPaddr;
  ip_addr_t SourceIPAddr;
  err_t err;

  upcb = udp_new();

  if (upcb!=NULL)
  {

    IP4_ADDR( &DestIPaddr, 192, 168, 0, 20);
    IP4_ADDR( &SourceIPAddr, 192, 168, 0, 10);

    //udp_disconnect(upcb, &DestIPaddr, 5000);
    err = udp_connect(upcb, &DestIPaddr, 5000);

    err = udp_bind(upcb, &SourceIPAddr, 5000);

    if (err == ERR_OK)
    {

      udp_recv(upcb, udp_receive_callback, NULL);
    }
  }
}

void udp_echoclient_send(void)
{
  struct pbuf *p;
  char data[1]="A";
  //strcat(data,buffer);
  //sprintf((char*)data, "sending udp client message %d", (int)message_count);
  p = pbuf_alloc(PBUF_TRANSPORT,sizeof(data), PBUF_POOL);
  if (p != NULL)
  {
    pbuf_take(p, (const void *)&data, sizeof(data));
    udp_send(upcb, p);
    pbuf_free(p);
  }
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
 // message_count++;
  //uint8_t *temp;
  //temp=;
  buffer = *(uint8_t*)(p->payload);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,write_address, buffer);
  write_address += 1;
  //HAL_GPIO_TogglePin(GPIOB,LD2_Pin);
  //udp_echoclient_send();
  //HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
  pbuf_free(p);
}


static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

//#ifdef USE_DHCP
//  ip_addr_set_zero_ip4(&ipaddr);
//  ip_addr_set_zero_ip4(&netmask);
//  ip_addr_set_zero_ip4(&gw);
//#else
  lwip_init();
  IP_ADDR4(&ipaddr,192,168,0,10);
  IP_ADDR4(&netmask,255,255,255,0);
  IP_ADDR4(&gw,192,168,0,1);
//#endif /* USE_DHCP */

  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /*  Registers the default network interface. */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

