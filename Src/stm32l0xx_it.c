/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "extern.h"
#include "user_define.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
byte adc_cnt = 0, T10s_cnt = 0;
byte thermister_0_adc_cnt = 0, thermister_1_adc_cnt = 0, thermister_slope_0_adc_cnt = 0, thermister_slope_1_adc_cnt = 0;
byte Thermistor_0_data_cnt = 0, Thermistor_1_data_cnt = 0, thermistor_0_slope_cnt = 0, thermistor_1_slope_cnt = 0;
byte THERMISTOR_0_Slope = 0, THERMISTOR_1_Slope = 0;
byte THERMISTOR_0_Slope_Buf[50], THERMISTOR_1_Slope_Buf[50];
byte Tx_buffer[7] = {0xC9,0x04,0x02,0x00,0x01,0x78,0xF0};
byte thermister_vol_0_cnt = 0, thermister_vol_1_cnt = 0;
byte thermister_vol = 0, thermister_vol_0 = 0, thermister_vol_1 = 0, thermister_vol_1_start_cnt = 0;
byte Fire_Fg = 0, Past_Fire_Fg = 0; // Fire_Fg가 1이면 화재 발생, Past_Fire_Fg가 0이면 화재 발생 조건문에 들어옴 but, 1이면 화재 발생 조건문에 들어오지 않음( 중복 방지 )
byte Rx1_buf[10] = {0};
byte CheckSum_ing = 0;
byte Rx1_step = 0,Rx1_index = 0;
byte Rx1_CRC_H = 0, Rx1_CRC_L = 0, CheckSum_EN = 0;
byte rx1_data = 0;
byte Equip_Key_c1 = 0, Equip_Key_c2 = 0, Equip_Key_c3 = 0, Equip_Key_c4 = 0, Equip_Key_c5 = 0, 
Equip_Key_c6 = 0, Equip_Key_c7 = 0, Equip_Key_c8 = 0, Equipment_Num = 0, Schematic_Num = 0;
byte Tx_Fg_Num = 0; // Tx_Fg_Num = 회로*20-20 + Equipment_Num, 수신기로부터 데이터가 오는데 5회로면 81~100값이 온다. 수신기에서 받은 데이터가 이와 같으면 화재 신호를 수신기로 보낸다 
byte Fire_Fg_0 = 0, Fire_Fg_1 = 0;

unsigned short THERMISTOR_0_AD[50],THERMISTOR_1_AD[50];
unsigned short THERMISTOR_0_Vol = 0, THERMISTOR_1_Vol = 0, THERMISTOR_Vol = 0;
unsigned short Past_THERMISTOR_0_Vol = 0, Current_THERMISTOR_0_Vol = 0, Past_THERMISTOR_1_Vol = 0, Current_THERMISTOR_1_Vol = 0, THERMISTOR_0_Slope_Vol = 0, THERMISTOR_1_Slope_Vol = 0;
word CheckSum_data = 0;
unsigned short THERMISTOR_Vol_0[50], THERMISTOR_Vol_1[50];

unsigned long int CAL_THERMISTOR_0_AD = 0,  CAL_THERMISTOR_1_AD = 0, CAL_THERMISTOR_Slope_0_AD = 0, CAL_THERMISTOR_Slope_1_AD = 0;
unsigned long int ADC_Value_0 = 0, ADC_Value_1 = 0;
unsigned int Rcv1_cnt = 0, Tx1_send_number = 0,Tx1_index = 0, Error1_cnt = 0;

uint16_t Read_Pin(GPIO_TypeDef* port, uint16_t pin, uint8_t shift) {
    return (!(port->IDR & pin) ? 1 : 0) << shift;
}

void Equipment_Num_Define(void)
{
    Equipment_Num = 0;
	Schematic_Num = 0;

    Schematic_Num |= Read_Pin(GPIOA, GPIO_PIN_3, 0);  // Schematic_Key_c0
	Schematic_Num |= Read_Pin(GPIOA, GPIO_PIN_2, 1);  // Schematic_Key_c1
	Schematic_Num |= Read_Pin(GPIOC, GPIO_PIN_15, 2); // Schematic_Key_c2
	Schematic_Num |= Read_Pin(GPIOC, GPIO_PIN_14, 3); // Schematic_Key_c3
    
    Equipment_Num |= Read_Pin(GPIOA, GPIO_PIN_13, 0);  // Equip_Key_c0
	Equipment_Num |= Read_Pin(GPIOA, GPIO_PIN_9, 1);  // Equip_Key_c1
	Equipment_Num |= Read_Pin(GPIOB, GPIO_PIN_1, 2);  // Equip_Key_c2
	Equipment_Num |= Read_Pin(GPIOA, GPIO_PIN_7, 3);  // Equip_Key_c3
	Equipment_Num |= Read_Pin(GPIOA, GPIO_PIN_6, 4);  // Equip_Key_c4
	
	Tx_Fg_Num = ((Schematic_Num * 20) - 20) + Equipment_Num;
	
	TX_ON;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern LPTIM_HandleTypeDef hlptim1;
extern UART_HandleTypeDef hlpuart1;
/* USER CODE BEGIN EV */
void P_Type_Reciever_Tx(byte p_reciever__tx_num);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
	  static byte again_cnt = 0;
	  again_cnt++;
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles LPTIM1 global interrupt / LPTIM1 wake-up interrupt through EXTI line 29.
  */
void LPTIM1_IRQHandler(void) // 500ms TIMER
{
        /* 인터럽트 발생 여부 확인 */
    if (__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_ISR_ARRM))
    {
        /* 인터럽트 플래그 클리어 (인터럽트 발생 후 클리어) */
		__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_ISR_ARRM);
	  
		static unsigned int timer_test_cnt;
		timer_test_cnt++;
		
		if (ADC1->ISR & ADC_ISR_EOC)  // 변환 완료 확인 후 데이터 읽기
		{
			if(adc_cnt == 0)
			{
				CAL_THERMISTOR_0_AD = 0; // 써미스터 0 AD 더한값을 초기화함
				ADC_Value_0 = ADC1->DR;
				THERMISTOR_0_AD[thermister_0_adc_cnt] = ADC_Value_0;
			
				if(thermister_0_adc_cnt >= 2) // thermister_0_adc_cnt이 2이상일경우 이동 평균 값
				{
					if(THERMISTOR_0_AD[49] != 0)
					{
						for(byte i = thermister_0_adc_cnt - 2; i <= thermister_0_adc_cnt; i++)
						{
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[i];
						}
						THERMISTOR_0_Vol = (CAL_THERMISTOR_0_AD * 1100) >> 12; // 3개 평균낸 ADC값
					}
					else // 초기 0번째 인덱스 값은 더미 데이터이므로 계산하지 않음
					{
						if(thermister_0_adc_cnt == 2)
						{
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[1];
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[2];
							THERMISTOR_0_Vol = (CAL_THERMISTOR_0_AD * 1650) >> 12;  // 배열 인덱스 1,2 평균 ADC값
						}
						else
						{
							for(byte i = thermister_0_adc_cnt - 2; i <= thermister_0_adc_cnt; i++)
							{
								CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[i];
							}
							THERMISTOR_0_Vol = (CAL_THERMISTOR_0_AD * 1100) >> 12; // 3개 평균낸 ADC값
						}
					}
				} 
				else if(thermister_0_adc_cnt < 2) // thermister_0_adc_cnt이 2이하일경우 이동 평균 값
				{
					if(thermister_0_adc_cnt == 0)
					{
						if(THERMISTOR_0_AD[49] != 0)
						{
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[48];
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[49];
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[0];
							THERMISTOR_0_Vol = (CAL_THERMISTOR_0_AD * 1100) >> 12;  // 3개 평균낸 ADC값
						}
					}
					else if(thermister_0_adc_cnt == 1)
					{
						if(THERMISTOR_0_AD[49] != 0)
						{
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[49];
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[0];
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[1];
							THERMISTOR_0_Vol = (CAL_THERMISTOR_0_AD * 1100) >> 12;  // 3개 평균낸 ADC값
						}
						else
						{
							CAL_THERMISTOR_0_AD += THERMISTOR_0_AD[1];
							THERMISTOR_0_Vol = (CAL_THERMISTOR_0_AD * 3300) >> 12;  // 1개  ADC값
						}
					}
				}
				
				if(++thermister_0_adc_cnt >= 50)
				{
					thermister_0_adc_cnt = 0;
				}

				if(Thermistor_0_data_cnt == 0)
				{
					Past_THERMISTOR_0_Vol = THERMISTOR_0_Vol;
					if(Current_THERMISTOR_0_Vol != 0)
					{
						if(Current_THERMISTOR_0_Vol > Past_THERMISTOR_0_Vol)
						{
							THERMISTOR_0_Slope = (Current_THERMISTOR_0_Vol - Past_THERMISTOR_0_Vol);
						}
						else
						{
							THERMISTOR_0_Slope = 0;
						}
					}
					Thermistor_0_data_cnt = 1;
				}
				else
				{
					Current_THERMISTOR_0_Vol = THERMISTOR_0_Vol;
					if(Past_THERMISTOR_0_Vol > Current_THERMISTOR_0_Vol)
					{
						THERMISTOR_0_Slope = (Past_THERMISTOR_0_Vol - Current_THERMISTOR_0_Vol);
					}
					else
					{
						THERMISTOR_0_Slope = 0;
					}
					Thermistor_0_data_cnt = 0;
				}	
				
				CAL_THERMISTOR_Slope_0_AD = 0;
				
				THERMISTOR_0_Slope_Buf[thermister_slope_0_adc_cnt] = THERMISTOR_0_Slope;
					
				if(thermister_slope_0_adc_cnt >= 2) // thermister_slope_0_adc_cnt이 2이상일경우 이동 평균 값
				{
					if(THERMISTOR_0_Slope_Buf[49] != 0)
					{
						for(byte i = thermister_slope_0_adc_cnt - 2; i <= thermister_slope_0_adc_cnt; i++)
						{
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[i];
						}
						THERMISTOR_0_Slope_Vol = (CAL_THERMISTOR_Slope_0_AD / 3); // 3개 평균낸 ADC값
					}
					else // 초기 0번째 인덱스 값은 더미 데이터이므로 계산하지 않음
					{
						if(thermister_slope_0_adc_cnt == 2)
						{
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[1];
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[2];
							THERMISTOR_0_Slope_Vol = (CAL_THERMISTOR_Slope_0_AD / 2);  // 배열 인덱스 1,2 평균 ADC값
						}
						else
						{
							for(byte i = thermister_slope_0_adc_cnt - 2; i <= thermister_slope_0_adc_cnt; i++)
							{
								CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[i];
							}
							THERMISTOR_0_Slope_Vol = (CAL_THERMISTOR_Slope_0_AD / 3); // 3개 평균낸 ADC값
						}
					}
				} 
				else if(thermister_slope_0_adc_cnt < 2) // thermister_0_adc_cnt이 2이하일경우 이동 평균 값
				{
					if(thermister_slope_0_adc_cnt == 0)
					{
						if(THERMISTOR_0_Slope_Buf[49] != 0)
						{
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[48];
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[49];
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[0];
							THERMISTOR_0_Slope_Vol = (CAL_THERMISTOR_Slope_0_AD / 3);  // 3개 평균낸 ADC값
						}
					}
					else if(thermister_slope_0_adc_cnt == 1)
					{
						if(THERMISTOR_0_Slope_Buf[49] != 0)
						{
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[49];
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[0];
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[1];
							THERMISTOR_0_Slope_Vol = (CAL_THERMISTOR_Slope_0_AD / 3) ;  // 3개 평균낸 ADC값
						}
						else
						{
							CAL_THERMISTOR_Slope_0_AD += THERMISTOR_0_Slope_Buf[1];
							THERMISTOR_0_Slope_Vol = CAL_THERMISTOR_Slope_0_AD;  // 1개  ADC값
						}
					}
				}
				
				/*if(thermister_vol)
				{
					THERMISTOR_Vol_0[thermister_vol_0++] = THERMISTOR_0_Vol;
					
					//
				}*/
				
				if(++thermister_slope_0_adc_cnt >= 50)
				{
					thermister_slope_0_adc_cnt = 0;
				}
				
				adc_cnt = 1;

				// 4번 채널 활성화, 5번 채널 비활성화
				ADC1->CHSELR = ADC_CHANNEL_4;
			}
			else if(adc_cnt == 1)
			{
				CAL_THERMISTOR_1_AD = 0; // 써미스터 1 AD 더한값을 초기화함
				ADC_Value_1 = ADC1->DR;
				THERMISTOR_1_AD[thermister_1_adc_cnt] = ADC_Value_1;
				
				if(thermister_1_adc_cnt >= 2) // thermister_1_adc_cnt이 2이상일경우 이동 평균 값
				{
					if(THERMISTOR_1_AD[49] != 0)
					{
						for(byte i = thermister_1_adc_cnt - 2; i <= thermister_1_adc_cnt; i++)
						{
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[i];
						}
						THERMISTOR_1_Vol = (CAL_THERMISTOR_1_AD * 1100) >> 12; // 3개 평균낸 ADC값
					}
					else // 초기 0번째 인덱스 값은 더미 데이터이므로 계산하지 않음
					{
						if(thermister_1_adc_cnt == 2)
						{
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[1];
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[2];
							THERMISTOR_1_Vol = (CAL_THERMISTOR_1_AD * 1650) >> 12;  // 배열 인덱스 1,2 평균 ADC값
						}
						else
						{
							for(byte i = thermister_1_adc_cnt - 2; i <= thermister_1_adc_cnt; i++)
							{
								CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[i];
							}
							THERMISTOR_1_Vol = (CAL_THERMISTOR_1_AD * 1100) >> 12; // 3개 평균낸 ADC값
						}
					}
				}
				else if(thermister_1_adc_cnt < 2) // thermister_1_adc_cnt이 2이하일경우 이동 평균 값
				{
					if(thermister_1_adc_cnt == 0)
					{
						if(THERMISTOR_1_AD[49] != 0)
						{
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[48];
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[49];
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[0];
							THERMISTOR_1_Vol = (CAL_THERMISTOR_1_AD * 1100) >> 12;  // 3개 평균낸 ADC값
						}
					}
					else if(thermister_1_adc_cnt == 1)
					{
						if(THERMISTOR_1_AD[49] != 0)
						{
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[49];
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[0];
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[1];
							THERMISTOR_1_Vol = (CAL_THERMISTOR_1_AD * 1100) >> 12;  // 3개 평균낸 ADC값
						}
						else
						{
							CAL_THERMISTOR_1_AD += THERMISTOR_1_AD[1];
							THERMISTOR_1_Vol = (CAL_THERMISTOR_1_AD * 3300) >> 12;  // 1개  ADC값
						}
					}
				}
				
				if(++thermister_1_adc_cnt >= 50)
				{
					thermister_1_adc_cnt = 0;
				}
				
				if(Thermistor_1_data_cnt == 0)
				{
					Past_THERMISTOR_1_Vol = THERMISTOR_1_Vol;
					if(Current_THERMISTOR_1_Vol != 0)
					{
						if(Current_THERMISTOR_1_Vol > Past_THERMISTOR_1_Vol)
						{
							THERMISTOR_1_Slope = (Current_THERMISTOR_1_Vol - Past_THERMISTOR_1_Vol);
						}
						else
						{
							THERMISTOR_1_Slope = 0;
						}
					}
					Thermistor_1_data_cnt = 1;
				}
				else
				{
					Current_THERMISTOR_1_Vol = THERMISTOR_1_Vol;
					if(Past_THERMISTOR_1_Vol > Current_THERMISTOR_1_Vol)
					{
						THERMISTOR_1_Slope = (Past_THERMISTOR_1_Vol - Current_THERMISTOR_1_Vol);
					}
					else
					{
						THERMISTOR_1_Slope = 0;
					}
					Thermistor_1_data_cnt = 0;
				}	
				
				CAL_THERMISTOR_Slope_1_AD = 0;
				
				THERMISTOR_1_Slope_Buf[thermister_slope_1_adc_cnt] = THERMISTOR_1_Slope;
					
				if(thermister_slope_1_adc_cnt >= 2) // thermister_slope_1_adc_cnt이 2이상일경우 이동 평균 값
				{
					if(THERMISTOR_1_Slope_Buf[49] != 0)
					{
						for(byte i = thermister_slope_1_adc_cnt - 2; i <= thermister_slope_1_adc_cnt; i++)
						{
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[i];
						}
						THERMISTOR_1_Slope_Vol = (CAL_THERMISTOR_Slope_1_AD / 3); // 3개 평균낸 ADC값
					}
					else // 초기 0번째 인덱스 값은 더미 데이터이므로 계산하지 않음
					{
						if(thermister_slope_1_adc_cnt == 2)
						{
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[1];
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[2];
							THERMISTOR_1_Slope_Vol = (CAL_THERMISTOR_Slope_1_AD / 2);  // 배열 인덱스 1,2 평균 ADC값
						}
						else
						{
							for(byte i = thermister_slope_1_adc_cnt - 2; i <= thermister_slope_1_adc_cnt; i++)
							{
								CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[i];
							}
							THERMISTOR_1_Slope_Vol = (CAL_THERMISTOR_Slope_1_AD / 3); // 3개 평균낸 ADC값
						}
					}
				} 
				else if(thermister_slope_1_adc_cnt < 2) // thermister_0_adc_cnt이 2이하일경우 이동 평균 값
				{
					if(thermister_slope_1_adc_cnt == 0)
					{
						if(THERMISTOR_1_Slope_Buf[49] != 0)
						{
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[48];
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[49];
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[0];
							THERMISTOR_1_Slope_Vol = (CAL_THERMISTOR_Slope_1_AD / 3);  // 3개 평균낸 ADC값
						}
					}
					else if(thermister_slope_1_adc_cnt == 1)
					{
						if(THERMISTOR_1_Slope_Buf[49] != 0)
						{
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[49];
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[0];
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[1];
							THERMISTOR_1_Slope_Vol = (CAL_THERMISTOR_Slope_1_AD / 3) ;  // 3개 평균낸 ADC값
						}
						else
						{
							CAL_THERMISTOR_Slope_1_AD += THERMISTOR_1_Slope_Buf[1];
							THERMISTOR_1_Slope_Vol = CAL_THERMISTOR_Slope_1_AD;  // 1개  ADC값
						}
					}
				}
				
				/*if(thermister_vol)
				{
					THERMISTOR_Vol_1[thermister_vol_1++] = THERMISTOR_1_Vol;

					//
				}*/
				
				if(++thermister_slope_1_adc_cnt >= 50)
				{
					thermister_slope_1_adc_cnt = 0;
				}

				adc_cnt = 0;

				// 5번 채널 활성화, 4번 채널 비활성화
				ADC1->CHSELR = ADC_CHANNEL_5;
			}
			
			/*if(thermister_vol_0 >= 50 && thermister_vol_1 >= 50)
			{
				thermister_vol = 0;
			}
			
			static byte slope_zero_x_cnt = 0;
			if(THERMISTOR_0_Slope != 0 || THERMISTOR_1_Slope != 0)
			{
				slope_zero_x_cnt++;
			}*/
			
			if(!Fire_Fg)
			{
				if(THERMISTOR_0_Vol > 2000) // 43도 보다 온도가 낮을 때는 기울기가 30 이상 차이가 난다면  Fire_Fg++ 
				{
					if(THERMISTOR_0_Slope_Vol >= 30)
					{
						Fire_Fg_0++;
					}
					else
					{
						if(Fire_Fg_0)
						{
							Fire_Fg_0--;
						}
					}
				}
				else if(THERMISTOR_0_Vol <= 2000 && THERMISTOR_0_Vol >= 1500) // 43도 보다 온도가 높고 57도보다 온도가 낮을때는 기울기가 20 이상 차이가 난다면  Fire_Fg++ 
				{
					if(THERMISTOR_0_Slope_Vol >= 20)
					{
						Fire_Fg_0++;
					}
					else
					{
						if(Fire_Fg_0)
						{
							Fire_Fg_0--;
						}
					}
				}
				else if(THERMISTOR_0_Vol < 1500) // 57도 보다 온도가 높을때는 기울기가 10 이상 차이가 난다면  Fire_Fg++ 
				{
					if(THERMISTOR_0_Slope_Vol >= 10)
					{
						Fire_Fg_0++;
					}
					else
					{
						if(Fire_Fg_0)
						{
							Fire_Fg_0--;
						}
					}
				}
				
				if(THERMISTOR_1_Vol > 2000) // 43도 보다 온도가 낮을 때는 기울기가 30 이상 차이가 난다면  Fire_Fg++ 
				{
					if(THERMISTOR_1_Slope_Vol >= 15)
					{
						Fire_Fg_1++;
					}
					else
					{
						if(Fire_Fg_1)
						{
							Fire_Fg_1--;
						}
					}
				}
				else if(THERMISTOR_1_Vol <= 2000 && THERMISTOR_1_Vol >= 1500) // 43도 보다 온도가 높고 57도보다 온도가 낮을때는 기울기가 20 이상 차이가 난다면  Fire_Fg++ 
				{
					if(THERMISTOR_1_Slope_Vol >= 10)
					{
						Fire_Fg_1++;
					}
					else
					{
						if(Fire_Fg_1)
						{
							Fire_Fg_1--;
						}
					}
				}
				else if(THERMISTOR_1_Vol < 1500) // 57도 보다 온도가 높을때는 기울기가 10 이상 차이가 난다면  Fire_Fg++ 
				{
					if(THERMISTOR_1_Slope_Vol >= 5)
					{
						Fire_Fg_1++;
					}
					else
					{
						if(Fire_Fg_1)
						{
							Fire_Fg_1--;
						}
					}
				}
			}
			
			if(THERMISTOR_0_Vol && THERMISTOR_1_Vol)
			{
				if(Fire_Fg_0 >= 5 || Fire_Fg_1 >= 5 || (THERMISTOR_0_Vol <= 1100 || THERMISTOR_1_Vol <= 1100)) //Fire_Fg_0, Fire_Fg_1의 둘중 하나가 5 이상이거나 온도가 70도 이상이면 화재(정온식 1종)
				{
					Fire_Fg = 1;
				}
			}
			
			
//#if 0
			if(Fire_Fg == 1 && Past_Fire_Fg == 0) // 화재가 나면 초기화 될때까지 여기를 다시 들어오지 않음
			{			
				Fire_Fg = 0;
				
				// DIP 입력 핀 구성 시작
				GPIO_InitTypeDef GPIO_InitStruct = {0};
				
				// 디버깅 기능 끊은 후 SWD 핀을 일반 GPIO로 사용
				__HAL_RCC_DBGMCU_CLK_DISABLE();  // 디버그 블록 클럭 차단
				GPIO_InitStruct.Pin = GPIO_PIN_13;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				
				/*Configure GPIO pins : Equip_Num_1_Pin Equip_Num_2_Pin */
				GPIO_InitStruct.Pin = Equip_Num_1_Pin|Equip_Num_2_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

				/*Configure GPIO pins : Equip_Num_3_Pin Equip_Num_4_Pin Equip_Num_5_Pin Equip_Num_6_Pin
										   Equip_Num_8_Pin */
				GPIO_InitStruct.Pin = Equip_Num_3_Pin|Equip_Num_4_Pin|Equip_Num_5_Pin|Equip_Num_6_Pin
										  |Equip_Num_8_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

				/*Configure GPIO pin : Equip_Num_7_Pin */
				GPIO_InitStruct.Pin = Equip_Num_7_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				HAL_GPIO_Init(Equip_Num_7_GPIO_Port, &GPIO_InitStruct);
				
				Equipment_Num_Define();
				Past_Fire_Fg = 1; // 과거 화재 발생 변수 1 ( 1이면 이 조건문에 들어오지 않음. 중복 방지 )
			}
//#endif	
		}

		// ADC 변환 시작
		ADC1->CR |= ADC_CR_ADSTART;
	}
}

/**
  * @brief This function handles LPUART1 global interrupt / LPUART1 wake-up interrupt through EXTI line 28.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */
	if((LPUART1->CR1 & USART_CR1_RXNEIE) && (LPUART1-> ISR & USART_ISR_RXNE))	// 수신버퍼가 채워졌다면?	
	{		
		Rcv1_cnt++;
		rx1_data = LPUART1->RDR;	// 수신값 저장
		
		if(CheckSum_ing == 0)
		{
			switch(Rx1_step) 
			{
				case 0:
					if(rx1_data > 0 && rx1_data == Tx_Fg_Num) // 감지기 번호가 동일한가?
					{
						Rx1_buf[0] = rx1_data;	//Rx_Buf[0] = 국번(0x01)
						Rx1_step = 1;
						Rx1_index = 1;
					}
					else
					{
						Rx1_step = 0;
						Rx1_index = 0;
					}
					break;
				case 1: //함수 
					if(rx1_data == 0x04) //function 0x04: Read Request
					{															 	   					
						Rx1_buf[Rx1_index++] = rx1_data;
						Rx1_step++;
					}
					else                                                    
					{
						Rx1_step = 0;     //Waiting for  국번
						Rx1_index = 0;
					}
					break;
				case 2: // 읽기 시작 주소(High)
					Rx1_buf[Rx1_index++] = rx1_data;
					Rx1_step++;
					break;
				
				case 3: // 읽기 시작 주소(Low)
					Rx1_buf[Rx1_index++] = rx1_data;
					Rx1_step++;
					break;	
				
				case 4: // 읽어올 레지스터의 수(High)
					Rx1_buf[Rx1_index++] = rx1_data;
					Rx1_step++;
					break;
					
				case 5: // 읽어올 레지스터의 수(Low)
					Rx1_buf[Rx1_index++] = rx1_data;
					Rx1_step++;
					break;
					
				case 6: //Waiting for CRC16 (Low byte)	
					Rx1_buf[Rx1_index++] = rx1_data;
					Rx1_step++;
					break;

				case 7: //Waiting for CRC16 (High byte)	
					Rx1_buf[Rx1_index] = rx1_data;	
					CheckSum_ing = 1;
					Rx1_CRC_L = Rx1_buf[Rx1_index-1];
					Rx1_CRC_H = Rx1_buf[Rx1_index];
					
					Fire_Detector_Data_crc = CRC16(Rx1_buf, 6);//수신한 데이터의 CheckSum계산	
					P_Type_Reciever_Data_crc = (Rx1_CRC_H<<8) | Rx1_CRC_L;	
					if(P_Type_Reciever_Data_crc == Fire_Detector_Data_crc)//CheckSum OK!
					{
						Tx_buffer[0] = 0xC9;
						Tx_buffer[1] = 0x04;
						Tx_buffer[2] = 0x02;
						Tx_buffer[3] = Tx_Fg_Num;
						Tx_buffer[4] = 0x01;

						CheckSum_data = CRC16(Tx_buffer, 5);	//6개 데이터의 CheckSum계산 
						Tx_buffer[5] = CheckSum_data & 0xFF; //CRC's Low byte	
						Tx_buffer[6] = CheckSum_data >>8; //CRC's High byte	
						
						P_Type_Reciever_Tx(7);
						
					}
					Rx1_step = 0;
					Rx1_index = 0;
					break;

					
				default:
					Rx1_step = 0;
					Rx1_index = 0;
					break;
			}
		}
	}
	else if((LPUART1->CR1 & USART_CR1_TCIE) && (LPUART1->ISR & USART_ISR_TC))
	{
		static byte Tx3_cnt=0;
		Tx3_cnt++;
		if(Tx1_send_number)
		{
			Tx1_send_number--;  
			LPUART1->TDR = Tx_buffer[Tx1_index++];
		}
		else	//모든 Data 전송 하였음.
		{
			//LPUART1->CR1 |= USART_CR1_RXNEIE; 	// UART2's RXE Interrupt Enable		
			LPUART1->CR1 &= ~USART_CR1_TCIE;		// UART2's TXE Interrupt Disable
		}
		return;	
	}
	
  /* USER CODE END LPUART1_IRQn 0 */
  //HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */

  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void P_Type_Reciever_Tx(byte p_reciever__tx_num)	//UART1
{
	/*Send the character*/
	LPUART1->TDR = Tx_buffer[0];
	Tx1_index = 1;
	Tx1_send_number = p_reciever__tx_num-1;
	//LPUART1->CR1 &= ~USART_CR1_RXNEIE; 	// UART1's RXE Interrupt Disable	
	LPUART1->CR1 |= USART_CR1_TCIE; 		//USART1's TXE Interrupt Enable	
}

/* USER CODE END 1 */
