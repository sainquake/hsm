/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

CAN_FilterConfTypeDef  sFilterConfig;

uint32_t a =0;
uint32_t b =0;
uint32_t c=0;

typedef struct{
float pwm;
float smooth;
float COUNT_TO_MKS;
long en;
}PWM_IN_Struct;
PWM_IN_Struct pwm_in;

float time=0;
float dt=0;
float f=2;
float f_inv = 0;
float f_inv_old=0;
#define POLES 11.0
#define STATES 6.0
int phase = 0;
int count = 0;
int ch = 3;

uint32_t analog;
float fpot=0;
uint32_t t0=0;
uint32_t n=0;
float fn0=0;//initial neutral
float fn=0;//neutral
float fadc=0;//phase value
uint32_t adc = 0;
float adc0=0;
uint32_t adc2 = 0;
uint32_t sm_adc = 0;
int ncount = 0;
ADC_ChannelConfTypeDef adc1ch[6];

uint32_t t4old = 0;
typedef struct{

	uint32_t adc;
	uint32_t neutral;
	uint32_t pot;
	uint32_t u_cur;

	float fpot;
	float current0;
	float current;


}ADC_Struct;
ADC_Struct ADC;

typedef struct{

 unsigned char RW;
 unsigned char Reg_Adress;
 unsigned char Data;
 unsigned char OK;

 unsigned char WHO_AM_I;

 unsigned char OUTX_L_XL;
 unsigned char OUTX_H_XL;
 unsigned char OUTY_L_XL;
 unsigned char OUTY_H_XL;
 unsigned char OUTZ_L_XL;
 unsigned char OUTZ_H_XL;

 short ax;
 short ay;
 short az;

}SPI_Struct;

SPI_Struct ACC;

typedef struct{

 uint8_t data[200];
 int count;
 char lock;

}TX_Struct;

TX_Struct TXBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
void init(void);
void PlaySound(float f_,int delay);

//ADC
void StartAnalogADC1( int ch );
int GetAnalogADC1( void );
void StartAnalogADC2( int ch );
int GetAnalogADC2( void );
int ReadAnalogADC1( int ch );
int ReadAnalogADC2( int ch );

//output control
void ResetControlLOW(void);
void ResetControlHIGH(void);
void SetControlHIGH(GPIO_PinState ah,GPIO_PinState al,GPIO_PinState bh,GPIO_PinState bl,GPIO_PinState ch,GPIO_PinState cl);
void SetControlLOW(GPIO_PinState ah,GPIO_PinState al,GPIO_PinState bh,GPIO_PinState bl,GPIO_PinState ch,GPIO_PinState cl);

void Accelerometer(unsigned char rw, unsigned char reg, unsigned char data_);
void UpdateFrequency(void);
void ResetControl(void);
void SetControl(GPIO_PinState ah,GPIO_PinState al,GPIO_PinState bh,GPIO_PinState bl,GPIO_PinState ch,GPIO_PinState cl);

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*if (htim->Instance==TIM3) //check if the interrupt comes from TIM3
	{
		switch(phase){
						case 0:SetControlLOW(0,0,1,0,0,1);break;
						case 1:SetControlLOW(0,1,1,0,0,0);break;
						case 2:SetControlLOW(0,1,0,0,1,0);break;
						case 3:SetControlLOW(0,0,0,1,1,0);break;
						case 4:SetControlLOW(1,0,0,1,0,0);break;
						case 5:SetControlLOW(1,0,0,0,0,1);break;
		}
	}*/
	if (htim->Instance==TIM4) //check if the interrupt comes from TIM3
	{
		count=-1;
		phase++;
		if(phase>=6)
			phase=0;
		switch(phase){
									case 0:SetControlHIGH(0,0,1,0,0,1);ch=2;break;
									case 1:SetControlHIGH(0,1,1,0,0,0);ch=1;break;
									case 2:SetControlHIGH(0,1,0,0,1,0);ch=0;break;
									case 3:SetControlHIGH(0,0,0,1,1,0);ch=2;break;
									case 4:SetControlHIGH(1,0,0,1,0,0);ch=1;break;
									case 5:SetControlHIGH(1,0,0,0,0,1);ch=0;break;
		}
		StartAnalogADC2(ch);
		count=0;
	}
}
/*void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance==TIM3) //check if the interrupt comes from TIM3
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			ResetControlLOW();
		}
	}

}*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan1)
{
  if ((hcan1->pRxMsg->StdId == 0x321) && (hcan1->pRxMsg->IDE == CAN_ID_STD) && (hcan1->pRxMsg->DLC == 2))
  {
	  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	  //LED_Display(CanHandle->pRxMsg->Data[0]);
    //ubKeyNumber = CanHandle->pRxMsg->Data[0];
  }

  /* Receive */
  if (HAL_CAN_Receive_IT(hcan1, CAN_FIFO0) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */


  /*  static CanTxMsgTypeDef        TxMessage;
    static CanRxMsgTypeDef        RxMessage;

    hcan1.pTxMsg = &TxMessage;
    hcan1.pRxMsg = &RxMessage;

    //##-2- Configure the CAN Filter ###########################################
      sFilterConfig.FilterNumber = 0;
      sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
      sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
      sFilterConfig.FilterIdHigh = 0x0000;
      sFilterConfig.FilterIdLow = 0x0000;
      sFilterConfig.FilterMaskIdHigh = 0x0000;
      sFilterConfig.FilterMaskIdLow = 0x0000;
      sFilterConfig.FilterFIFOAssignment = 0;
      sFilterConfig.FilterActivation = ENABLE;
      sFilterConfig.BankNumber = 14;

      if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
      {
        Error_Handler();
      }

      //##-3- Configure Transmission process #####################################
      hcan1.pTxMsg->StdId = 0x321;
      hcan1.pTxMsg->ExtId = 0x01;
      hcan1.pTxMsg->RTR = CAN_RTR_DATA;
      hcan1.pTxMsg->IDE = CAN_ID_STD;
      hcan1.pTxMsg->DLC = 2;

      if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK)
        {
          Error_Handler();
        }*/

  init();


  f=1.1;
  f_inv =  1/(f*POLES*STATES);
  TIM4->ARR = (uint32_t)round(1280000.0*f_inv);
  //TIM3->CCR1 = 350;
  TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 600;
  HAL_Delay(100);

  pwm_in.smooth = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){}
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	 // HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	 // hcan1.pTxMsg->Data[0] = 1;
	 // hcan1.pTxMsg->Data[1] = 2;

      /*##-3- Start the Transmission process ###############################*/
	  /*if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK)
	    {
	      Error_Handler();
	    }*/

	  //HAL_CAN_Transmit(&hcan1, 50);
      //HAL_Delay(200);

	  uint32_t cnt = TIM4->CNT;
	  if(count>=0){
	 	 	count++;
	 	 	StartAnalogADC2(ch);
	 }
	  adc = GetAnalogADC2();
	  fadc = (float)adc;
	  if(count>=0 && fabs(f_inv_old - f_inv)<0.01 ){
	  	if( phase==0 || phase==2 || phase==4 )
	  		if(fadc<fn && cnt>0 ){
	  			f_inv = (f_inv*999.0+((float)(cnt))*1.4/1280000.0)/1000.0;
	  			TIM4->ARR = (uint32_t)round(1280000.0*f_inv);
	  			count=-1;
	  		}
	  	if( phase==1 || phase==3 || phase==5 )
	  		if(fadc>fn  && cnt>0 ){
	  			f_inv = (f_inv*999.0+((float)(cnt))*1.4/1280000.0)/1000.0;
	  			TIM4->ARR = (uint32_t)round(1280000.0*f_inv);
	  			count=-1;
	  		}
	  }
	  f_inv_old = f_inv;
	  //n  = ReadAnalogADC1(ch);//neutral
	  fn = ((float)HAL_ADC_GetValue(&hadc1) +fn*799.0)/800.0;


	  a = TIM2->CCR1;
	  b = TIM2->CCR2;
	  c= 0;//( TIM2->CCR2 - TIM2->CCR1 )/2;
	  if(b>a){
		  c=b-a;
	  }else{
		//  c=TIM2->ARR+b-a;
	  }
	  pwm_in.pwm = ((float)c)*pwm_in.COUNT_TO_MKS;
	  if(pwm_in.pwm<700 )
		 pwm_in.pwm=0;
	 if(pwm_in.pwm>2300)
		 pwm_in.pwm=2300;

	  if(pwm_in.pwm<700)
		  pwm_in.en=0;
	  else
		  pwm_in.en++;

	  if(pwm_in.pwm>=700)
		  pwm_in.smooth = (pwm_in.pwm + pwm_in.smooth*999.0)/1000.0;
	  if(pwm_in.en>100){
		  //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		  uint32_t c = 250 + round((pwm_in.smooth-700)/2);
		  if(c>1200)
			  c=1200;
		  TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = c;
	  }else{
		  //HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

		  /*f=1.1;
		  f_inv =  1/(f*POLES*STATES);
		  TIM4->ARR = (uint32_t)round(1280000.0*f_inv);
		  TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 250;*/
		  //HAL_Delay(100);
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 100;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_6TQ;
  hcan1.Init.BS2 = CAN_BS2_8TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AH_Pin|BH_Pin|CH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_R_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AH_Pin BH_Pin CH_Pin */
  GPIO_InitStruct.Pin = AH_Pin|BH_Pin|CH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UpdateFrequency(void){
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	  f_inv = (f_inv*39.0+((float)TIM4->CNT)*2.0/12800.0)/40.0;
	  TIM4->ARR = (uint32_t)round(12800.0*f_inv);
	  count=-1;
}
void init(void){

	//timer 2
	//HAL_TIM_Base_MspInit(&htim2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	//HAL_TIM_Base_Start_IT(&htim2);
	//HAL_NVIC_EnableIRQ(TIM2_IRQn);

	pwm_in.COUNT_TO_MKS = ((float)TIM2->PSC)/72.0;
	 // TIMER3 16 khz PWM
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	 // HAL_TIM_Base_MspInit(&htim3);
	  //HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	  //HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
	  //HAL_TIM_Base_Start_IT(&htim3);
	  //HAL_NVIC_EnableIRQ(TIM3_IRQn);

	//  dt = (float)(htim3.Init.Prescaler+1)*(float)(htim3.Init.Period)/64000000.0;
	  //
	 // HAL_TIM_Base_MspInit(&htim4);
	  HAL_TIM_Base_Start_IT(&htim4);
	  //HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_1);
	  HAL_NVIC_EnableIRQ(TIM4_IRQn);


	  adc1ch[0].Channel = ADC_CHANNEL_2;//B
	  adc1ch[0].Rank = 1;
	  adc1ch[0].SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	  adc1ch[1].Channel = ADC_CHANNEL_3;//C
	  adc1ch[1].Rank = 1;
	  adc1ch[1].SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	  adc1ch[2].Channel = ADC_CHANNEL_1;//A
	  adc1ch[2].Rank = 1;
	  adc1ch[2].SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	  adc1ch[3].Channel = ADC_CHANNEL_4;//CNT
	  adc1ch[3].Rank = 1;
	  adc1ch[3].SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	  adc1ch[4].Channel = ADC_CHANNEL_0;//pot
	  adc1ch[4].Rank = 1;
	  adc1ch[4].SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	  adc1ch[5].Channel = ADC_CHANNEL_6;//cur
	  adc1ch[5].Rank = 1;
	  adc1ch[5].SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	  //ADC1
	  //HAL_ADC_MspInit(&hadc1);
	  HAL_ADC_Start(&hadc1);
	  //adc2
	  HAL_ADC_MspInit(&hadc2);
	  //HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

	  //i2c
	  /*HAL_I2C_MspInit(&hi2c1);
	  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);*/

	  //uart3
	  /*HAL_UART_MspInit(&huart3);

	  //uart2
	  HAL_UART_MspInit(&huart2);

	  //spi2
	  HAL_SPI_MspInit(&hspi2);*/

	  /*PlaySound(100,100);
	  PlaySound(3,100);
	  PlaySound(100,100);
	  PlaySound(3,100);
	  PlaySound(100,100);
	  PlaySound(1,100);*/
}
/*void Accelerometer(unsigned char rw, unsigned char reg, unsigned char data_){
	ACC.RW = rw;
	ACC.Data = data_;
	ACC.Reg_Adress = reg | ACC.RW;

	HAL_GPIO_WritePin(CS_Acc_GPIO_Port, CS_Acc_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,&ACC.Reg_Adress,1,1);
	if(ACC.RW==READ)
		HAL_SPI_Receive(&hspi2,&ACC.Data,1,1);
	else
		HAL_SPI_Transmit(&hspi2,&ACC.Data,1,1);
	HAL_GPIO_WritePin(CS_Acc_GPIO_Port, CS_Acc_Pin, GPIO_PIN_SET);
}*/

void PlaySound(float f_,int delay){
	f=f_;
	f_inv =  1/(f*POLES*STATES);
	TIM4->ARR = (uint32_t)round(12800.0*f_inv);
	TIM3->CCR1 = 250;
	TIM3->CCR2 = 250;
	HAL_Delay(delay);
}

//ADC
void StartAnalogADC1( int ch ){
	HAL_ADC_ConfigChannel(&hadc1, &adc1ch[ch]);
	HAL_ADC_Start(&hadc1);
}
int GetAnalogADC1( void ){
	while( __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)==0 ){}
	return HAL_ADC_GetValue(&hadc1);
}
int ReadAnalogADC1( int ch ){
	HAL_ADC_ConfigChannel(&hadc1, &adc1ch[ch]);//A4 / B
	HAL_ADC_Start(&hadc1);
	while( __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)==0 ){}
	return HAL_ADC_GetValue(&hadc1);
}
void StartAnalogADC2( int ch ){
	HAL_ADC_ConfigChannel(&hadc2, &adc1ch[ch]);
	HAL_ADC_Start(&hadc2);
}
int GetAnalogADC2( void ){
	while( __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC)==0 ){}
	return HAL_ADC_GetValue(&hadc2);
	return 1;
}
int ReadAnalogADC2( int ch ){
	HAL_ADC_ConfigChannel(&hadc2, &adc1ch[ch]);
	HAL_ADC_Start(&hadc2);
	while( __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC)==0 ){}
	return HAL_ADC_GetValue(&hadc2);
	return 1;
}


void ResetControlLOW(void){
	HAL_GPIO_WritePin(AL_GPIO_Port, AL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CL_GPIO_Port, CL_Pin, GPIO_PIN_RESET);
}
void ResetControlHIGH(void){
	HAL_GPIO_WritePin(AH_GPIO_Port, AH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BH_GPIO_Port, BH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CH_GPIO_Port, CH_Pin, GPIO_PIN_RESET);
}
void SetControlHIGH(GPIO_PinState ah,GPIO_PinState al,GPIO_PinState bh,GPIO_PinState bl,GPIO_PinState ch,GPIO_PinState cl){
	HAL_GPIO_WritePin(AH_GPIO_Port, AH_Pin, ah);
	//HAL_GPIO_WritePin(AL_GPIO_Port, AL_Pin, al);
	HAL_GPIO_WritePin(BH_GPIO_Port, BH_Pin, bh);
	//HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, bl);
	HAL_GPIO_WritePin(CH_GPIO_Port, CH_Pin, ch);
	//HAL_GPIO_WritePin(CL_GPIO_Port, CL_Pin, cl);

	if(al)HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);else HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	if(bl)HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);else HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
	if(cl)HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);else HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);

}
void SetControlLOW(GPIO_PinState ah,GPIO_PinState al,GPIO_PinState bh,GPIO_PinState bl,GPIO_PinState ch,GPIO_PinState cl){
	//HAL_GPIO_WritePin(AH_GPIO_Port, AH_Pin, ah);
	HAL_GPIO_WritePin(AL_GPIO_Port, AL_Pin, al);
	//HAL_GPIO_WritePin(BH_GPIO_Port, BH_Pin, bh);
	HAL_GPIO_WritePin(BL_GPIO_Port, BL_Pin, bl);
	//HAL_GPIO_WritePin(CH_GPIO_Port, CH_Pin, ch);
	HAL_GPIO_WritePin(CL_GPIO_Port, CL_Pin, cl);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
