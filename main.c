/**
  ******************************************************************************
  * @file    SysTick/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

/** @addtogroup Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
static __IO uint32_t BurstEventsCounter;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);
void BaseTimerSetup();
void PulseBurstTimerSetup();
void DisableMainSeq();
void EnableMainSeq();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    BaseTimerSetup();
    PulseBurstTimerSetup();
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

  /* Initialize Leds LD3 and LD4 mounted on STM32VLDISCOVERY board */  
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);

  /* Turn on LD3 and LD4 */
  STM32vldiscovery_LEDOn(LED3);
  //STM32vldiscovery_LEDOn(LED4);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }

  while (1)
  {
    /* Toggle LD3 */
    STM32vldiscovery_LEDToggle(LED3);

    /* Insert 50 ms delay */
    Delay(50);

    /* Toggle LD4 */
   //STM32vldiscovery_LEDToggle(LED4);

    /* Insert 100 ms delay */
    Delay(100);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void HandleBurstTimerTrigger(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
    {
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	STM32vldiscovery_LEDToggle(LED4);
	BurstEventsCounter++;
	if (BurstEventsCounter & 0x01) {
	    EnableMainSeq();
	} else {
	    DisableMainSeq();
	}
    }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
 * @brief Base (main sequence) timer setup: (~1 kHz period, 500 us impulses), PA_1 output
 */
void BaseTimerSetup() 
{
    // Before we interact to timers we need to initialize proper periferial bus
    GPIO_InitTypeDef  gpio_init;
    TIM_TimeBaseInitTypeDef tim_base_init;
    TIM_OCInitTypeDef  tim_oc_init; 
    TIM_TypeDef* tim = TIM2;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

    // Init GPIO
    // Pin A_1 is attached to Tim2 OC2, set to alternative push-pull
    gpio_init.GPIO_Pin = GPIO_Pin_1;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init); 
    // Configure the timer, 24 MHz internal clock, 1 MHz after prescaler
    TIM_TimeBaseStructInit(&tim_base_init);
    tim_base_init.TIM_Period = 500; // Period 1 kHz, ratio 2
    tim_base_init.TIM_Prescaler = 23; // 1 us PSC output
    tim_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &tim_base_init);
    // Configure timer output channel
    tim_oc_init.TIM_OCMode = TIM_OCMode_Toggle;
    tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init.TIM_Pulse = 0;
    tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_High; 
    TIM_OC2Init(tim, &tim_oc_init);
    TIM_SetCompare2(tim, 20);
    // Configure I/O alternate function (AFIO), it is on APB2
    // Ref STM32F100 reference manual, sec. 2.3 Table 1
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(tim, ENABLE);
    // Remap output
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    //GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );
    TIM_Cmd(tim, ENABLE);
}

/**
 * @brief Pulse burst timer setup, 1 sec period, 500 ms burst lenght, PA_7 output
 */
void PulseBurstTimerSetup() 
{
    GPIO_InitTypeDef  gpio_init;
    TIM_TimeBaseInitTypeDef tim_base_init;
    TIM_OCInitTypeDef  tim_oc_init; 
    NVIC_InitTypeDef nvic_init;
    TIM_TypeDef* tim = TIM3;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

    // Init GPIO
    // Pin PA_7 is attached to Tim3 OC2, set to alternative push-pull
    gpio_init.GPIO_Pin = GPIO_Pin_7;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init); 

    // Configure the timer, 24 MHz internal clock, 1 MHz after prescaler
    TIM_TimeBaseStructInit(&tim_base_init);
    tim_base_init.TIM_Period = 500; // Period 500 ms
    tim_base_init.TIM_Prescaler = 24000; // 1 ms PSC output
    tim_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &tim_base_init);

    // Configure timer output channel
    tim_oc_init.TIM_OCMode = TIM_OCMode_Toggle;
    tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init.TIM_Pulse = 0;
    tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_High; 
    TIM_OC2Init(tim, &tim_oc_init);
    TIM_SetCompare2(tim, 20);

    // Configure I/O alternate function (AFIO), it is on APB2
    // Ref STM32F100 reference manual, sec. 2.3 Table 1
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(tim, ENABLE);

    // Enable the TIM3 global Interrupt
    nvic_init.NVIC_IRQChannel = TIM3_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 1;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);


    // Enable interrupt on counter compare
    TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
    //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(tim, ENABLE);
}

/**
 * @brief Disables impulses main sequence
 */
void DisableMainSeq()
{
    // Keep the timer working, just connect output to port and reset
    GPIO_InitTypeDef  gpio_init;
    gpio_init.GPIO_Pin = GPIO_Pin_1;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init); 
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void EnableMainSeq()
{
    // Connect output to timer 
    GPIO_InitTypeDef  gpio_init;
    gpio_init.GPIO_Pin = GPIO_Pin_1;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init); 
}


/**
 * @}
 */



/******************* (C) COPYRIGHT 2015 Yiri Borisov *****END OF FILE****/
