/**
 *  独立工作、非扫描、单次转换、定时器触发模式（规则通道）
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay(volatile unsigned long n);
void UART1_Init(void);
void ADC1_Init(void);
unsigned short Get_Adc(void);
/* Private functions ---------------------------------------------------------*/
void delay(volatile unsigned long n)
{
  while(n--);
}

void UART1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  USART_InitTypeDef USART_InitStructure;  

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);  
    
  /* Configure USART Tx as alternate function push-pull */  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
    
  /* Configure USART Rx as input floating */  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  USART_InitStructure.USART_BaudRate = 115200;  
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  
  USART_InitStructure.USART_Parity = USART_Parity_No;  
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
  USART_Init(USART1, &USART_InitStructure );   

  USART_Cmd(USART1, ENABLE); 
}

int fputc(int ch, FILE *f)
{

  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
  return (ch);
}

void ADC1_Init(void)
{
  RCC->APB2ENR  |= 1<<2;      //使能 PORTA 口时钟
  GPIOA->CRL    &= 0XFFFFFF00;
  GPIOA->CRL    |= 12<<0;     //PA0 复用推挽输入
  RCC->APB2ENR  |= 1<<9;      //ADC1 时钟使能
  RCC->APB2RSTR |= 1<<9;      //ADC1 复位
  RCC->APB2RSTR &= ~(1<<9);   //复位结束
  RCC->CFGR     &= ~(3<<14);  //分频因子清零
  //SYSCLK/DIV6=12M ADC 时钟设置为 12M,ADC 最大时钟不能超过 14M!
  //否则将导致 ADC 准确度下降!
  RCC->CFGR |= 2<<14;
  
  ADC1->CR1 &= 0XF0FFFF; //工作模式清零
  ADC1->CR1 |= 0<<16;    //独立工作模式
  ADC1->CR1 &= ~(1<<8);  //非扫描模式
  
  ADC1->CR2 &= ~(1<<1);    //单次转换模式
  ADC1->CR2 &= ~(7<<17); 
  ADC1->CR2 |= 2<<17;   //定时器1的CC3事件控制转换
  ADC1->CR2 |= 1<<20;   //使用用外部触发
  ADC1->CR2 &= ~(1<<11);//右对齐
  
  ADC1->SQR1 &= ~(0XF<<20);
  ADC1->SQR1 |= 0<<20;   //1 个转换在规则序列中
  
  //设置通道 0 的采样时间
  ADC1->SMPR2 &= ~(7<<0); //通道 0 采样时间清空
  ADC1->SMPR2 |= 7<<0;    //通道 0 239.5 周期,提高采样时间可以提高精确度
  
  //设置转换序列 
  ADC1->SQR3 &= 0XFFFFFFE0;//规则序列第一个转换0通道
  ADC1->SQR3 |= (0<<0); 
  
  ADC1->CR2 |= 1<<0;  //开启 AD 转换器
  ADC1->CR2 |= 1<<3;  //使能复位校准
  while(ADC1->CR2 & 1<<3); //等待校准结束
  //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。
  ADC1->CR2 |= 1<<2;       //开启 AD 校准
  while(ADC1->CR2 & 1<<2); //等待校准结束
}

//读取ADC
unsigned short Get_Adc()
{
  while(!(ADC1->SR & 1<<1));//等待转换结束 
  return ADC1->DR;          //返回 adc 值
}

void Key_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
  EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
}

void TIM1_Configuration(void)  
{   
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;   
  TIM_OCInitTypeDef TIM_OCInitStructure;   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);   
  TIM_TimeBaseStructure.TIM_Period = 10000; 
  TIM_TimeBaseStructure.TIM_Prescaler = 7199; 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;   
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM1, & TIM_TimeBaseStructure);  
      
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//下面详细说明   
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//TIM_OutputState_Disable;   
  TIM_OCInitStructure.TIM_Pulse = 1000;   
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//如果是PWM1要为Low，PWM2则为High   
  TIM_OC3Init(TIM1, & TIM_OCInitStructure);     

  TIM_CtrlPWMOutputs(TIM1, ENABLE);  
  TIM_Cmd(TIM1, ENABLE);
} 

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  Key_Init();
  UART1_Init();
  ADC1_Init();
  TIM1_Configuration();
  printf("Start!\n");
  /* Infinite loop */
  while (1)
  {
    while(!(ADC1->SR & 1<<1));//等待转换结束 
    printf("AD = %d\n", ADC1->DR);
  }
}

void EXTI15_10_IRQHandler(void)
{ 
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    
    EXTI_ClearITPendingBit(EXTI_Line12);
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
