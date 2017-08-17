/**
 *  独立工作、扫描、连续转换、DMA、软件触发模式（规则通道）
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned short AD_Value[2] = {0};
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
  GPIOA->CRL    |= 12<<4;     //PA1 复用推挽输入
  RCC->APB2ENR  |= 1<<9;      //ADC1 时钟使能
  RCC->APB2RSTR |= 1<<9;      //ADC1 复位
  RCC->APB2RSTR &= ~(1<<9);   //复位结束
  RCC->CFGR     &= ~(3<<14);  //分频因子清零
  //SYSCLK/DIV6=12M ADC 时钟设置为 12M,ADC 最大时钟不能超过 14M!
  //否则将导致 ADC 准确度下降!
  RCC->CFGR |= 2<<14;
  
  ADC1->CR1 &= 0XF0FFFF; //工作模式清零
  ADC1->CR1 |= 0<<16;    //独立工作模式
  ADC1->CR1 |= 1<<8;     //扫描模式
  
  ADC1->CR2 |= 1<<1;    //连续转换模式
  ADC1->CR2 &= ~(7<<17); 
  ADC1->CR2 |= 7<<17;   //软件控制转换
  ADC1->CR2 |= 1<<20;   //使用用外部触发
  ADC1->CR2 &= ~(1<<11);//右对齐
  
  ADC1->SQR1 &= ~(0XF<<20);
  ADC1->SQR1 |= 1<<20;   //2 个转换在规则序列中
  
  //设置通道 0 的采样时间
  ADC1->SMPR2 &= ~(7<<0); //通道 0 采样时间清空
  ADC1->SMPR2 |= 7<<0;    //通道 0 239.5 周期,提高采样时间可以提高精确度
  
  //设置通道 1 的采样时间
  ADC1->SMPR2 &= ~(7<<3); //通道 1 采样时间清空
  ADC1->SMPR2 |= 7<<3;    //通道 1 239.5 周期,提高采样时间可以提高精确度
  
  //设置转换序列 
  ADC1->SQR3 &= 0XFFFFFC00;//规则序列第一个转换0通道
  ADC1->SQR3 |= (0<<0); 
  ADC1->SQR3 |= (1<<5);
  
  ADC1->CR2 |= 1<<8;  //开启DMA
  
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

void DMA1_Init(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输
  DMA_DeInit(DMA1_Channel1); //将DMA的通道1寄存器重设为缺省值
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&ADC1->DR; //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr =(u32)&AD_Value; //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//内存作为数据传输的目的地
  DMA_InitStructure.DMA_BufferSize = 2; //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc =DMA_PeripheralInc_Disable; //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc =DMA_MemoryInc_Enable; //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize =DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize =DMA_MemoryDataSize_HalfWord; //数据宽度为16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在循环缓存模式
  DMA_InitStructure.DMA_Priority =DMA_Priority_High; //DMA通道 x拥有高优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据DMA_InitStruct中指定的参数初始化DMA的通道
  DMA_Cmd(DMA1_Channel1, ENABLE); //启动DMA通道
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  Key_Init();
  DMA1_Init();
  UART1_Init();
  ADC1_Init();
  printf("Start!\n");
  
  ADC1->CR2  |= 1<<22;      //启动规则转换通道
  
  /* Infinite loop */
  while (1)
  {
    delay(5000000);
    printf("AD1 = %04d AD2 = %04d\n", AD_Value[0], AD_Value[1]);
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
