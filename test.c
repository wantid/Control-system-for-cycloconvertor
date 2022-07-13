/*
#include "stm32f7xx.h"
#include <math.h>

#define CLOCKFREQ 33333333
#define FSIN 1
#define DEADTIME 18
#define VREF 3.3

void TIM1_init(void), clock_config(void), SinUpdate(void), 
    Initial_Data(void), ADC_init(void), get_adc_value(void);;

int32_t maxCount, amplitude, points, step = 0;

uint32_t sin1, sin2, sin3; 

uint32_t res, readyFlag = 1; //АЦП

float voltage;

int main(void)
{
    Initial_Data();
    clock_config();
    TIM1_init();
    ADC_init();
    
    while(1)
    {

    }
}

void Initial_Data(void)
{
    maxCount = CLOCKFREQ/10000;
    amplitude = maxCount/2;
    points = CLOCKFREQ/(FSIN*maxCount); //333
}

void get_adc_value(void)
{
    if (readyFlag == 2) readyFlag = 1;
    
    if (readyFlag == 1)
    {
        readyFlag = 0;
        int temp = 0;
        for (int i = 0; i < 10; i++)
        {
            ADC1->CR2 |= ADC_CR2_SWSTART;
            while (!(ADC1->SR & ADC_SR_EOC));
            temp += ADC1->DR;
            if (i == 9) res = temp/10;
            //запуск АЦП, 10 измерений -> среднее, в дебаге смотреть
        }
        voltage = res * VREF / 4096;
        readyFlag = 2;
    }
}

void ADC_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= (11 << 0); //ch10
    GPIOC->MODER |= (11 << 2); //ch11
    GPIOC->MODER |= (11 << 4); //ch12
    GPIOC->MODER |= (11 << 6); //ch13
    GPIOC->MODER |= (11 << 8); //ch14
    GPIOC->MODER |= (11 << 10); //ch15
    GPIOA->MODER |= (11 << 0); //PA0 analog input
    
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    ADC1->CR2 &= ~ADC_CR2_ADON; 
    
    ADC1->SQR1 = 0;//1 регулярный канал
    ADC1->SQR3 = 1010;
    //ADC1->SQR3 = 0;//1 преобразование - канал 0
    
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 &= ~ADC_CR1_SCAN;
    //ADC->CCR |= (10 << 16); //div6
    //ADC1->SMPR2 |= (011 << 0); //55 cycles
    
    ADC1->CR2 |= ADC_CR2_ADON;
}

void TIM1_init(void)
{
    
    // Тактирование  GPIOA , TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //CH1,2,3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //CH1n,2n,3n
    
    //PA8 - CH1
    GPIOA->MODER |= (10 << 16);
    GPIOA->OSPEEDR |= (11 << 16);
    GPIOA->AFR[1] |= (0001 << 0);
    //PA9 - CH2
    GPIOA->MODER |= (10 << 18);
    GPIOA->OSPEEDR |= (11 << 18);
    GPIOA->AFR[1] |= (0001 << 4);
    //PA10 - CH3
    GPIOA->MODER |= (10 << 20);
    GPIOA->OSPEEDR |= (11 << 20);
    GPIOA->AFR[1] |= (0001 << 8);
    
    //PB13 - CH1n
    GPIOB->MODER |= (10 << 26);
    GPIOB->OSPEEDR |= (11 << 26);
    GPIOB->AFR[1] |= (0001 << 20);
    //PB14 - CH2n
    GPIOB->MODER |= (10 << 28);
    GPIOB->OSPEEDR |= (11 << 28);
    GPIOB->AFR[1] |= (0001 << 24);
    //PB15 - CH3n
    GPIOB->MODER |= (10 << 30);
    GPIOB->OSPEEDR |= (11 << 30);
    GPIOB->AFR[1] |= (0001 << 28);
	
	TIM1->PSC = 1-1;
    TIM1->ARR = maxCount;

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    //TIM1->CCER ^= (00 << 0) ^ (00 << 2); //Ch1 Ch1n for inversed dead time

    //Настройка DeadTime
    TIM1->BDTR = TIM_BDTR_LOCK_0 | TIM_BDTR_MOE | DEADTIME;
    
    //TIM1->CCER |= (01 << 0) | (01 << 2);
    
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NE | TIM_CCER_CC1NP;
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NE | TIM_CCER_CC2NP;
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NE | TIM_CCER_CC3NP;
     
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;

	TIM1->CR1 &= ~TIM_CR1_DIR; 
	TIM1->CR1 |= (11 << 5); 
    
    
    TIM1->DIER |= TIM_DIER_CC4IE;
    TIM1->CR1 |= TIM_CR1_CEN;
    
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    
    SinUpdate();

    step = 1; 
}

void SinUpdate(void)
{
    sin1 = amplitude + amplitude*sin(1.571*4*step/points);
    sin2 = amplitude + amplitude*sin(1.571*4*step/points - 2.094);
    sin3 = amplitude + amplitude*sin(1.571*4*step/points + 2.094); 
    
    TIM1->CCR1 = sin1; 
    TIM1->CCR2 = sin2;
    TIM1->CCR3 = sin3;
}

void TIM1_CC_IRQHandler(void){
    if (step > points) step = 0;
    
    SinUpdate();
    
    step++;
    
    get_adc_value();
}

void clock_config(void)
{

    //RCC->CR |= RCC_CR_HSEON;
    //while ((RCC->CR & RCC_CR_HSERDY) == 0);
    
    //RCC -> PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    //RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLM_5;
    //RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLM_4;
    //RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLM_3;
    //RCC -> PLLCFGR |= RCC_PLLCFGR_PLLM_2;
    //RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLM_1;
    //RCC -> PLLCFGR &= ~RCC_PLLCFGR_PLLM_0;
    //RCC -> PLLCFGR |= (00001010 << 6); //*216
    //RCC -> PLLCFGR |= (00 << 16); //*2

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; 
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= (11 << 21); //PLL clock selected for MCO1
    RCC->CFGR |= (11 << 30);
    
    //RCC->CFGR ^= (0000 << 4);
    //RCC->CFGR ^= (000 << 10);
    //RCC->CFGR ^= (000 << 13);
    
    RCC->CFGR |= (0000 << 4); //AHB /1
    RCC->CFGR |= (100 << 10); //APB1 /2
    RCC->CFGR |= (100 << 13); //APB2 /2
    
    //RCC->PLLCFGR ^= (000000 << 0) ^ (00000000 << 6) ^ (00 << 16);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI | (001000 << 0) | (010110100 << 6) | (10 << 16);
    //RCC->PLLCFGR |= (001000 << 0); //div8
    //RCC->PLLCFGR |= (01100100 << 6); //*216
    //RCC->PLLCFGR |= (10 << 16); //*2
    
    RCC->CR |= RCC_CR_PLLON;
    
    RCC->CFGR |= (10 << 0); //PLL as system clock
    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; //PB(светодиоды) и PC(кнопка)
    //GPIOB->MODER |= (1 << 0) | (1 << 14) | (1 << 28); //Режим вывода светодиодов
}*/

/*
//НПЧ
#include "stm32f7xx.h"

#define LED1 (1 << 0)
#define LED2 (1 << 7)
#define LED3 (1 << 14)

#define PULSEW 16000000/(f*6)

void Gpio_init(void), systick_init(void), RCC_init(void);

static unsigned int pulse = 1, f = 50;

int main(void)
{
    unsigned int t=0;
    
    RCC_init();
    Gpio_init();
    systick_init();
    
    while(1)
    {
        
    }
}

void SysTick_Handler(void)
{
    //GPIOE->ODR |= (1 << 2);
    //GPIOE->ODR |= (1 << 4);
    //GPIOE->ODR |= (1 << 5);
    //GPIOF->ODR |= (1 << 2);
    //GPIOF->ODR |= (1 << 8);
    //GPIOF->ODR |= (1 << 9);

    GPIOE->ODR = 0;
    GPIOF->ODR = 0;
    
    
    switch(pulse)
    {
        case 1:
            GPIOE->ODR |= (1 << 2); //S1
            GPIOB->ODR |= LED3;
            break;
        case 2:
            GPIOE->ODR |= (1 << 4); //S2
            GPIOB->ODR &= (0 << 14); //Отключение LED3
            break;
        case 3:
            GPIOE->ODR |= (1 << 5); //S3
            break;
        case 4:
            GPIOF->ODR |= (1 << 2); //S4
            break;
        case 5:
            GPIOF->ODR |= (1 << 8); //S5
            break;
        case 6:
            GPIOF->ODR |= (1 << 9); //S6
            pulse = 0;
            break;
    }
    pulse++;
}

void systick_init(void)
{   
    SysTick->LOAD = PULSEW;
    SysTick->VAL = PULSEW;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

void RCC_init(void)
{
    RCC->CR |= RCC_CR_HSION; //HSI ON 16Mhz
    while (!(RCC->CR & RCC_CR_HSIRDY)); //Ожидание включения HSI
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //AHB 16Mhz/1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; //APB1 16Mhz/1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; //APB2 16Mhz/1
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; //PB(светодиоды) и PC(кнопка)
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN; //тактирование системного таймера
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN; //ключи
}

void Gpio_init(void)
{  
    GPIOB->MODER |= (1 << 0) | (1 << 14) | (1 << 28); //Режим вывода светодиодов
    GPIOE->MODER |= (1 << 4) | (1 << 8) | (1 << 10); //Ключи S1-3
    GPIOF->MODER |= (1 << 4) | (1 << 16) | (1 << 18); //Ключи S4-6
    
    GPIOE->OSPEEDR |= (3 << 4) | (3 << 8) | (3 << 10);//время спада 2мс, 500 Гц
    GPIOF->OSPEEDR |= (3 << 4) | (3 << 16) | (3 << 18);
}
*/

//Инвертор +датчик
#include "stm32f7xx.h"
#include <math.h>

#define CLOCKFREQ 16000000 //33333333
#define FSIN (50 - f_st)*1.225 //*1.22
#define DEADTIME 18

void TIM1_init(void), TIM6_init(void), ETR_init(void), ETR_Handler(void), 
    clock_config(void), SinUpdate(void), Initial_Data(void);

int32_t maxCount, amplitude, points, step = 0;

uint32_t sin1, sin2, sin3; 

//Счёт оборотов
volatile uint32_t impulses, f_st;

int main(void)
{
    Initial_Data();
    //clock_config();
	
    TIM1_init();
	
	// Счёт оборотов, посмотреть глобальные прерывания
	ETR_init();
	TIM6_init();
	
	// Приоритеты прерываний 
	NVIC_SetPriority(TIM1_CC_IRQn, 1);
	NVIC_SetPriority(TIM6_DAC_IRQn , 0);
    //__enable_irq(); //Включение глобальных прерываний
    
    while(1)
    {

    }
}

void SinUpdate(void)
{
    if (f_st < 50)
    {
        if (f_st >= 1) points = CLOCKFREQ/(FSIN*maxCount); 
        sin1 = amplitude + amplitude*sin(1.571*4*step/points); //1.571
        sin2 = amplitude + amplitude*sin(1.571*4*step/points - 2.094);
        sin3 = amplitude + amplitude*sin(1.571*4*step/points + 2.094); 
    } else if (f_st > 50)
    {
        points = CLOCKFREQ/(-FSIN*maxCount); 
        sin1 = amplitude + amplitude*sin(1.571*4*step/points);
        sin2 = amplitude + amplitude*sin(1.571*4*step/points + 2.094);
        sin3 = amplitude + amplitude*sin(1.571*4*step/points - 2.094); 
    } else
    {
        sin1 = maxCount;
        sin2 = 0;
        sin3 = 0; 
    }
    
    TIM1->CCR1 = sin1; 
    TIM1->CCR2 = sin2;
    TIM1->CCR3 = sin3;
}

void TIM1_CC_IRQHandler(void){
    if (step > points) step = 0;
    
    SinUpdate();
    
    step++;
}

void TIM6_DAC_IRQHandler(void) //Посмотреть название функции
{
	TIM2->CR1 &= ~TIM_CR1_CEN; //Остановка счета испульсов
	
	impulses = TIM2->CNT;
    f_st = impulses * .05f;
	
	TIM2->CNT = 0;
	TIM6->SR &= ~TIM_SR_UIF; //Сброс флага прерывания
	
	TIM2->CR1 |= TIM_CR1_CEN; //Запуск таймера
	TIM6->CR1 |= TIM_CR1_CEN; //Запуск таймера
}

void Initial_Data(void)
{
    maxCount = CLOCKFREQ/10000;
    amplitude = maxCount/2;
    points = CLOCKFREQ/(FSIN*maxCount); 
}

void TIM1_init(void)
{
    // Тактирование  GPIOA , TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //CH1,2,3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //CH1n,2n,3n
    
    //PA8 - CH1
    GPIOA->MODER |= (2 << 16);
    GPIOA->OSPEEDR |= (3 << 16);
    GPIOA->AFR[1] |= (1 << 0); 
    //PA9 - CH2
    GPIOA->MODER |= (2 << 18);
    GPIOA->OSPEEDR |= (3 << 18);
    GPIOA->AFR[1] |= (1 << 4);
    //PA10 - CH3
    GPIOA->MODER |= (2 << 20);
    GPIOA->OSPEEDR |= (3 << 20);
    GPIOA->AFR[1] |= (1 << 8); 
    //PB13 - CH1n
    GPIOB->MODER |= (2 << 26);
    GPIOB->OSPEEDR |= (3 << 26);
    GPIOB->AFR[1] |= (1 << 20);
    //PB14 - CH2n
    GPIOB->MODER |= (2 << 28);
    GPIOB->OSPEEDR |= (3 << 28);
    GPIOB->AFR[1] |= (1 << 24);
    //PB15 - CH3n
    GPIOB->MODER |= (2 << 30);
    GPIOB->OSPEEDR |= (3 << 30);
    GPIOB->AFR[1] |= (1 << 28);
	
	TIM1->PSC = 1-1;
    TIM1->ARR = maxCount;

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    //Настройка DeadTime
    TIM1->BDTR = TIM_BDTR_LOCK_0 | TIM_BDTR_MOE | DEADTIME;

    
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NE | TIM_CCER_CC1NP;
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NE | TIM_CCER_CC2NP;
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NE | TIM_CCER_CC3NP;
     
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;

	TIM1->CR1 &= ~TIM_CR1_DIR; 
	TIM1->CR1 |= (3 << 5); 
    
    
    TIM1->DIER |= TIM_DIER_CC4IE;
    TIM1->CR1 |= TIM_CR1_CEN;
    
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    
    SinUpdate();

    step = 1; 
}

void TIM6_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	
	TIM6->PSC = maxCount-1; //Подобрать предделитель для новой частоты
	TIM6->ARR = 10000; //Период счета, должен быть равен примерно 1ой секунде
	TIM6->CR1 &= ~TIM_CR1_DIR; //Счёт вверх
	TIM6->CR1 |= TIM_CR1_OPM; //Досчитав до конца, таймер отключается
	TIM6->DIER |= TIM_DIER_UIE; //Разрешить прерывания
	
    TIM6->CR1 |= TIM_CR1_CEN; //Запуск таймера
    
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void ETR_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
    //PA0 - CH1
    GPIOA->MODER |= (2 << 0);
    GPIOA->AFR[0] |= (1 << 0); //AF1 = TIM2_CH1_ETR
    
	TIM2->PSC = 1-1;
	TIM2->CNT = 0;
	TIM2->ARR = 60000; //подобрать значение
	TIM2->CR1 &= ~TIM_CR1_DIR;
	
    //TIM2->SMCR = (0000 << 8) | (0 << 12) | (0 << 15) | (<< 14);

	TIM2->SMCR |= (9 << 8);//TIM2->SMCR &= ~TIM_SMCR_ETF; //фильтр
	TIM2->SMCR &= ~TIM_SMCR_ETPS; //предделитель 0
	TIM2->SMCR &= ~TIM_SMCR_ETP; //возрастающий -> ~TIM_SMCR_ETP
	TIM2->SMCR |= TIM_SMCR_ECE; //внешнее тактирование
    
    TIM2->CR1 |= TIM_CR1_CEN; //Запуск таймера
}

void clock_config(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; 
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= (3 << 21); //PLL clock selected for MCO1
    RCC->CFGR |= (3 << 30);
    
    RCC->CFGR |= (0 << 4); //AHB /1
    RCC->CFGR |= (4 << 10); //APB1 /2
    RCC->CFGR |= (4 << 13); //APB2 /2
    
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI | (8 << 0) | (180 << 6) | (2 << 16);

    RCC->CR |= RCC_CR_PLLON;
    RCC->CFGR |= (2 << 0); //PLL as system clock
}

/*
//НПЧ6 Два таймера
#include "stm32f7xx.h"

#define PULSEW 16000000/300 // 16MHz/50Hz*6

void TIM1_init(void), TIM8_init(void), TIM2_init(void);

uint16_t pulse; 

int main(void)
{
    __disable_irq();
    
    TIM2_init();
    TIM1_init();
    TIM8_init();
   
    __enable_irq(); //Включение глобальных прерываний
    
    
    while(1)
    {

    }
}

void TIM2_IRQHandler(void)
{
    switch(pulse)
    {
        case 1:
            TIM1->CCR1 = PULSEW;
            TIM8->CCR3 = 0;
            break;
        case 2:
            TIM1->CCR2 = PULSEW;
            TIM1->CCR1 = 0;
            break;
        case 3:
            TIM1->CCR3 = PULSEW;
            TIM1->CCR2 = 0;
            break;
        case 4:
            TIM8->CCR1 = PULSEW;
            TIM1->CCR3 = 0;
            break;
        case 5:
            TIM8->CCR2 = PULSEW;
            TIM8->CCR1 = 0;
            break;
        default:
            TIM8->CCR3 = PULSEW;
            TIM8->CCR2 = 0;
            pulse = 0;
            break;
    }
    pulse++;
    
    TIM2->SR &= ~TIM_SR_UIF; //Сброс флага прерывания
	TIM2->CR1 |= TIM_CR1_CEN; //Запуск таймера
}

void TIM2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    TIM2->CR1 &= ~TIM_CR1_CEN;
    
	TIM2->PSC = 1-1;
	TIM2->CNT = 0;
	TIM2->ARR = PULSEW; 

    TIM2->SMCR |= (1 << 7); //Reset mode
    TIM2->CR1 &= ~TIM_CR1_DIR; //Счёт вверх
    
    TIM2->CR1 |= TIM_CR1_OPM; //Досчитав до конца, таймер отключается
	TIM2->DIER |= TIM_DIER_UIE; //Разрешить прерывания
    
    TIM2->CR1 |= TIM_CR1_CEN; //Запуск таймера
    
    TIM2->EGR |= TIM_EGR_UG; //Шлем Update для сброса
    
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn , 0);
}

void TIM1_init(void)
{
    // Тактирование  GPIOA , TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //CH1,2,3
    
    //PA8 - CH1
    GPIOA->MODER |= (2 << 16);
    GPIOA->OSPEEDR |= (3 << 16);
    GPIOA->AFR[1] |= (1 << 0); 
    //PA9 - CH2
    GPIOA->MODER |= (2 << 18);
    GPIOA->OSPEEDR |= (3 << 18);
    GPIOA->AFR[1] |= (1 << 4);
    //PA10 - CH3
    GPIOA->MODER |= (2 << 20);
    GPIOA->OSPEEDR |= (3 << 20);
    GPIOA->AFR[1] |= (1 << 8); 
	
	TIM1->PSC = 1-1;
    TIM1->ARR = PULSEW;

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
     
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    
    TIM1->BDTR = TIM_BDTR_MOE;
	TIM1->CR1 &= ~TIM_CR1_DIR; 

    TIM1->SMCR |= (1 << 4); //Internal trigger = TIM2
    TIM1->SMCR |= (1 << 16); //Reset mode
    
    TIM1->CR1 |= TIM_CR1_CEN;
}
void TIM8_init(void)
{
    // Тактирование  GPIOA , TIM8
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //CH1,2,3
    
    //PC6 - CH1
    GPIOC->MODER |= (2 << 12);
    GPIOC->OSPEEDR |= (3 << 12);
    GPIOC->AFR[0] |= (3 << 24); 
    //PС7 - CH2
    GPIOC->MODER |= (2 << 14);
    GPIOC->OSPEEDR |= (3 << 14);
    GPIOC->AFR[0] |= (3 << 28);
    //PС8 - CH3
    GPIOC->MODER |= (2 << 16);
    GPIOC->OSPEEDR |= (3 << 16);
    GPIOC->AFR[1] |= (3 << 0); 
	
	TIM8->PSC = 1-1;
    TIM8->ARR = PULSEW;

    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
    
    TIM8->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
     
    TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 
    TIM8->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM8->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    
    TIM8->BDTR = TIM_BDTR_MOE;
	TIM8->CR1 &= ~TIM_CR1_DIR; 

    TIM8->SMCR |= (1 << 4); //Internal trigger = TIM2
    TIM8->SMCR |= (1 << 16); //Reset mode
    
    TIM8->CR1 |= TIM_CR1_CEN;
}
*/
