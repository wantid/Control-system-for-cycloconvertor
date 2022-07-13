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
   
    __enable_irq(); 

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
    
    TIM2->SR &= ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    TIM2->CR1 &= ~TIM_CR1_CEN;
    
	TIM2->PSC = 1-1;
	TIM2->CNT = 0;
	TIM2->ARR = PULSEW; 

    TIM2->SMCR |= (1 << 7);
    TIM2->CR1 &= ~TIM_CR1_DIR;
    
    TIM2->CR1 |= TIM_CR1_OPM;
	TIM2->DIER |= TIM_DIER_UIE;
    
    TIM2->CR1 |= TIM_CR1_CEN;
    
    TIM2->EGR |= TIM_EGR_UG;
    
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn , 0);
}

void TIM1_init(void)
{
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
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //CH1,2,3
    
    //PC6 - CH1
    GPIOC->MODER |= (2 << 12);
    GPIOC->OSPEEDR |= (3 << 12);
    GPIOC->AFR[0] |= (3 << 24); 
    //PÑ7 - CH2
    GPIOC->MODER |= (2 << 14);
    GPIOC->OSPEEDR |= (3 << 14);
    GPIOC->AFR[0] |= (3 << 28);
    //PÑ8 - CH3
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
