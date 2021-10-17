#include "stm32f7xx.h"                  // Device header
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "stdio.h"

#define APB1_DIV4_PRESCALER						  RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0
#define APB2_DIV2_PRESCALER						  RCC_CFGR_PPRE2_2
#define HSE_CLOCK_SOURCE								RCC_CFGR_SW_0
#define HSE_CLOCK_USED  								RCC_CFGR_SWS_0

#define PLL_M_4											    RCC_PLLCFGR_PLLM_2
#define PLL_N_72											  RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_3
#define PLL_P_4  											  RCC_PLLCFGR_PLLP_0
#define PLL_SOURCE_HSE									RCC_PLLCFGR_PLLSRC_HSE

#define POWER_SCALING_3									PWR_CR1_VOS_0

#define GPIOF0_ALTERNATE_FUNCTION       GPIO_MODER_MODER0_1
#define GPIOF1_ALTERNATE_FUNCTION       GPIO_MODER_MODER1_1
#define GPIOF2_ALTERNATE_FUNCTION       GPIO_MODER_MODER2_1

#define GPIOF0_OPEN_DRAIN       				GPIO_OTYPER_OT0
#define GPIOF1_OPEN_DRAIN       				GPIO_OTYPER_OT1
#define GPIOF2_OPEN_DRAIN       				GPIO_OTYPER_OT2

#define GPIOF0_SPEED_3									GPIO_OSPEEDER_OSPEEDR0
#define GPIOF1_SPEED_3									GPIO_OSPEEDER_OSPEEDR1
#define GPIOF2_SPEED_3									GPIO_OSPEEDER_OSPEEDR2

#define GPIOF0_PULL_UP									GPIO_PUPDR_PUPDR0_1
#define GPIOF1_PULL_UP									GPIO_PUPDR_PUPDR1_1
#define GPIOF2_PULL_UP									GPIO_PUPDR_PUPDR2_1

#define GPIOF0_AF_I2C2_SDA							GPIO_AFRL_AFRL0_2
#define GPIOF1_AF_I2C2_SCL							GPIO_AFRL_AFRL1_2
#define GPIOF0_AF_I2C2_SMBA							GPIO_AFRL_AFRL2_2

#define PWM_MIN                         25U
#define PWM_MAX                         125U
#define TEMP_MIN                        25U
#define TEMP_MAX                        60U

#define STTS22H_ADDRESS									0x7EU

#define STTS22H_CTRL                    (uint8_t)0x04
#define STTS22H_SOFTWARE_RESET          (uint8_t)0x0C
#define STTS22H_TEMP_L_OUT              (uint8_t)0x06
#define STTS22H_TEMP_H_OUT              (uint8_t)0x06

#define SW_RESET                        (uint8_t)0x02
#define STTS22G_CONFIG									(uint8_t)0x0C

#define CC_INTERUPT_PASSES              2U 
#define TIMER4_CLOCK										18000000.f

#define NUMBER_OF_MEANS									(uint8_t)8

#define DISTANCE_MIN                    (float).1
#define DISTANCE_MAX                    (float).6	

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
/*----------------------------------------------------------------------------------------------*/
//configure SYSCLK HCLK PCLK1 and PLCK2, PLL is used sourced by HSE
static void clkConfig(void);
//initalizes pins for I2C use
static void GPIOinit(void);
static void Timer7Init(void);
static void Timer6Init(void);
static void I2Cinit(void);
static void Timer1Init(void);
static void USART3Init(void);
static void Timer25ROPMInit (void);
static void Timer34EchoInit (void);
static void buttonTim13Init (void);

void TIM7_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
//void TIM2_IRQHandler(void);
//void TIM5_IRQHandler(void);
void TIM4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);

static void configSensor(void);
static void readTemp(void);
static void USART3Transmit(uint8_t *pData);
static void initalizeQueue(void);
static float queueProcess(uint32_t element);
static void calcSoS(void);

static volatile uint16_t temperature_reg = 0;
static volatile float temperature = 1.;
static volatile float temperatureA = 1.;
static volatile float percent = 0;
static volatile uint8_t read = 0;
static volatile uint8_t CC_ITpass = 0;
static volatile uint16_t CCMSB[2] = {0, 0};
static volatile uint16_t LSB;
static volatile uint8_t dataProcess = 0;
static volatile uint8_t garbageData = 0;
static volatile uint8_t american = 0;
static uint16_t MSB = 0;
static uint32_t distance32_raw = 0;
static float mean = 0;
static float distance32_raw_mean = 0.;
static float distance = 0;
static float distanceA = 0;
static float distance1 = 0;
static float percentDst = 0;
//static uint32_t queue[NUMBER_OF_MEANS]
static float queue[NUMBER_OF_MEANS];
static const float speed_of_sound[16]  = {306.2f, 331.4f, 334.4f, 337.4f, 340.4f, 343.3f, 346.3f, 349.1f, 354.7f, 360.3f, 365.7f, 371.2f, 376.6f, 381.7f, 386.9f, 434.5f};
static const float speed_of_soundT[16] = {  -40.,     0.,     5.,    10.,    15.,    20.,    25.,    30.,    40.,    50.,    60.,    70.,    80.,    90.,   100.,   200.};
static float currentSoS = 0;
static float currentSoSA = 0;
/*--------------------------------------------------------------------------------------------*/
int main()
{
	//set up MCU clock to 48Mhz
	clkConfig();
	//Initializes all GPIO pins needed
	GPIOinit();
	//Initializes I2C
	I2Cinit();
	//Timer 7 is used to initiate temperature reading from sensor via I2C
	Timer7Init();
	//Timer 6 is used to give sensor time to stabilize
	Timer6Init();
	//Timer1 is configured as PWM and is used to drive servo motor
	Timer1Init();
	USART3Init();
	Timer25ROPMInit();
	Timer34EchoInit();
	buttonTim13Init();
	
	//initalizeQueue();
	const float feets_in_meter = (float)3.2808399;
	
	while(1){
		if (read) {
			readTemp();
			read = 0;
		}
		percent = (((temperature > (float)TEMP_MIN) ? temperature : (float)TEMP_MIN) - (float)TEMP_MIN)/((float)(TEMP_MAX - TEMP_MIN));
		percent = percent > 1.f ? 1.f : percent;
		percentDst = (((distance > DISTANCE_MIN) ? distance : DISTANCE_MIN) - DISTANCE_MIN)/(DISTANCE_MAX - DISTANCE_MIN);
		percentDst = percentDst > 1.f ? 1.f : percentDst;
		#define TEMP
		#ifndef TEMP
			#define DST
		#endif
		#ifdef TEMP
			TIM1->CCR2 = PWM_MIN + (uint32_t)(percent*(float)(PWM_MAX - PWM_MIN));
		#endif 
		#ifdef DST
				TIM1->CCR2 = PWM_MIN + (uint32_t)(percentDst*(float)(PWM_MAX - PWM_MIN));
		#endif
		
		if (dataProcess) {
			if (CCMSB[0] < CCMSB[1]) 
				MSB = (CCMSB[1] - CCMSB[0]) - 1;
			else
				MSB = 0xFFFFU - (CCMSB[0] - CCMSB[1]) - 1;
			dataProcess = 0;
			distance32_raw = ((uint32_t)MSB << 16) | (uint32_t)LSB;
			distance32_raw_mean = queueProcess(distance32_raw);
			calcSoS();
			distance = ((distance32_raw_mean/TIMER4_CLOCK)*currentSoS)/2.f;
			distance1 = (((float)distance32_raw/TIMER4_CLOCK)*currentSoS)/2.f;
			distanceA = distance*feets_in_meter;
			temperatureA = temperature*1.8f + 32.f;
			currentSoSA = currentSoS*feets_in_meter;
		}
		#define DEBUG
		#ifdef DEBUG
			if (american) {
					printf("T=%4.2fF SoS=%3.1fft/s D=%1.2fft \r", temperatureA, currentSoSA, distanceA);
			}
			else
					printf("T=%4.2fC SoS=%3.1fm/s D=%1.2fm \r", temperature, currentSoS, distance);
		#endif
		#ifndef DEBUG
			USART3Transmit((uint8_t *)&temperature);
		#endif
	}
	
}
/*--------------------------------------------------------------------------------------------*/
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
	(void)f;
  USART3Transmit((uint8_t *)&ch);

  return ch;
}
/*--------------------------------------------------------------------------------------------*/
void clkConfig(void)
{
	uint32_t temp;
	
	//Enable clock to power interface
	/*RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	temp = RCC->APB1ENR & RCC_APB1ENR_PWREN;
	//setup power scaling to SCALING 3 (0x1), used for optimizing power consumption
	//flash memory latency maybe depends on voltage scaling
	//can only be modified while PLL is off
	//probably not needed
	temp = PWR->CR1 & ~PWR_CR1_VOS;
	PWR->CR1 = temp | POWER_SCALING_3;*/
	
	//activate HSE clock, clock is provided by ST-LINK, which uses 8MHz crystal oscillator
	RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	//wait until HSE is satbilized and ready
	while (!(RCC->CR & RCC_CR_HSERDY)){}
	
	//configure PLL for 36MHz, HSE is selected for clock source, M=4, N=72, P=4, ((8/4)*72)/4=36	
	temp = RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC);
	RCC->PLLCFGR = temp | PLL_M_4 | PLL_N_72 | PLL_P_4 | PLL_SOURCE_HSE;
	//PLL is activated, then program is waiting until PLL is stabilized and ready	
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)){}
	
	//beacuse flash memory is slow, in order for read and write operations to work correctly, wait cycles are introduced and depend on HCLK
	//since our desired HCLK is 36MHz , according to table 5. in reference manual we need 1 wait cycle
	//this must be done before we configure PLL to be SYSCLK source
	FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
	//waiting until new latency has taken effect	
	if ((FLASH->ACR & FLASH_ACR_LATENCY_1WS) != FLASH_ACR_LATENCY_1WS)
		while(1){}
	
  //AHB prescaler default(1), APB1 prescaler is 4, APB2 perscaler is 2, thus making HCLK=SYSCLK=36MHz,
	//PCLK1 9MHZ except for timers on APB1 bus that get 18MHz, PCLK2 18MHZ except timers on APB2 bus that get 36MHz			
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 ;
	//PLL is selected as SYSCLK source		
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}
	
	//disable internal oscillator and waint for stabilization and confirmation
	RCC->CR &= ~RCC_CR_HSION;
	while (RCC->CR & RCC_CR_HSIRDY){}
}
/*--------------------------------------------------------------------------------------------*/
void GPIOinit(void)
{
	uint32_t temp;
	//GPIOF is clocked
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	temp = RCC->AHB1ENR & RCC_AHB1ENR_GPIOFEN;
	
	//F0 and F1 pins are used for alternate functions
	GPIOF->MODER |= GPIOF0_ALTERNATE_FUNCTION |  GPIOF1_ALTERNATE_FUNCTION/* | GPIOF2_ALTERNATE_FUNCTION*/;
	//F0 and F1 pins are configures as open drain, this is needed for I2C
	GPIOF->OTYPER |= GPIOF0_OPEN_DRAIN | GPIOF1_OPEN_DRAIN/* | GPIOF2_OPEN_DRAIN*/;
	//F0 and F1 pins configureds to maximum speed - not sure that this is needed
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1/* | GPIO_OSPEEDER_OSPEEDR2*/;
	//GPIOF->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0/* | GPIO_PUPDR_PUPDR2_0*/;
	//F0 and F1 pins configured alternate function to be respectivly, I2C serial data line, and I2C serial clock line 
	GPIOF->AFR[0] |= GPIOF0_AF_I2C2_SDA | GPIOF1_AF_I2C2_SCL/* | GPIOF0_AF_I2C2_SMBA*/;
}	
/*--------------------------------------------------------------------------------------------*/
void I2Cinit(void)
{
	//enable clock to I2C on APB1 bus
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	//disable I2C, so I2C timings can be configured
	I2C2->CR1 &= ~I2C_CR1_PE;
	
	//CUBE MX must be used to generate this number, configurations used is: speed 100Khz, rise time 100ns, fall time 20ns
	I2C2->TIMINGR = 0x20100B0FU;
	//I2C2->TIMINGR = 0x00C0252CU;
	//I2C2->CR1 |= I2C_CR1_ANFOFF | (0x1U << I2C_CR1_DNF_Pos);
	//enable I2C, after new timings configuration
	I2C2->CR1 |= I2C_CR1_PE;
}
/*--------------------------------------------------------------------------------------------*/
void Timer7Init(void)
{
	//Timer 7 used to tell MCU when it is time to read from temperature sensor
	//Sensor is configuired to have measurments ready at frequency of 25Hz
	//Timer 7 will sent interrupt at frequency of 25Hz and initiate reading from temperature sensor
	
	uint32_t temp;
	
	//enable clock to Timer 7 on APB1 bus
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	temp = RCC->APB1ENR & RCC_APB1ENR_TIM7EN;
	//configuring Timer 7 prescaler and counting register
	TIM7->PSC = 719U;
	TIM7->ARR = 1000U;
	
	//setup B7 and B14 pins as output
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	temp = RCC->APB1ENR & RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER14_0;
	//GPIOB->ODR |= GPIO_ODR_OD14;
	
	//Enables Timer7 interrupt at NVIC
	__NVIC_EnableIRQ(TIM7_IRQn);
	//Enable interrupt in Timer7 on update event(overflow value is reached by counter)
	TIM7->DIER |= TIM_DIER_UIE;
	//TIM4->CR1 |= TIM_CR1_CEN;
}
/*--------------------------------------------------------------------------------------------*/
void Timer6Init(void)
{
	//Timer6 is used to give time to sensor to stabilize, temperature sensor requires 12ms to stabilaze and be ready for usage
	//Timer6 should be counting up to 216000, at frequency 18Mhz, this gives 12ms
	
	//Enables clock for Timer6 on APB1 bus
	uint32_t temp;
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	temp = RCC->APB1ENR & RCC_APB1ENR_TIM6EN;
	//configure Timer6 prescaler and auto reload registers
	TIM6->PSC = 215;
	TIM6->ARR = 1000U;
	
	//Enables Timer6 interrupt and NVIC
	__NVIC_EnableIRQ(TIM6_DAC_IRQn);
	//Enable Timer6 interrupt on update event
	TIM6->DIER |= TIM_DIER_UIE;
	//Start Timer6
	TIM6->CR1 |= TIM_CR1_CEN;
}
/*--------------------------------------------------------------------------------------------*/
void Timer1Init(void)
{
	//Timer1 is configured as PWM and it can drive servo motor
	
	//enable Timer1 clock, timer PCLK2 = 36MHz
	uint32_t temp;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	temp = RCC->APB2ENR & RCC_APB2ENR_TIM1EN;
	
	//prescaler and autoreload register config, 50Hz or 20ms
	TIM1->PSC = 719U;
	TIM1->ARR = 1000U;
	//capture control set to 0.5ms, wich gives -90 degrees on servo
	TIM1->CCR2 = PWM_MIN;
	//output compare preload enable, PWM mode 1 (1, cnt<ccr2)
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	//Auto reload preload enable
	TIM1->CR1 |= TIM_CR1_ARPE;
	//TIM1->EGR |= TIM_EGR_UG;
	
	//capture compare complementary output enable - cant rember why i needed complemetary output
	TIM1->CCER |= TIM_CCER_CC2NE;
	//OC and OCN are enabled if CCxE and CCxNE bits are set in CCER register
	TIM1->BDTR |= TIM_BDTR_MOE;
	//generate update event
	TIM1->EGR |= TIM_EGR_UG;
	
	//configure B0 pin as alternate function of Timer1
	GPIOB->MODER |= GPIO_MODER_MODER0_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL0_0;
	//Start timer
	TIM1->CR1 |= TIM_CR1_CEN;
}
/*------++------------------------------------------------------------------------------------*/
void USART3Init(void)
{
	RCC->DCKCFGR2 |= RCC_DCKCFGR2_USART3SEL_0;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	USART3->BRR = 0x270U;
	USART3->CR1 |= USART_CR1_OVER8;
	USART3->CR1 |= USART_CR1_UE;
	USART3->CR1 |= USART_CR1_TE;
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= GPIO_MODER_MODER8_1;
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH0_1 | GPIO_AFRH_AFRH0_0;
}
/*-+++++--------------------------------------------------------------------------------------*/
void Timer25ROPMInit (void)
{
	//Timer 2 generates pusle that starts ultrasonic sensor
	
	//enabe clock for Timer2 and Timer5 ftom APB1, 18Mhz
	uint32_t temp;
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM2EN;
	temp = RCC->APB1ENR & (RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM2EN);
	//TIM5->PSC = 17999;
	TIM5->ARR = 180U;
	//TIM5->CCR1 = 500U;
	//capture compare 1 is confugered with output compare preload enable, and retriggerable one pulse mode 2
	TIM5->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_0;
	//auto reload preload enable and counter stops counting at next update event
	TIM5->CR1 |= TIM_CR1_ARPE | TIM_CR1_OPM;
	//generate update event
	TIM5->EGR |= TIM_EGR_UG;
	//capture comapre 1 output enable
	TIM5->CCER |= TIM_CCER_CC1E;
	//combined reset and trigger on rising edge of selected trigger, internal trigger
	TIM5->SMCR |= TIM_SMCR_SMS_3;
	//TIM5->DIER |= TIM_DIER_UIE;
	//__NVIC_EnableIRQ(TIM5_IRQn);
	
	//Timer2 is used as master to trigger Timer5, its slave
	
	//timer counts to 720 000, or with frequency of 25Hz
	TIM2->PSC = 999U;
	TIM2->ARR = 720U;
	//Timer2 is used in master mode to sent update event to slave timer, Timer5
	TIM2->CR2 |= TIM_CR2_MMS_1;
	//this bit synchronizes Timer2 with its slave Timer5
	TIM2->SMCR |= TIM_SMCR_MSM;
	//Start Timer2
	TIM2->CR1 |= TIM_CR1_CEN;
	//TIM2->DIER |= TIM_DIER_UIE;
	//__NVIC_EnableIRQ(TIM2_IRQn);
	
	//enable clock to GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	temp = RCC->APB1ENR & RCC_AHB1ENR_GPIOAEN;	
	//pin A0 configured as alternate function of Timer5
	GPIOA->MODER |= GPIO_MODER_MODER0_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL0_1;
	//Start Timer5
	TIM5->CR1 |= TIM_CR1_CEN;
	
	//Reason why we need two timer is because one timer cannot make long enough pause between pusles, cause its limited 16 registers
}
/*--------------------------------------------------------------------------------------------*/
void Timer34EchoInit (void)
{
	GPIOA->MODER |= GPIO_MODER_MODER6_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_1;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL6_1;	
	
	uint32_t temp;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	temp = RCC->APB1ENR & (RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN);
	
	TIM3->ARR = 0xFFFFU;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM3->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP;
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0 | TIM_SMCR_MSM;
	TIM3->SMCR |= TIM_SMCR_SMS_2;
	TIM3->CR2 |= TIM_CR2_MMS_1;
	//TIM3->DIER |= TIM_DIER_CC1IE;
	//__NVIC_EnableIRQ(TIM3_IRQn);
	
	//TIM4->PSC = 0xFFFFU;
	TIM4->ARR = 0xFFFFU;
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM4->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->SMCR |= TIM_SMCR_TS_1 | TIM_SMCR_MSM;
	TIM4->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
	TIM4->DIER |= TIM_DIER_CC1IE;
	__NVIC_EnableIRQ(TIM4_IRQn);
	
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;
}
/*--------------------------------------------------------------------------------------------*/
void buttonTim13Init(void)
{
	uint32_t temp;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	temp = RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	temp = RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
	temp = RCC->APB1ENR & RCC_APB1ENR_TIM13EN;
	(void)temp;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER13;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->RTSR |= EXTI_RTSR_TR13;
	EXTI->IMR |= EXTI_IMR_IM13;
	__NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	TIM13->PSC = 999U;
	TIM13->ARR = 540U;
	TIM13->CR1 |= TIM_CR1_OPM;
	__NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}
/*--------------------------------------------------------------------------------------------*/
//#define CHECK
void configSensor(void)
{	
#ifdef CHECK		
	uint8_t temp;
#endif
	//configure I2C to send 2 bytes to sensor, and automaticly generate stop condition
	I2C2->CR2 |= STTS22H_ADDRESS | (0x2U << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
	//generate start condition, or start I2C transmission
	I2C2->CR2 |= I2C_CR2_START;
	//sending sensor internal adress, or adress where writing is needed
	I2C2->TXDR = STTS22H_SOFTWARE_RESET;
	//waiting until I2C is ready to send again
	while(!(I2C2->ISR & I2C_ISR_TXIS)){}
	//send information that needs to be writen, in this case, reseting all digital blocks
	I2C2->TXDR = SW_RESET;
	//waint until stop condition is detected
	while (!(I2C2->ISR & I2C_ISR_STOPF)){}
	//clearing stop flag
	I2C2->ICR |= I2C_ICR_STOPCF;
	//reseting I2C CR2 register
	I2C2->CR2 = 0x0U;
	/**/	
	//this blck was used for debugging purpose, in order to check if sent data was received	
#ifdef CHECK		
	I2C2->CR2 |= STTS22H_ADDRESS | (0x1U << I2C_CR2_NBYTES_Pos);
	I2C2->TXDR = STTS22H_SOFTWARE_RESET;
	I2C2->CR2 |= I2C_CR2_START;
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	//I2C2->CR2 &= ~I2C_CR2_NBYTES;
	I2C2->CR2 |=  /*(0x2U << I2C_CR2_NBYTES_Pos) | */I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	while (!(I2C2->ISR & I2C_ISR_RXNE)){}
	temp = (uint8_t)(I2C2->RXDR);
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	I2C2->CR2 |= I2C_CR2_STOP;
	while (I2C2->CR2 & I2C_CR2_STOP){}
	I2C2->CR2 = 0x0U;   
#endif		
		
	/**/
	//configuring transmission, address of temperature sensor, 2 bytes of data, automatic generation of stop condition
	I2C2->CR2 |= STTS22H_ADDRESS | (0x2U << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
	// first byte being sent represents adress on internal register, where we need to writer data, or STTS22H_SOFTWARE_RESET register
	I2C2->TXDR = STTS22H_SOFTWARE_RESET;
	//generate I2C start condition, and start transmission
	I2C2->CR2 |= I2C_CR2_START;
	//waint until I2C is ready to send new data
	while(!(I2C2->ISR & I2C_ISR_TXIS)){}
	//clear sensor internal STTS22H_SOFTWARE_RESET register
	I2C2->TXDR = 0x0U;
	//wait until stop condition is detected
	while (!(I2C2->ISR & I2C_ISR_STOPF)){}
	//clear stop flag
	I2C2->ICR |= I2C_ICR_STOPCF;
  //reset CR2 register
	I2C2->CR2 = 0x0U;
	/**/
#ifdef CHECK		
	I2C2->CR2 |= STTS22H_ADDRESS | (0x1U << I2C_CR2_NBYTES_Pos);
	I2C2->TXDR = STTS22H_SOFTWARE_RESET;
	I2C2->CR2 |= I2C_CR2_START;
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	//I2C2->CR2 &= ~I2C_CR2_NBYTES;
	I2C2->CR2 |=  /*(0x2U << I2C_CR2_NBYTES_Pos) | */I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	while (!(I2C2->ISR & I2C_ISR_RXNE)){}
	temp = (uint8_t)(I2C2->RXDR);
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	I2C2->CR2 |= I2C_CR2_STOP;
	while (I2C2->CR2 & I2C_CR2_STOP){}
	I2C2->CR2 = 0x0U;   
#endif		
	/**/	
	//configuring transmission, address of temperature sensor, 2 bytes of data, automatic generation of stop condition
	I2C2->CR2 |= STTS22H_ADDRESS | (0x2U << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;
	// first byte being sent represents adress on internal register, where we need to writer data, or STTS22H_CTRL register
	I2C2->TXDR = STTS22H_CTRL;
	//generate I2C start condition, and start transmission
	I2C2->CR2 |= I2C_CR2_START;
	//waint until I2C is ready to send new data
	while(!(I2C2->ISR & I2C_ISR_TXIS)){}
	//configure sensor to send average of 8 acquisitions, at frequency of 25Hz, configure sensor to autoicrement address in case of multiple byte readings
	I2C2->TXDR = STTS22G_CONFIG;
	//wait until stop condition is detected
	while (!(I2C2->ISR & I2C_ISR_STOPF)){}
	//clear stop flag
	I2C2->ICR |= I2C_ICR_STOPCF;
	//reset CR2 register
	I2C2->CR2 = 0x0U;
	/**/
#ifdef CHECK		
	I2C2->CR2 |= STTS22H_ADDRESS | (0x1U << I2C_CR2_NBYTES_Pos);
	I2C2->TXDR = STTS22H_CTRL;
	I2C2->CR2 |= I2C_CR2_START;
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	//I2C2->CR2 &= ~I2C_CR2_NBYTES;
	I2C2->CR2 |=  /*(0x2U << I2C_CR2_NBYTES_Pos) | */I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	while (!(I2C2->ISR & I2C_ISR_RXNE)){}
	temp = (uint8_t)(I2C2->RXDR);
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	I2C2->CR2 |= I2C_CR2_STOP;
	while (I2C2->CR2 & I2C_CR2_STOP){}
	I2C2->CR2 = 0x0U;   
#endif		
}
/*--------------------------------------------------------------------------------------------*/
void readTemp(void)
{	
	//configure I2C2 with adress of the sensor and 1 bytes transmission, but no automatic stop generation
	I2C2->CR2 |= STTS22H_ADDRESS | (0x1U << I2C_CR2_NBYTES_Pos);
	//generate start condition, start transmission
	I2C2->CR2 |= I2C_CR2_START;
	//wait until I2C is ready to send
	while(!(I2C2->ISR & I2C_ISR_TXIS)){}
	//send adress on internal sensor register, where lower byte of read temperature is stored
	I2C2->TXDR = STTS22H_TEMP_L_OUT;
	//wait until transfer is complete
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	//reset number of bytes configuration
	I2C2->CR2 &= ~I2C_CR2_NBYTES;
  //configure new transmission, same sensor address as before, 2 bytes transmission, reading , stop condition is automaticly generated
	I2C2->CR2 |=  (0x2U << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND;
	//generate start condition
	I2C2->CR2 |= I2C_CR2_START;
	//wait until data is received and ready to be read	
	while (!(I2C2->ISR & I2C_ISR_RXNE)){}
	//read temperature lower byte
	temperature_reg = (uint16_t)(I2C2->RXDR);
	//wait until data is received and ready to be read
	while (!(I2C2->ISR & I2C_ISR_RXNE)){}
	//read temperature higher byte and combine lowere and highher byte into complete value	
	temperature_reg |= (uint16_t)(I2C2->RXDR << (uint8_t)8);
	//while(!(I2C2->ISR & I2C_ISR_TC)){}
	//I2C2->CR2 |= I2C_CR2_STOP;
	//wait until stop condition is detected
	while (!(I2C2->ISR & I2C_ISR_STOPF)){}
	//clear stop flag
	I2C2->ICR |= I2C_ICR_STOPCF;
	//reset transmission configuiration
	I2C2->CR2 = 0x0U;
	
  //convert binary war temperature value into real value, two complement is used,
	//calculated values also need to be divided by 100 , because LSB shows 0.01C degrees
	if (temperature_reg & (uint16_t)0x8000)
		temperature = -((float)((~temperature_reg) + 1))/100.0f;
	else
		temperature = ((float)(temperature_reg & (uint16_t)0x7FFF))/100.0f;
}

/*--------------------------------------------------------------------------------------------*/
void USART3Transmit(uint8_t *pData)
{
	uint8_t count = sizeof(pData);
	while (count > 0) {
		while (!(USART3->ISR & USART_ISR_TXE)) {}
		USART3->TDR = (uint8_t)(*(pData++) & 0xFFU);
		--count;
	}
	#ifndef DEBUG
	while (!(USART3->ISR & USART_ISR_TXE)) {}
	USART3->TDR = (uint8_t)0x0DU & 0xFFU;
	/*while (!(USART3->ISR & USART_ISR_TXE)) {}
	USART3->TDR = (uint8_t)0x0CU & 0xFFU;*/
	#endif
	while (!(USART3->ISR & USART_ISR_TC)) {}
}
/*--------------------------------------------------------------------------------------------*/
void initalizeQueue()
{
	uint8_t index = NUMBER_OF_MEANS;
	do {
		queue[--index] = 0.f;
	} while (index > 0);
}
/*--------------------------------------------------------------------------------------------*/
float queueProcess(uint32_t element)
{
	//queue is array of NUMBER_OF_MEANS elements(all of wich are 0 at start), and it behaves like FIFO buffer
	//queue[0] is first element, and queue[last] is last element of FIFO buffer
	
	//This function, returnes average of last NUMBER_OF_MEANS elements given by sensor, in such a way that,
	//every time, sensor send new value, we calculate new average. Main advatage of this approach, is that program doesnt wait 
	//for sensor to send NUMBER_OF_MEANS readings before new average can be calculated, instead every time sensor sends new reading
	//we just modify current average value("mean"), by removing first element of the FIFO buffer(queue) from the average, and adding
	//new element to last FIFO buffer(queue) place and current average(mean).
	
	//last is index of first lelement in FIFO buffer
	uint8_t last = NUMBER_OF_MEANS - 1;
	
	//mean represents average value of the FIFO buffer, and it is zero in the begining
	//mean vaule is decreased by last element of queue divided by FIFO elngth
	//mean -= (float)queue[0]/(float)NUMBER_OF_MEANS;
	mean -= (float)queue[0];
	
	//all elements of FIFO buffer are moved 1 spot
	//point of this is to, always know the values of all elements, which are being sumed and averaged
  //especialy so we know, which elements is being removed form the sum	
	for (uint8_t i = 0; i < last; i++){
		queue[i] = queue[i + 1];
	}
	
	//new value is added to the last spot in FIFO buffer
	//queue[last] = element;
	queue[last] = (float)element/(float)NUMBER_OF_MEANS;
	
	//average value of last NUMBER_OF_MEANS elements is increased by
	//mean += (float)element/(float)NUMBER_OF_MEANS;
	mean += queue[last];
	return mean;
}
/*--------------------------------------------------------------------------------------------*/
void calcSoS(void)
{
	uint8_t index = 16;
	float percentT = 0;
	
	while (temperature < speed_of_soundT[--index]) { }
	if (index == 15) 
		--index;
	percentT = (temperature - speed_of_soundT[index])/(speed_of_soundT[index + 1] - speed_of_soundT[index]);
	currentSoS = speed_of_sound[index] + percentT*(speed_of_sound[index + 1] - speed_of_sound[index]);
}
/*--------------------------------------------------------------------------------------------*/
void TIM7_IRQHandler(void)
{
	//clearing interrupt flag
	TIM7->SR &= ~TIM_SR_UIF;
	//signal that it is rime to read from temperature sensor via I2C
  read = 1;
	//Blue Led is tunred ON/OFF, if it was OFF/ON before, used for debuging purposes
	GPIOB->ODR ^= GPIO_ODR_OD7;
}
/*--------------------------------------------------------------------------------------------*/
void TIM6_DAC_IRQHandler(void)
{
	//clear Timer6 interrupt flag
	TIM6->SR &= ~TIM_SR_UIF;
	//stops Timer6
	TIM6->CR1 &= ~TIM_CR1_CEN;
	//configure sensor
	configSensor();
	//Start Timer7 -- on second thought Timer6 should only be starting here, to give sensor time to stabilize after receiving new configuration settings, but it seems to be working anyway
	TIM7->CR1 |= TIM_CR1_CEN;
	//disable interrupts from Timer6 at NVIC
	__NVIC_DisableIRQ(TIM6_DAC_IRQn);
	//disable clock for Timer6
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
}
/*--------------------------------------------------------------------------------------------*/
/*void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF;
}*/
/*--------------------------------------------------------------------------------------------*/
/*void TIM5_IRQHandler(void)
{
	TIM5->SR &= ~TIM_SR_UIF;
}*/
/*--------------------------------------------------------------------------------------------*/
void TIM4_IRQHandler(void)
{
	GPIOB->ODR ^= GPIO_ODR_OD14;
	TIM4->SR &= ~TIM_SR_CC1IF;
	#define ATOMIC	
	#ifndef ATOMIC
		CCMSB[CC_ITpass++] = (uint16_t)TIM4->CCR1;
		if (CC_ITpass == CC_INTERUPT_PASSES) {
			TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
			LSB = (uint16_t)TIM3->CCR1;
			CC_ITpass = 0;
			dataProcess = 1;
		}
		else {
			TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
			TIM4->CCER |= TIM_CCER_CC1P;
		}
	#endif
	#ifdef ATOMIC
		if (CC_ITpass == 1) {
			TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
			if (!(garbageData)) {
				LSB = (uint16_t)TIM3->CCR1;
				CCMSB[CC_ITpass] = (uint16_t)TIM4->CCR1;
				dataProcess = 1;
			}
			CC_ITpass = 0;
			garbageData = 0;
		}
		else {
			if (!(dataProcess)) {
				CCMSB[CC_ITpass] = (uint16_t)TIM4->CCR1;
				TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
				TIM4->CCER |= TIM_CCER_CC1P;
			}
			else
				garbageData = 1;
			++CC_ITpass;
		}
	#endif
}
/*--------------------------------------------------------------------------------------------*/
void EXTI15_10_IRQHandler(void)
{
	EXTI->PR |= EXTI_PR_PR13;
	EXTI->IMR &= ~EXTI_IMR_MR13;
	TIM13->DIER |= TIM_DIER_UIE;
	TIM13->CR1 |= TIM_CR1_CEN;
}
/*--------------------------------------------------------------------------------------------*/
void TIM8_UP_TIM13_IRQHandler(void)
{
	TIM13->SR &= ~TIM_SR_UIF;
	if (GPIOC->IDR & GPIO_IDR_ID13)
		american ^= (uint8_t)1;
	EXTI->IMR |= EXTI_IMR_IM13;
}
