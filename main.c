//MCU STM32F042
// SPI Pinout   PA7- MOSI, PA6 - MISO, PA5 - SCK, PA4- SCN, PA3 - Chip Enable (CE) , PA0 - IRQ
// I2C LCD Pinout  - PB6 - SCL, PB7 - SDA



#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"
#include "RC522.h"


#define I2C_400				1		//		400 kHz i2C Freq
#define I2C_GPIOB			1	//		i2C PB6,PB7


uint8_t i,k,test=0;
uint8_t IRQ_Flag,sec_tic;
uint16_t TimingDelay,led_count,ms1000;

//=====================RC522==================================
uint8_t blockAddr;
uint8_t RC_size;

uint8_t sectorKeyA[16][16] = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},};

uint8_t status=MI_ERR;
uint8_t	str[MFRC522_MAX_LEN];
uint8_t sn[4];
char	buff[64];		
//=================END of RC522==============================

void TimingDelayDec(void) 																													{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!ms1000) {ms1000=1000;sec_tic=1;}
 led_count--;ms1000--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}
void EXTI0_1_IRQHandler(void)																												{
  if(EXTI->PR & EXTI_PR_PIF0)	{
			EXTI->PR |= EXTI_PR_PIF0;
			IRQ_Flag=1;
	}
}

void initial (void)																																{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
//------------I2C1 GPIOB_SETTING ---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER 		|=GPIO_MODER_MODER6_1 		| GPIO_MODER_MODER7_1; 							// Alt -mode /Pb6 -SCL , Pb7- SDA
	GPIOB->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR6 	| GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->OTYPER		|=GPIO_OTYPER_OT_6 				| GPIO_OTYPER_OT_7;
	GPIOB->AFR[0] 	|=(1<<GPIO_AFRL_AFRL6_Pos) |(1<<GPIO_AFRL_AFRL7_Pos);  				// I2C - Alternative PB7, PB6

	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
//============I2C_400khz===============
	I2C1->TIMINGR |=(0x0	<<I2C_TIMINGR_PRESC_Pos); 	//400 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x9	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLDEL_Pos);

	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
	
//------------------------SPI-----------------------------------
	RCC->AHBENR 		|=RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR 	|= GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 |GPIO_OSPEEDER_OSPEEDR5 \
									| GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
										
	GPIOA->MODER 		|=GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 								//Pa5..7 - Alt_mode 
	GPIOA->AFR[0] 	|=(0<<GPIO_AFRL_AFRL7_Pos) |(0<<GPIO_AFRL_AFRL6_Pos) | (0<<GPIO_AFRL_AFRL5_Pos);  // SPI - Alternative
	
	GPIOA->MODER 		|=GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0; 																			//Pa3, Pa4 - out
	
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |=SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (0<<SPI_CR1_BR_Pos); 											// if HSI8 - SpiSpeed with BR=0 - 4mHz
	SPI1->CR2 |=SPI_CR2_FRXTH;		//8 bit
	SPI1->CR1 |=SPI_CR1_SPE;
	CS_HI();
	
//----------------------EXTI-----------------------------------
	RCC->APB2ENR |=RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->FTSR |= EXTI_FTSR_TR0; //Falling
	
  NVIC_SetPriority(EXTI0_1_IRQn, 2); 
  NVIC_EnableIRQ(EXTI0_1_IRQn); 
	//EXTI->IMR |= EXTI_IMR_MR0;				//Cannot activate immediately, because need switch GD0 to Rx\Tx buf Interrupt mode
	__enable_irq ();	
} 

int main(void)
{
initial();
delay_ms (100);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (20,0);LCD_PrintStr(" TEST RC522 ",1);

EXTI->IMR |= EXTI_IMR_MR0;
RES_ON();
delay_ms(100);
RES_OFF();	
	delay_ms(100);
MFRC522_Init();
delay_ms (100);		
//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
	
	if (sec_tic) {	sec_tic=0;
	
		MFRC522_Init();
		delay_ms(20);
		
		status = MFRC522_Request(PICC_REQIDL, str);							
	  if (status == MI_OK) {
			LCD_Gotoxy (1,1); LCD_PrintStr("ID 0x",0);LCD_PrintHex(str[1],0);LCD_PrintHex(str[0],0);
		}
		else {
			LCD_Gotoxy (1,1); LCD_PrintStr("ID:      ",0);
			}
		
	  // Anti-collision, return the card's 4-byte serial number
	  LCD_Gotoxy (1,2);
		status = MFRC522_Anticoll(sn);
    if (status == MI_OK) {
			LCD_PrintStr("Sn 0x",0);LCD_PrintHex((uint32_t)((sn[0]<<24)| sn[1]<<16| sn[2]<<8 | sn[3]),0);
		
		}
		else {
			LCD_PrintStr("Sn            ",0);
		}
			
		// Election card, return capacity
	  LCD_Gotoxy (64,1); 
		RC_size = MFRC522_SelectTag(sn);
	  if (RC_size != 0) {
			LCD_PrintStr("Size ",0);LCD_PrintDec(RC_size,0);LCD_PrintStr(" kb.",0);
		}
		else {
			LCD_PrintStr("Size      ",0);
		}
			
		// Card reader
	  status = MFRC522_Auth(PICC_AUTHENT1A, 8, sectorKeyA[2], sn);	
	  if (status == MI_OK) {
			// Read data
		  status = MFRC522_Read(11, str);
			if (status == MI_OK) {
			
				for (uint8_t j=0;j<8;j++) {
					uint8_t temp_y=	5+(j/8);
					LCD_Gotoxy (16 *j,5); LCD_PrintHex(str[j],0);
				}
				for (uint8_t j=8;j<16;j++) {
					LCD_Gotoxy (16 *(j-8),6); LCD_PrintHex(str[j],0);
				}
		  }		
	  }
		else {
			for (uint8_t j=0;j<16;j++) {
				
					LCD_ClearStr(5,2);
					
				}
		}
		
		delay_ms(2);
		
		MFRC522_Halt();	
		MFRC522_AntennaOff();

    
if (IRQ_Flag)	{	//RX_Mode
	}

 
}

} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
