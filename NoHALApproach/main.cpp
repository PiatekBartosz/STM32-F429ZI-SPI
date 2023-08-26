#include "stm32f429xx.h"

void initClocks(void);
void initSPI(void);
void setPinMode(void);
void setAlternateFunction(void);
void configSPI(void);
uint16_t transferFunction(uint16_t tx_data);
void initTim2(void);
void delay(unsigned int ms);

int main(void){
	uint16_t test_display_data = (0x0f << 8) | 1u; 
	
	uint16_t rxd = 0;
	
	initTim2();
	
	initSPI();
	
	
	
	
	return 0;
}

/* 
	About bit operations:
		1u, 2u, 4u and so on means that they are UNSIGNED INT
		By applying bit shift operation to these integers we create masks

		1u -> 0b01
		2u -> 0b10
		3u -> 0b11
		5u -> 0b0101
		7u -> 0b0111
		15u -> 0b1111

		for >= C++14 we can use 0b11 insted of 3u (integer literals)

*/
void initClocks(void){
	// Init peripheral clocks used by SPI1

	RCC->AHB1ENR |= ((1u << 1)			//ENABLE GPIOA CLOCK
                  |(1u << 0)			//ENABLE GPIOB CLOCK
                  );
    
  RCC->APB2ENR |= (1u << 12);			//ENABLE SPI1 CLOCK
}

void initSPI(void){
	initClocks();
	
	configSPI();
	
	//setPinMode()
}

void setPinMode(void){
	// Reset Pin Modes for GPIOA & GPIOB (set to 0b00)
	
	GPIOA->MODER &= ~(3u << (2 * 4));		// Reset PA4

	GPIOB->MODER &= ~((3u << (2 * 3))			// Reset PB3
									|(3u << (2 * 4))			// Reset PB4
									|(3u << (2 * 5))			// Reset PB5
									);
	
	// Set Pin Modes for GPIOA & GPIOB to alternate function mode (0b10)
	GPIOA->MODER |= (2u << (2 * 4));		// Set PA4

	GPIOB->MODER |= ((2u << (2 * 3))			// Set PB3
									|(2u << (2 * 4))			// Set PB4
									|(2u << (2 * 5))			// Set PB5
									);
}

void setAlternateFunction(void){
	// we use AFR[0] for pins <0, 7> 
	
	// Reset PIN Alternative function
	GPIOA->AFR[0] &= ~(15u << (2 * 4));			// Reset PA4

	GPIOB->AFR[0] &= ~((15u << (2 * 3))			// Reset PB3
										|(15u << (2 * 4))			// Reset PB4
										|(15u << (2 * 5))			// Reset PB5
										);
	
	// Set PIN Alternative function to SPI1/SPI2
	GPIOA->AFR[0] |= (5u << (2 * 4));		// Set PA4

	GPIOB->AFR[0] |= ((5u << (2 * 3))			// Set PB3
									|(5u << (2 * 4))			// Set PB4
									|(5u << (2 * 5))			// Set PB5
									);
}

void configSPI(void){
	// Configure SPI1_CR1 Register
	// Set bits to 0
	SPI1->CR1 &= ~((1u << 15)			// Full duplex
								|(1u << 13)			// Disable CRC Calculations
								|(1u << 10)			// Disable recieve only
								|(1u << 9)			// Hardware slave managment
								|(1u << 7)			// MSB first
								|(7u << 3)			// Reset bits that will be later set for frequency div
								|(1u << 1)			// Set clock polarity 0 -> CK to 0 when idle
								|(1u << 0)			// Set clock phase 0 -> first clock transistion is the first data catpure
								);
	
	// Set bits to 1
	SPI1->CR1 |= ((1u << 11)			// Set 16-bit received data
								|(5u << 3)			// Set SPICLK to system_clk / 64
								|(1u << 2)			// Set master mode
								);
	
	// Cofigures SPI1_CR2 Register
	// Set bits to 0
	SPI1->CR2 &= ~((1u << 7)			// No Tx buffer empty interrupt
								|(1u << 6)			// No Rx buffer empty interrupt
								|(1u << 5)			// No Error interrupt
								|(1u << 4)			// Motorola frame format
								|(1u << 1)			// No DMA Tx buffer
								|(1u << 0)			// No DMA Rx buffer
								);
	
	// Set bits to 1
	SPI1->CR2 |= (1u << 2);			// No multimaster configuration
}

uint16_t transferFunction(uint16_t tx_data){
	// This function uses status and data SPI registers
	
	uint8_t rx_data = 0;
	
	// Enable SPI 
	SPI1->CR1 |= (1u << 6);
	
	// Write dtat to register with additional 8 empty dummy bytes
	SPI1->DR = (uint16_t) tx_data;
	
	// Wait until send (checks BUSY and RX not empty respectively)
	while ( ((SPI1->SR) & (1u << 7) | (!(SP1->SR) & (1u << 0))) );
	
	// Read from buffer TODO: for now do not ignore dummy bits
	rx_data = SPI1->DR;
	
	// Disable SPI
	SPI1->CR1 &= ~(1u << 6);
	
	return rx_data;
}

void initTim2(void) 
{
    //ENABLE TIM2 CLOCK
    RCC->APB1ENR1 |= (1u << 0);
    
    //LEAVE THE COUNTER FREQUENCY UNCHANGED
    TIM2->PSC = 0;
    
    //SET TIMER RELOAD VALUE
    TIM2->ARR = (uint32_t)4000000;
    
    //SET INITIAL COUNTER VALUE
    TIM2->CNT = 0;
    
    //ENABLE TIM2 COUNTER
    TIM2->CR1 |= (1u << 0);
}

void delay(unsigned int ms)
{
    /*HOLDS THE TOTAL COUNT FROM TM2_CNT REGISTER TO RECORD HOW
    MUCH TIME ELAPSED.*/
    unsigned int counter = 0;
    //HOLDS THE COUNT FOR THE COUNTER TO COUNT TO.
    unsigned int goalCount = ms * 4000u;
    //HOLDS THE MOST RECENT VALUE OBTAINED FROM TIM2_CNT.
    unsigned int currentCntVal = 0;
    //HOLDS THE PREVIOUS VALUE OBTAINED FROM TIM2_CNT.
    unsigned int prevCntVal = 0;
    //HOLDS RESULT OF CALCULATION BETWEEN CURRENT AND PREVIOUS COUNTS.
    unsigned int countToAdd = 0;
    
    //GET INITIAL VALUE OF CNT
    prevCntVal = TIM2->CNT; 
    
    //LOOP UNTIL COUNTER IS EQUAL OR EXCEED GOAL COUNT
    while(counter < goalCount)
    {
        //GET NEWEST COUNT
        currentCntVal = TIM2->CNT;
        
        //HANDLE SITUATION WHERE TIM2_CNT RESET
        if(currentCntVal < prevCntVal)
        {
            //GET THE COUNT BEFORE THE CNT REGISTER RESET AND THEN 
            //ADD THE COUNT AFTER IT RESET TO GET ELAPSED COUNT
            countToAdd = (4000000 - prevCntVal) + currentCntVal;
        }
        else
        {
            //SSUBTRACT CURRENT COUNT FROM PREVIOUS COUNT TO GET
            //ELAPSED COUNT
            countToAdd = currentCntVal - prevCntVal;
        }
        
        //ADD ELAPSED COUNT TO THE COUNTER
        counter += countToAdd;
        
        //CURRENT COUNT NOW BECOMES PREVIOUS COUNT
        prevCntVal = currentCntVal;
    }
}
