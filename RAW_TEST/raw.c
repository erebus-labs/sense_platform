#include<stdint.h>
#define REG32(addr) (*(volatile uint32_t *)(addr))

#define GPIOE_BASE 0x40021000
#define GPIOE_MODER  REG32(GPIOE_BASE + 0x0)
#define GPIOE_OTYPER REG32(GPIOE_BASE + 0x4)
#define GPIOE_ODR    REG32(GPIOE_BASE + 0x14)
#define GPIOE_BSRR   REG32(GPIOE_BASE + 0x18)

#define RCC_BASE   0x40023800
#define RCC_AHBENR REG32(RCC_BASE + 0x1C)
#define RCC_AHBENR_GPIOEEN 0x10

#define STAT4_PIN 10
#define STAT4_PIN_MASK (1UL<<STAT4_PIN)

static void delay(int nops){
	while(nops>0){
		asm("nop");
		nops--;
	}
}/*end while*/

static void set_gpioe_moder(int pin, int mode){

	uint32_t moder;
	uint32_t moder_pin_pos;
	uint32_t moder_pin_mask;

	moder_pin_pos = pin*2;
	moder_pin_mask = 0x3UL << moder_pin_pos;

	moder = GPIOE_MODER;
	moder &= ~moder_pin_mask;
	moder |= (mode << moder_pin_pos);
	GPIOE_MODER = moder;
}/*end set_gpioe_pin*/

void main(void){
	RCC_AHBENR |= RCC_AHBENR_GPIOEEN;
	GPIOE_OTYPER |= STAT4_PIN_MASK;
	set_gpioe_moder(STAT4_PIN, 1);
	while(1){
		GPIOE_ODR |= STAT4_PIN_MASK;
//		GPIOE_BSRR |= (1UL<<11);
		delay(1000000);
//		GPIOE_BSRR |= (0b1<<25);
		GPIOE_ODR &= ~STAT4_PIN_MASK;
		delay(1000000);
	}
}
void SystemInit(void){
	return;
}
void _exit(int code){
	while(1);
}
