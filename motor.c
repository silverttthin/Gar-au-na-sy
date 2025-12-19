#include "stm32f10x.h"

#define RCC_APB2ENR (*(volatile unsigned int *)0x40021018) 


#define GPIOA_CRL (*(volatile unsigned int *)0x40010800) // base: 0x4001 0800, offset: 0x00
#define GPIOB_CRH (*(volatile unsigned int *)0x40010C04) // base: 0x4001 0C00, offset: 0x04
#define GPIOC_CRL (*(volatile unsigned int *)0x40011000) // base: 0x4001 1000, offset: 0x00
#define GPIOC_CRH (*(volatile unsigned int *)0x40011004) // base: 0x4001 1000, offset: 0x04

#define GPIOC_BSRR (*(volatile unsigned int *)0x40011010)// base: 0x4001 1000, offset: 0x10


#define GPIOA_IDR (*(volatile unsigned int *)0x40010808) // base: 0x4001 0800, offset: 0x08
#define GPIOB_IDR (*(volatile unsigned int *)0x40010C08) // base: 0x4001 0C00, offset: 0x08
#define GPIOC_IDR (*(volatile unsigned int *)0x40011008) // base: 0x4001 1000, offset: 0x08

#define RESET 0x44444444

void delay(){
  int i;
  for(i=0; i<10000000; i++){}
}


int main(void)
{
    
    RCC_APB2ENR |= 0x3C;

    // reset 값으로 초기화
    GPIOA_CRL = RESET;
    GPIOB_CRH = RESET;
    GPIOC_CRL = RESET;
    GPIOC_CRH = RESET;
    

    GPIOA_CRL = 0x00000008; // PORT A의 0번 핀 사용
                            // CNF = input with pull-up/pull-down: 10, MODE = input mode: 00
                            // 0번 핀을 사용하프로 3~0번 비트를 조작해야함 -> 2진수 1000 = 16진수 0x8
    GPIOB_CRH = 0x00000800; // PB10, 위와 마찬가지로 몇번 핀을 사용할지 보고 그에 맞게 설정
    GPIOC_CRH = 0x00800000; // PC13
    GPIOC_CRL = 0x00080000; // PC4


    // Relay Module
    GPIOC_CRH |= 0x00000033; // PC8, PC9
    
    
    unsigned int KEY1 = 0x0010; // PC4, 4번핀 값을 읽기 위해서는 4번 비트 값을 읽어야함
    unsigned int KEY2 = 0x0400; // PB10, 10번핀은 10번 비트
    unsigned int KEY3 = 0x2000; // PC13, 13번핀은 13번 비트
    unsigned int KEY4 = 0x0001; // PA0, 0번핀은 9번 비트

    
    while(1){
        if((GPIOC_IDR & KEY1) == 0){ // Key1
            GPIOC_BSRR = 0x02000100; // PC8 set, PC9 reset
        }
        else if((GPIOB_IDR & KEY2) == 0){ // Key2
            GPIOC_BSRR = 0x01000200; // PC8 reset, PC9 set
        }
        else if((GPIOC_IDR & KEY3) == 0){ //key 3
            GPIOC_BSRR = 0x02000100; // PC8 set, PC9 reset
            delay();
            delay();
            GPIOC_BSRR = 0x01000200; // PC8 reset, PC9 set
            delay();
            delay();
            GPIOC_BSRR = 0x03000000; // PC8 reset, PC9 reset
        }
        else if((GPIOA_IDR & KEY4) == 0){ //key 4
            GPIOC_BSRR = 0x03000000; // PC8 reset, PC9 reset
        }
    }
    return 0;
}