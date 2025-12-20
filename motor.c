#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

/* Function Prototypes */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);

// ==========================================
// 1. RCC 설정
// ==========================================
void RCC_Configure(void)
{   
    // GPIOA, GPIOC, GPIOD 및 AFIO 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    
    // USART1, USART2 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

// ==========================================
// 2. GPIO 설정
// ==========================================
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // [1] USART1 (PC 연결): PA9(TX), PA10(RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // [2] USART2 (블루투스): PD5(TX), PD6(RX) - 리맵됨
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // [3] 모터 제어 핀 설정 (PC10,11 -> PC6,7 로 변경됨)
    // 왼쪽: PC8, PC9
    // 오른쪽: PC6, PC7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// ==========================================
// 3. USART 초기화
// ==========================================
void USART1_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART1, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART2, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

// ==========================================
// 4. NVIC 설정
// ==========================================
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART1
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ==========================================
// 5. 인터럽트 핸들러
// ==========================================

// PC -> STM32
void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART1);
        USART_SendData(USART2, word); 
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// 블루투스 -> STM32 (모터 제어 핀 변경 적용)
void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        
        word = USART_ReceiveData(USART2);

        // BSRR 레지스터 계산이 복잡하므로, 직관적인 함수로 변경했습니다.
        // 동작은 기존과 100% 동일합니다.
        switch(word) {
            case 'F': case 'f': // 전진
                // Left Fwd (8=H, 9=L)
                GPIO_SetBits(GPIOC, GPIO_Pin_8);
                GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                // Right Fwd (6=H, 7=L) - New Pins
                GPIO_SetBits(GPIOC, GPIO_Pin_6);
                GPIO_ResetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'B': case 'b': // 후진
                // Left Back (8=L, 9=H)
                GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                GPIO_SetBits(GPIOC, GPIO_Pin_9);
                // Right Back (6=L, 7=H) - New Pins
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                GPIO_SetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'L': case 'l': // 좌회전
                // Left Back
                GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                GPIO_SetBits(GPIOC, GPIO_Pin_9);
                // Right Fwd
                GPIO_SetBits(GPIOC, GPIO_Pin_6);
                GPIO_ResetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'R': case 'r': // 우회전
                // Left Fwd
                GPIO_SetBits(GPIOC, GPIO_Pin_8);
                GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                // Right Back
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                GPIO_SetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'S': case 's': // 정지
                GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
                break;
        }

        USART_SendData(USART1, word);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();      
    USART2_Init();      
    NVIC_Configure();

    while (1) {
    }
    return 0;
}
