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
// 1. RCC 설정 (클럭 활성화)
// ==========================================
void RCC_Configure(void)
{   
    /* USART1, USART2 TX/RX 및 모터 제어용 포트 클럭 활성화 */
    // GPIOA: USART1
    // GPIOD: USART2 (Remap)
    // GPIOC: 모터 제어 (PC8 ~ PC11)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
    
    /* USART1, USART2 클럭 활성화 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    /* Alternate Function IO 클럭 활성화 (Remap 사용 시 필수) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

// ==========================================
// 2. GPIO 설정 (핀 입출력 모드)
// ==========================================
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ------------------------------------
    // [1] USART1 핀 설정 (PA9: TX, PA10: RX)
    // ------------------------------------
    // TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // ------------------------------------
    // [2] USART2 핀 설정 (Remap -> PD5: TX, PD6: RX)
    // ------------------------------------
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    
    // TX (PD5)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // RX (PD6)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // ------------------------------------
    // [3] 모터 제어 핀 설정 (PC8, PC9, PC10, PC11)
    // ------------------------------------
    // Output Push-Pull, 50MHz
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// ==========================================
// 3. USART 초기화
// ==========================================
void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);
    
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1; // 보통 1비트 사용 (1.5는 특수목적)
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART1_InitStructure);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART_Cmd(USART2, ENABLE);
    
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART2_InitStructure);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

// ==========================================
// 4. NVIC (인터럽트) 설정
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
// 5. 인터럽트 핸들러 (동작 로직)
// ==========================================

// PC -> STM32 (디버깅용, 받은거 그대로 블루투스로 전송)
void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART1);
        USART_SendData(USART2, word); // 블루투스로 전달
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// 블루투스 -> STM32 (★핵심: 여기서 모터 제어)
void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        
        // 1. 문자 수신
        word = USART_ReceiveData(USART2);

        // 2. 문자에 따른 모터 제어 (대소문자 구분 없이 처리)
        // PC8, PC9 : 왼쪽 바퀴 / PC10, PC11 : 오른쪽 바퀴
        // BSRR 레지스터: 상위 16비트(Reset), 하위 16비트(Set)
        switch(word) {
            case 'F': // Forward (앞으로)
            case 'f':
                // Left(8,9): Fwd(8=1,9=0), Right(10,11): Fwd(10=1,11=0)
                // Set: 8, 10 / Reset: 9, 11
                GPIOC->BSRR = 0x0A000500; 
                break;

            case 'B': // Backward (뒤로)
            case 'b':
                // Left: Back(8=0,9=1), Right: Back(10=0,11=1)
                // Set: 9, 11 / Reset: 8, 10
                GPIOC->BSRR = 0x05000A00;
                break;

            case 'L': // Left (좌회전 - 제자리 돌기)
            case 'l':
                // Left: Back(8=0,9=1), Right: Fwd(10=1,11=0)
                // Set: 9, 10 / Reset: 8, 11
                GPIOC->BSRR = 0x0E000100;
                break;

            case 'R': // Right (우회전 - 제자리 돌기)
            case 'r':
                // Left: Fwd(8=1,9=0), Right: Back(10=0,11=1)
                // Set: 8, 11 / Reset: 9, 10
                GPIOC->BSRR = 0x0B000400;
                break;

            case 'S': // Stop (정지)
            case 's':
                // 모든 핀 Reset (8, 9, 10, 11 = 0)
                GPIOC->BRR = 0x0F00; 
                break;
        }

        // 3. 받은 문자를 PC로 에코 (디버깅용)
        USART_SendData(USART1, word);

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

// ==========================================
// 메인 함수
// ==========================================
int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();      // PC 연결용
    USART2_Init();      // 블루투스 연결용
    NVIC_Configure();

    while (1) {
        // 메인 루프는 비워둡니다. 모든 처리는 인터럽트에서 발생합니다.
    }
    return 0;
}
