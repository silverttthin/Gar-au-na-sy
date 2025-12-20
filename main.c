#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void UART4_Init(void); // 이름 변경
void NVIC_Configure(void);

void RCC_Configure(void)
{  
    /* USART1, UART4 TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // USART1용 GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  // UART4용 GPIOC (PC10, PC11)
    
    /* USART1 clock enable (APB2) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);    
    
    /* UART4 clock enable (APB1) */
    // UART4도 APB1 버스를 사용합니다.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    
    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* USART1 pin setting (PA9, PA10) */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* UART4 pin setting (PC10, PC11) */
    // UART4는 기본적으로 PC10(TX), PC11(RX)를 사용합니다. 별도 리매핑 불필요.
    
    //TX (PC10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    //RX (PC11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);
    
    USART1_InitStructure.USART_BaudRate = 115200;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode= USART_Mode_Rx| USART_Mode_Tx;
    USART_Init(USART1, &USART1_InitStructure);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void UART4_Init(void)
{
    USART_InitTypeDef UART4_InitStructure;

    // Enable the UART4 peripheral
    USART_Cmd(UART4, ENABLE);
    
    // UART4 설정 (구조체는 USART_InitTypeDef를 그대로 사용)
    UART4_InitStructure.USART_BaudRate = 115200;
    UART4_InitStructure.USART_StopBits = USART_StopBits_1;
    UART4_InitStructure.USART_WordLength = USART_WordLength_8b;
    UART4_InitStructure.USART_Parity = USART_Parity_No;
    UART4_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    UART4_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    // UART4 초기화
    USART_Init(UART4, &UART4_InitStructure);
    
    // RX 인터럽트 활성화
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART1 (PC 통신용)
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART4 (감바랩스 모듈용)
    // IRQ 채널을 UART4_IRQn으로 변경
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        // PC에서 받은 데이터를 UART4(감바랩스)로 전송
        word = USART_ReceiveData(USART1);

        // UART4로 전송
        USART_SendData(UART4, word);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// UART4 인터럽트 핸들러 (이름 주의: UART4_IRQHandler)
void UART4_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET){
        // 감바랩스 모듈(UART4)에서 받은 데이터를 PC(USART1)로 전송
        word = USART_ReceiveData(UART4);

        // USART1로 전송
        USART_SendData(USART1, word);

        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

int main(void)
{
    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    USART1_Init();      // PC (Debug)
    
    UART4_Init();       // Gambalabs Module (PC10, PC11)

    NVIC_Configure();

    while (1) {
    }
    return 0;
}