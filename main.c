/*
 * main.c - 스마트 쓰레기통 통합 펌웨어 (최종 수정: IR 센서 민감도 복구)
 *
 * [수정 사항]
 * 1. IR_FILTER_COUNT: 1000 -> 50 (반응 속도 높임)
 * 2. IsMotorRunning: 모터가 멈춰있을 때만 센서 작동 (유지)
 */

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "lcd.h"

/* =========================================================
   IR + Servo
   ========================================================= */
#define IR_SENSOR_PIN        GPIO_Pin_1      /* PA1 */
#define IR_THRESHOLD         170           /* 감도 약간 더 민감하게 조정 (2500 -> 2000) */
#define IR_FILTER_COUNT      10              /* [중요] 반응 속도 빠르게 수정 (1000 -> 50) */

#define SERVO_CLOSE          700
#define SERVO_OPEN           2300
#define SERVO_MOVE_TIME_MS   500U

/* =========================================================
   Ultrasonic
   ========================================================= */
#define US_TRIG_PIN          GPIO_Pin_10     /* PB10 */
#define US_ECHO_PIN          GPIO_Pin_11     /* PB11 */
#define US_GPIO_PORT         GPIOB

#ifndef ORANGE
#define ORANGE               0xFD20
#endif
#define ECHO_TIMEOUT_US      30000U

/* =========================================================
   UART
   ========================================================= */
#define VOICE_UART           UART4
#define VOICE_UART_BAUDRATE  115200

#define BT_UART              USART2
#define BT_UART_BAUDRATE     9600

/* =========================================================
   Motor Pins (PA8/PA9, PC5/PC9)
   ========================================================= */
#define MOTOR_LEFT_PORT      GPIOC
#define MOTOR_LEFT_FWD       GPIO_Pin_5
#define MOTOR_LEFT_BWD       GPIO_Pin_9

#define MOTOR_RIGHT_PORT     GPIOA
#define MOTOR_RIGHT_FWD      GPIO_Pin_8
#define MOTOR_RIGHT_BWD      GPIO_Pin_9

/* =========================================================
   LCD
   ========================================================= */
#if defined(LCD_WIDTH) && defined(LCD_HEIGHT)
  #define LCD_SCREEN_W   LCD_WIDTH
  #define LCD_SCREEN_H   LCD_HEIGHT
#elif defined(LCD_W) && defined(LCD_H)
  #define LCD_SCREEN_W   LCD_W
  #define LCD_SCREEN_H   LCD_H
#elif defined(LCD_X_MAX) && defined(LCD_Y_MAX)
  #define LCD_SCREEN_W   LCD_X_MAX
  #define LCD_SCREEN_H   LCD_Y_MAX
#else
  #define LCD_SCREEN_W   240
  #define LCD_SCREEN_H   320
#endif

#define FONT_W          8
#define FONT_H          16

static uint16_t BgColorFromDistance(uint32_t dist_cm)
{
    if (dist_cm <= 5U)       return RED;
    else if (dist_cm <= 10U) return ORANGE;
    else                     return BLUE;
}

static const char* StatusTextFromDistance(uint32_t dist_cm)
{
    if (dist_cm <= 5U)       return "Warning";
    else if (dist_cm <= 10U) return "Caution";
    else                     return "Normal";
}

static void DrawCenteredStatus(const char* text, uint16_t textColor, uint16_t bgColor)
{
    uint16_t len = 0;
    while (text[len] != '\0') len++;

    uint16_t x0 = (uint16_t)((LCD_SCREEN_W - (len * FONT_W)) / 2U);
    uint16_t y0 = (uint16_t)((LCD_SCREEN_H - FONT_H) / 2U);

    LCD_ShowString(x0, y0, "          ", bgColor, bgColor);
    LCD_ShowString(x0, y0, (char*)text, textColor, bgColor);
}

/* =========================================================
   Global Variables
   ========================================================= */
static volatile int isOpen = 0;
static volatile int isMoving = 0;
static volatile int voiceFlag = 0;
static volatile int irFlag = 0;
static volatile uint32_t openStartTime = 0;
static volatile uint32_t moveDoneTime = 0;
static volatile int moveToOpen = 0;
static volatile uint32_t ir_detect_count = 0;
static volatile uint8_t voiceMatchIdx = 0;

/* =========================================================
   Prototypes
   ========================================================= */
void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void TIM_Configure(void);
void UART4_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);
uint16_t Get_Adc_Value(void);
void setServoPulse(uint16_t pulse);
void TimerInit(void);
void Delay_us(uint32_t us);
uint32_t Get_Distance_cm(void);

/* 모터 작동 여부 확인 함수 */
uint8_t IsMotorRunning(void)
{
    /* 왼쪽 모터 핀 상태 읽기 */
    if (GPIO_ReadOutputDataBit(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD)) return 1;
    if (GPIO_ReadOutputDataBit(MOTOR_LEFT_PORT, MOTOR_LEFT_BWD)) return 1;
    
    /* 오른쪽 모터 핀 상태 읽기 */
    if (GPIO_ReadOutputDataBit(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD)) return 1;
    if (GPIO_ReadOutputDataBit(MOTOR_RIGHT_PORT, MOTOR_RIGHT_BWD)) return 1;

    return 0; /* 모두 꺼져있음 */
}

/* =========================================================
   DWT Tick
   ========================================================= */
static void DWT_TickInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t GetTickMs(void)
{
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000U));
}

/* =========================================================
   Interrupt Handlers
   ========================================================= */
void UART4_IRQHandler(void)
{
    uint16_t rx_char;
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        rx_char = USART_ReceiveData(UART4);
        switch (voiceMatchIdx) {
            case 0: voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0; break;
            case 1: 
                if ((rx_char == 'p') || (rx_char == 'P')) voiceMatchIdx = 2;
                else voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0;
                break;
            case 2:
                if ((rx_char == 'e') || (rx_char == 'E')) voiceMatchIdx = 3;
                else voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0;
                break;
            case 3:
                if ((rx_char == 'n') || (rx_char == 'N')) voiceFlag = 1;
                voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0;
                break;
            default: voiceMatchIdx = 0; break;
        }
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

void USART2_IRQHandler(void)
{
    uint16_t cmd;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        cmd = USART_ReceiveData(USART2);
        switch (cmd) {
            case 'F': case 'f':
                GPIO_SetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD);
                GPIO_ResetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_BWD);
                GPIO_SetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD);
                GPIO_ResetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_BWD);
                break;
            case 'B': case 'b':
                GPIO_ResetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD);
                GPIO_SetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_BWD);
                GPIO_ResetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD);
                GPIO_SetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_BWD);
                break;
            case 'R': case 'r':
                GPIO_ResetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD);
                GPIO_SetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_BWD);
                GPIO_SetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD);
                GPIO_ResetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_BWD);
                break;
            case 'L': case 'l':
                GPIO_SetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD);
                GPIO_ResetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_BWD);
                GPIO_ResetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD);
                GPIO_SetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_BWD);
                break;
            case 'S': case 's':
                GPIO_ResetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD | MOTOR_LEFT_BWD);
                GPIO_ResetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD | MOTOR_RIGHT_BWD);
                break;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* =========================================================
   Main
   ========================================================= */
int main(void)
{
    uint16_t prevColor = 0xFFFFU;
    uint32_t nextUltrasonicTime = 0U;
    char lcd_buffer[20]; // LCD 출력을 위한 버퍼

    isOpen = 0; isMoving = 0; voiceFlag = 0; irFlag = 0; openStartTime = 0;
    ir_detect_count = 0;

    Init();
    SystemCoreClockUpdate();
    DWT_TickInit();
    LCD_Clear(BLUE);
    DrawCenteredStatus("Normal", WHITE, BLUE);

    while (1)
    {
        uint32_t now = GetTickMs();

        /* [디버깅용] IR 센서 값을 LCD 좌측 상단에 표시 */
        /* 이 부분은 최종 동작 확인 후 주석 처리 가능 */
        uint16_t current_ir_value = Get_Adc_Value();
        sprintf(lcd_buffer, "IR: %4d", current_ir_value);
        LCD_ShowString(10, 10, lcd_buffer, WHITE, BLUE);

        /* (A) 서보 이동 중 처리 (기존 동일) */
        if (isMoving)
        {
            voiceFlag = 0; irFlag = 0; ir_detect_count = 0;
            if ((int32_t)(now - moveDoneTime) >= 0)
            {
                isMoving = 0;
                if (moveToOpen) {
                    isOpen = 1; openStartTime = now;
                    LCD_Clear(WHITE); DrawCenteredStatus("opened", BLACK, WHITE);
                    prevColor = WHITE;
                } else {
                    isOpen = 0; voiceFlag = 0; irFlag = 0; prevColor = 0xFFFFU;
                }
            }
            continue;
        }

        /* (B) OPEN 상태 처리 (기존 동일) */
        if (isOpen)
        {
            /* OPEN 상태에서도 IR 값은 계속 갱신해서 보여줌 (디버깅용) */
            sprintf(lcd_buffer, "IR: %4d", Get_Adc_Value());
            LCD_ShowString(10, 10, lcd_buffer, BLACK, WHITE);

            if ((uint32_t)(now - openStartTime) >= 3000U)
            {
                isMoving = 1; moveToOpen = 0;
                setServoPulse(SERVO_CLOSE);
                moveDoneTime = now + SERVO_MOVE_TIME_MS;
                voiceFlag = 0; irFlag = 0;
            }
            continue;
        }

        /* (C) CLOSED 상태 처리 */
        
        /* (C-1) 음성 트리거 (기존 동일) */
        if (voiceFlag == 1)
        {
            if ((isOpen == 0) && (isMoving == 0))
            {
                isMoving = 1; moveToOpen = 1;
                setServoPulse(SERVO_OPEN);
                moveDoneTime = now + SERVO_MOVE_TIME_MS;
                voiceFlag = 0;
            }
            continue;
        }

        /* (C-2) IR 센서 로직 [수정됨] */
        /* 모터가 돌고 있으면 무조건 IR 무시 */
        if (IsMotorRunning() == 1)
        {
            ir_detect_count = 0;
            irFlag = 0;
            LCD_ShowString(10, 30, "Motor: ON ", RED, BLUE); // 디버깅: 모터 상태 표시
        }
        else 
        {
            LCD_ShowString(10, 30, "Motor: OFF", GREEN, BLUE); // 디버깅: 모터 상태 표시

            /* IR 값은 위에서 읽은 current_ir_value 사용 */
            if (current_ir_value > IR_THRESHOLD) 
            {
                ir_detect_count++;
            }
            else 
            {
                /* * [중요] 한 번이라도 낮아지면 0으로 리셋하는 대신, 
                 * 약간의 히스테리시스를 주거나 천천히 감소시킬 수도 있으나,
                 * 일단 0으로 리셋하되 Threshold를 낮추는 방향으로 접근
                 */
                ir_detect_count = 0;
                irFlag = 0;
            }

            if (ir_detect_count > IR_FILTER_COUNT) {
                irFlag = 1;
                // 카운트 오버플로우 방지 (51에서 멈춤)
                ir_detect_count = IR_FILTER_COUNT + 1; 
            }
        }

        /* (C-3) IR 트리거 검사 */
        if ((voiceFlag == 0) && (irFlag == 1) && (isOpen == 0) && (isMoving == 0))
        {
            isMoving = 1; moveToOpen = 1;
            setServoPulse(SERVO_OPEN);
            moveDoneTime = now + SERVO_MOVE_TIME_MS;
            irFlag = 0; ir_detect_count = 0;
            continue;
        }

        /* (C-4) 초음파 (기존 동일) */
        if ((int32_t)(now - nextUltrasonicTime) >= 0)
        {
            uint32_t dist_cm = Get_Distance_cm();
            uint16_t color;
            if (dist_cm == 0U) dist_cm = 999U;
            if (dist_cm > 999U) dist_cm = 999U;
            color = BgColorFromDistance(dist_cm);
            
            /* LCD 배경색이 바뀔 때 IR 값 표시가 지워지지 않게 주의 */
            if (color != prevColor) {
                LCD_Clear(color); prevColor = color;
            }
            DrawCenteredStatus(StatusTextFromDistance(dist_cm), WHITE, color);
            nextUltrasonicTime = now + 60U;
        }
        
        /* [중요] 루프 속도 안정화 */
        /* 너무 빠른 루프는 센서 카운팅을 불안정하게 합니다. 1ms 정도 지연을 줍니다. */
        /* 이렇게 하면 IR_FILTER_COUNT 50은 약 50ms 동안 감지됨을 의미하게 됩니다. */
        Delay_us(1000); 
    }
}

/* =========================================================
   Init Functions
   ========================================================= */
void Init(void)
{
    SystemInit(); RccInit(); GpioInit(); AdcInit();
    TIM_Configure(); TimerInit(); UART4_Init(); USART2_Init(); NVIC_Configure();
    LCD_Init(); setServoPulse(SERVO_CLOSE);
    GPIO_ResetBits(MOTOR_LEFT_PORT, MOTOR_LEFT_FWD | MOTOR_LEFT_BWD);
    GPIO_ResetBits(MOTOR_RIGHT_PORT, MOTOR_RIGHT_FWD | MOTOR_RIGHT_BWD);
}

void RccInit(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_UART4 | RCC_APB1Periph_USART2, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}

void GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // IR (PA1)
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Servo (PB0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // US Trig(PB10)/Echo(PB11)
    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
    
    // LED (PD2,3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
    
    // UART4 (PC10,11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // USART2 (PD5,6 Remap)
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Motors
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_FWD | MOTOR_LEFT_BWD;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_LEFT_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_RIGHT_FWD | MOTOR_RIGHT_BWD;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MOTOR_RIGHT_PORT, &GPIO_InitStructure);
}

void AdcInit(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1); while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TIM_Configure(void)
{
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure_local;
    uint16_t prescale = (uint16_t)((SystemCoreClock / 1000000U) - 1U);
    TIM3_InitStructure.TIM_Period = 20000U - 1U;
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_OCInitStructure_local.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure_local.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure_local.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure_local.TIM_Pulse = SERVO_CLOSE;
    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure_local);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void UART4_Init(void)
{
    USART_InitTypeDef UART4_InitStructure;
    USART_Cmd(UART4, ENABLE);
    UART4_InitStructure.USART_BaudRate = VOICE_UART_BAUDRATE;
    UART4_InitStructure.USART_StopBits = USART_StopBits_1;
    UART4_InitStructure.USART_WordLength = USART_WordLength_8b;
    UART4_InitStructure.USART_Parity = USART_Parity_No;
    UART4_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    UART4_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &UART4_InitStructure);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;
    USART_Cmd(USART2, ENABLE);
    USART2_InitStructure.USART_BaudRate = BT_UART_BAUDRATE;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART2_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint16_t Get_Adc_Value(void) { return ADC_GetConversionValue(ADC1); }
void setServoPulse(uint16_t pulse) { TIM_SetCompare3(TIM3, pulse); }
void TimerInit(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
}
void Delay_us(uint32_t us) {
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < us) { }
}
uint32_t Get_Distance_cm(void) {
    uint32_t t_us;
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN); Delay_us(2);
    GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN); Delay_us(10);
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
    TIM_SetCounter(TIM2, 0);
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == RESET) { if ((uint32_t)TIM_GetCounter(TIM2) > ECHO_TIMEOUT_US) return 0U; }
    TIM_SetCounter(TIM2, 0);
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == SET) { if ((uint32_t)TIM_GetCounter(TIM2) > ECHO_TIMEOUT_US) break; }
    t_us = (uint32_t)TIM_GetCounter(TIM2);
    return (t_us / 58U);
}

//AI 안되는버전
