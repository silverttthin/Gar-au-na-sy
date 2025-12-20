#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "lcd.h"
#include "touch.h"

// ============================================================================
// 통합 main.c (IR + 초음파 + LCD + 음성인식(open) + 블루투스 모터)
// - "포트/핀/주변장치 매핑"은 각 모듈 테스트 코드 기준을 그대로 사용합니다.
// - 음성 인식 모듈 변경점 반영: USART가 아니라 UART4(PC10, PC11)로 수신
// - 모터 핀 변경점 반영(충돌 회피): PC10/11 -> PC6/7 로 이동 (motor.c 변경사항)
// - 3초 자동 닫힘은 SysTick(ms tick) 기반 비블로킹 비교로 구현(HAL_Delay 금지)
// ============================================================================

// -------------------------------
// [IR + SERVO] (servo_motor_with_IR.c 그대로)
// -------------------------------
#define IR_SENSOR_PIN   GPIO_Pin_1  // PA1
#define IR_THRESHOLD    2500        // 테스트 완료 기준값

// 서보모터 펄스폭 (테스트 완료 값 그대로)
#define SERVO_CLOSE     700         // 0도(닫힘)
#define SERVO_OPEN      2300        // 90도(열림)

// -------------------------------
// [ULTRASONIC] (ultrasonic_senser.c 그대로)
// -------------------------------
#define US_TRIG_PIN     GPIO_Pin_10
#define US_ECHO_PIN     GPIO_Pin_11
#define US_GPIO_PORT    GPIOB

#ifndef ORANGE
#define ORANGE          0xFD20
#endif

// -------------------------------
// [LCD UI 좌표] (각 테스트 코드 값 그대로)
// -------------------------------
#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_STATUS_X    20
#define LCD_STATUS_Y    80
#define LCD_VAL_X       20
#define LCD_VAL_Y       110

#define LCD_DIST_VAL_X  20
#define LCD_DIST_VAL_Y  110

// -------------------------------
// [요구된 상태/플래그]
// -------------------------------
volatile uint8_t isOpen = 0;          // 0=CLOSED, 1=OPEN
volatile uint8_t isMoving = 0;        // 0/1 (서보 동작 중)
volatile uint8_t voiceFlag = 0;       // 음성 트리거 플래그("open" 문자열 수신 시 1)
volatile uint8_t irFlag = 0;          // IR 이벤트 플래그(main에서 set)
volatile uint32_t openStartTime = 0;  // OPEN 진입 시각(서보 오픈 완료 시점)

// -------------------------------
// [ms tick] (HAL_Delay 금지 → SysTick 사용)
// -------------------------------
// SysTick_Handler는 프로젝트의 `stm32f10x_it.c`에 이미 정의되어 있으므로,
// main.c에서 중복 정의하면 링크 에러가 발생합니다.
// 따라서 tick 카운터는 main.c에 "변수만" 두고, 증가시키는 ISR은 stm32f10x_it.c에서 수행합니다.
volatile uint32_t g_msTick = 0;

static uint32_t GetTickMs(void) {
    return g_msTick;
}

// -------------------------------
// [초음파 주기]
// - 요구사항: 보정/평균/히스테리시스 없이 raw 값 그대로 사용
// - 주기는 "측정 빈도"만 제어 (raw 값 자체는 그대로)
// -------------------------------
#define ULTRA_PERIOD_MS 100u
static uint32_t g_lastUltraTime = 0;
static uint16_t g_lastBgColor = 0xFFFF;

// -------------------------------
// [서보 이동 시간(비블로킹)]
// - 테스트 코드에는 "서보 완료 인터럽트"가 없어서,
//   명령 후 일정 시간은 isMoving=1로 유지하여 트리거 무시 정책을 지킵니다.
// -------------------------------
#define SERVO_MOVE_TIME_MS 600u
static uint32_t g_servoMoveStart = 0;
static uint8_t  g_servoTargetOpen = 0; // 1=OPEN으로 이동 중, 0=CLOSE로 이동 중

// -------------------------------
// [VOICE(open) 문자열 파서 상태]
// - 잡음/무음은 UART 수신 자체가 없으므로 flag가 올라갈 일이 없음
// - 수신이 있더라도 "o->p->e->n" 순서가 맞을 때만 voiceFlag=1
// -------------------------------
static volatile uint8_t g_openMatchIdx = 0;

// ============================================================================
// (블루투스+모터) motor.c에서 사용하던 함수 원형 + (음성) voice.c 함수 원형
// ============================================================================
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void UART4_Init(void);
void NVIC_Configure(void);

// ============================================================================
// servo_motor_with_IR.c에서 사용하던 함수 원형 그대로
// ============================================================================
void AdcInit(void);
uint16_t Get_Adc_Value(void);
void setServoPulse(uint16_t pulse);
void TIM3_Configure_ForServo(void);

// ============================================================================
// ultrasonic_senser.c에서 사용하던 함수 원형 그대로
// ============================================================================
void TimerInit(void);
void Delay_us(uint32_t us);
uint32_t Get_Distance(void);

// ============================================================================
// [RCC 설정]
// - motor.c(변경버전): GPIOA/GPIOC/GPIOD/AFIO + USART1/USART2
// - voice.c: GPIOA/GPIOC + USART1 + UART4
// - IR/서보/초음파: GPIOB/ADC1/TIM2/TIM3
// ============================================================================
void RCC_Configure(void)
{
    // GPIOA, GPIOB, GPIOC, GPIOD 및 AFIO
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_AFIO  |
        RCC_APB2Periph_ADC1,
        ENABLE
    );

    // USART1, USART2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // UART4 (음성인식 모듈)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    // TIM2(초음파 us 타이머), TIM3(서보 PWM)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

// ============================================================================
// [GPIO 설정]
// - USART1: PA9/PA10
// - USART2(Remapped): PD5/PD6
// - UART4: PC10(TX) / PC11(RX)  (voice.c 그대로)
// - 모터: PC8/PC9(왼쪽), PC6/PC7(오른쪽)  (motor.c 변경사항 그대로)
// - IR: PA1(ADC), Servo: PB0(TIM3 CH3), Ultrasonic: PB10/PB11
// ============================================================================
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // ------------------------------------
    // [1] USART1 (PA9: TX, PA10: RX)
    // ------------------------------------
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ------------------------------------
    // [2] USART2 (Remap -> PD5: TX, PD6: RX)
    // ------------------------------------
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // ------------------------------------
    // [3] UART4 (PC10: TX, PC11: RX) - voice.c 그대로
    // ------------------------------------
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ------------------------------------
    // [4] 모터 제어 핀 (motor.c 변경사항 그대로)
    //  - 왼쪽: PC8, PC9
    //  - 오른쪽: PC6, PC7  (기존 PC10/11에서 이동 → UART4와 충돌 방지)
    // ------------------------------------
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ------------------------------------
    // [5] IR Sensor (PA1) -> Analog Input
    // ------------------------------------
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ------------------------------------
    // [6] Servo Motor (PB0) -> AF_PP
    // ------------------------------------
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ------------------------------------
    // [7] Ultrasonic (PB10 Trig: Out_PP, PB11 Echo: IN_FLOATING)
    // ------------------------------------
    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
}

// ============================================================================
// USART1/USART2 초기화 (motor.c 패턴 유지)
// ============================================================================
void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
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

// ============================================================================
// UART4 초기화 (voice.c 그대로: 115200, RXNE interrupt)
// ============================================================================
void UART4_Init(void)
{
    USART_InitTypeDef UART4_InitStructure;

    USART_Cmd(UART4, ENABLE);

    UART4_InitStructure.USART_BaudRate = 115200;
    UART4_InitStructure.USART_StopBits = USART_StopBits_1;
    UART4_InitStructure.USART_WordLength = USART_WordLength_8b;
    UART4_InitStructure.USART_Parity = USART_Parity_No;
    UART4_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    UART4_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &UART4_InitStructure);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}

// ============================================================================
// NVIC 설정
// - USART1/USART2는 기존 motor.c 우선순위 유지
// - UART4는 voice.c 패턴(우선순위 3)로 추가
// ============================================================================
void NVIC_Configure(void)
{
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

    // UART4 (voice.c)
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ============================================================================
// [ADC 초기화] (servo_motor_with_IR.c 그대로)
// ============================================================================
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

    // Channel 1 (PA1)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t Get_Adc_Value(void)
{
    return ADC_GetConversionValue(ADC1);
}

// ============================================================================
// [TIM3 서보 PWM 설정] (servo_motor_with_IR.c 그대로)
// ============================================================================
static TIM_OCInitTypeDef g_TIM_OCInitStructure;

void TIM3_Configure_ForServo(void)
{
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure_local;

    uint16_t prescale = (uint16_t)(SystemCoreClock / 1000000);

    TIM3_InitStructure.TIM_Period = 20000; // 20ms
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;

    TIM_OCInitStructure_local.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure_local.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure_local.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure_local.TIM_Pulse = SERVO_CLOSE;

    TIM_OC3Init(TIM3, &TIM_OCInitStructure_local);

    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void setServoPulse(uint16_t pulse)
{
    g_TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    g_TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    g_TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    g_TIM_OCInitStructure.TIM_Pulse = pulse;
    TIM_OC3Init(TIM3, &g_TIM_OCInitStructure);
}

// ============================================================================
// [TIM2 초음파용 1us 타이머 설정] (ultrasonic_senser.c 그대로)
// ============================================================================
void TimerInit(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM2, ENABLE);
}

void Delay_us(uint32_t us)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < us);
}

uint32_t Get_Distance(void)
{
    uint32_t time = 0;

    // Trig 10us
    GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

    // Echo High 대기
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == RESET);

    // 측정 시작
    TIM_SetCounter(TIM2, 0);

    // Echo Low 대기
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == SET);

    time = TIM_GetCounter(TIM2);

    // cm 변환 (ultrasonic_senser.c 그대로)
    return (time / 58);
}

// ============================================================================
// [USART IRQ] (블루투스 바퀴 제어는 ISR에서 즉시)
// - USART2: motor.c 변경버전(PC6/7/8/9) 그대로 반영
// - USART1: 기존처럼 디버깅 에코 유지
// ============================================================================
void USART1_IRQHandler(void)
{
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        word = USART_ReceiveData(USART1);
        USART_SendData(USART2, word);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void USART2_IRQHandler(void)
{
    uint16_t word;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

        word = USART_ReceiveData(USART2);

        // motor.c 변경버전 동작 그대로:
        // Left: PC8/PC9, Right: PC6/PC7
        switch (word) {
            case 'F': case 'f': // 전진
                GPIO_SetBits(GPIOC, GPIO_Pin_8);
                GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                GPIO_SetBits(GPIOC, GPIO_Pin_6);
                GPIO_ResetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'B': case 'b': // 후진
                GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                GPIO_SetBits(GPIOC, GPIO_Pin_9);
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                GPIO_SetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'L': case 'l': // 좌회전(제자리)
                GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                GPIO_SetBits(GPIOC, GPIO_Pin_9);
                GPIO_SetBits(GPIOC, GPIO_Pin_6);
                GPIO_ResetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'R': case 'r': // 우회전(제자리)
                GPIO_SetBits(GPIOC, GPIO_Pin_8);
                GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                GPIO_SetBits(GPIOC, GPIO_Pin_7);
                break;

            case 'S': case 's': // 정지
                GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
                break;
        }

        USART_SendData(USART1, word);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

// ============================================================================
// [UART4 IRQ] : 음성인식 모듈 수신
// - "open" 문자열을 정확히 받았을 때만 voiceFlag=1
// - 그 외 수신(잡음 문자열 등)은 voiceFlag를 올리지 않음
// ============================================================================
void UART4_IRQHandler(void)
{
    uint16_t word;

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
        word = USART_ReceiveData(UART4);

        // "open" 매칭(대소문자 허용)
        switch (g_openMatchIdx) {
            case 0:
                g_openMatchIdx = ((word == 'o') || (word == 'O')) ? 1 : 0;
                break;
            case 1:
                g_openMatchIdx = ((word == 'p') || (word == 'P')) ? 2 : 0;
                break;
            case 2:
                g_openMatchIdx = ((word == 'e') || (word == 'E')) ? 3 : 0;
                break;
            case 3:
                if ((word == 'n') || (word == 'N')) {
                    voiceFlag = 1; // "open" 완성 → 이벤트 발생
                }
                g_openMatchIdx = 0;
                break;
            default:
                g_openMatchIdx = 0;
                break;
        }

        // (선택) 디버깅: PC로 에코하고 싶으면 아래를 켜도 됨
        // USART_SendData(USART1, word);

        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

// ============================================================================
// [뚜껑 제어 헬퍼] (요구 플로우 준수: isMoving 동안 트리거 무시)
// ============================================================================
static void StartOpenLid(uint32_t now)
{
    isMoving = 1;
    g_servoTargetOpen = 1;
    g_servoMoveStart = now;

    setServoPulse(SERVO_OPEN);
}

static void StartCloseLid(uint32_t now)
{
    isMoving = 1;
    g_servoTargetOpen = 0;
    g_servoMoveStart = now;

    setServoPulse(SERVO_CLOSE);
}

static void ProcessServoDone(uint32_t now)
{
    if (!isMoving) return;

    // 서보 이동 중에는 새로운 트리거를 무시하므로, 플래그 잔재가 남지 않게 정리
    voiceFlag = 0;
    irFlag = 0;

    if ((uint32_t)(now - g_servoMoveStart) < SERVO_MOVE_TIME_MS) {
        return;
    }

    // 이동 완료 처리
    isMoving = 0;

    if (g_servoTargetOpen) {
        // 오픈 완료 후: isOpen=1, 초음파 비활성(메인루프에서 OPEN이면 측정 안 함), openStartTime=현재 tick
        isOpen = 1;
        openStartTime = now;

        // OPEN이면 LCD 전체 흰색
        LCD_Clear(WHITE);

        // 이벤트 플래그 소비
        voiceFlag = 0;
        irFlag = 0;
    } else {
        // 닫힘 완료 후: isOpen=0, 플래그 잔재 정리
        isOpen = 0;
        voiceFlag = 0;
        irFlag = 0;

        // CLOSED로 돌아오면 다음 초음파 측정에서 색이 갱신되도록 강제
        g_lastBgColor = 0xFFFF;
    }
}

static uint16_t BgColorFromDistance(uint32_t dist_cm)
{
    // 요구사항: 파란색(11~16), 주황색(6~10), 빨간색(~5)
    // 16보다 큰 경우는 "11 이상" 그룹으로 보고 BLUE 처리(추가 규칙이 없으므로 자연스럽게 확장)
    if (dist_cm <= 5)       return RED;
    else if (dist_cm <= 10) return ORANGE;
    else                    return BLUE;
}

// ============================================================================
// main
// ============================================================================
int main(void)
{
    // 기본 시스템 클럭 초기화(테스트 코드와 동일 패턴)
    SystemInit();

    // SysTick 1ms tick 설정 (비블로킹 타이밍 기반)
    SysTick_Config(SystemCoreClock / 1000);

    RCC_Configure();
    GPIO_Configure();

    USART1_Init();
    USART2_Init();
    UART4_Init();          // 음성인식 모듈 수신 포트(UART4) 추가
    NVIC_Configure();

    // IR/Servo/Ultrasonic 초기화
    AdcInit();
    TimerInit();           // TIM2 1us
    TIM3_Configure_ForServo();

    // LCD/Touch 초기화 (기존 버전 유지)
    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();

    // 초기 UI
    LCD_Clear(WHITE);
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "SMART BIN", BLUE, WHITE);
    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "State: CLOSED", BLACK, WHITE);
    LCD_ShowString(LCD_VAL_X, LCD_VAL_Y, "IR/Dist:", BLACK, WHITE);

    // 초기 상태
    isOpen = 0;
    isMoving = 0;
    voiceFlag = 0;
    irFlag = 0;

    setServoPulse(SERVO_CLOSE);

    while (1) {
        uint32_t now = GetTickMs();

        // 0) 서보 이동 완료 처리(비블로킹)
        ProcessServoDone(now);

        // 1) 서보 동작 중이면 새 트리거 무시(정책 고정)
        if (isMoving) {
            // ProcessServoDone()에서 이미 voiceFlag/irFlag 정리함
            continue;
        }

        // 2) OPEN 상태: 초음파 비활성 + 3초 후 자동 닫힘(연장 없음)
        if (isOpen) {
            // OPEN 중에는 어떤 트리거도 무시(연장 없음)
            voiceFlag = 0;
            irFlag = 0;

            if ((uint32_t)(now - openStartTime) >= 3000u) {
                StartCloseLid(now);
            }
            continue;
        }

        // 3) CLOSED 상태: 음성 우선(voiceFlag 먼저 검사)
        //    - voiceFlag==1이면 IR/초음파 트리거 판단에서 IR은 보지 않고 음성 트리거로만 처리
        if ((voiceFlag == 1) && (isOpen == 0) && (isMoving == 0)) {
            voiceFlag = 0;   // 소비
            StartOpenLid(now);
            continue;
        }

        // 4) CLOSED + voiceFlag==0 일 때만 IR/초음파 활성
        if (voiceFlag == 0) {
            // 4-1) IR 폴링 → irFlag set (raw ADC 값 그대로)
            {
                uint16_t ir_value = Get_Adc_Value();

                // LCD 디버깅(왼쪽은 IR)
                LCD_ShowNum(LCD_VAL_X + 80, LCD_VAL_Y, ir_value, 4, RED, WHITE);

                if (ir_value > IR_THRESHOLD) {
                    irFlag = 1;
                }
            }

            // 4-2) 초음파 주기 측정 → LCD 색 변경(원시값 그대로 3구간)
            if ((uint32_t)(now - g_lastUltraTime) >= ULTRA_PERIOD_MS) {
                uint32_t d = Get_Distance();
                uint16_t bg = BgColorFromDistance(d);

                g_lastUltraTime = now;

                // 거리 표시(기존 패턴 유지)
                LCD_ShowString(LCD_DIST_VAL_X, LCD_DIST_VAL_Y, "DIST(cm):", BLUE, WHITE);
                LCD_ShowNum(LCD_DIST_VAL_X + 80, LCD_DIST_VAL_Y, d, 4, RED, WHITE);

                // 배경색이 바뀌었을 때만 Clear(값 자체는 가공하지 않음)
                if (bg != g_lastBgColor) {
                    g_lastBgColor = bg;
                    LCD_Clear(bg);

                    // Clear 후 글자 복원
                    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "SMART BIN", BLUE, WHITE);
                    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "State: CLOSED", BLACK, WHITE);
                    LCD_ShowString(LCD_VAL_X, LCD_VAL_Y, "IR/Dist:", BLACK, WHITE);
                }
            }
        }

        // 5) IR 트리거 (voiceFlag==0일 때만, 우선순위 규칙 준수)
        if ((voiceFlag == 0) && (irFlag == 1) && (isOpen == 0) && (isMoving == 0)) {
            irFlag = 0;     // 소비
            StartOpenLid(now);
            continue;
        }
    }
}