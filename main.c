/*
 * main.c - 스마트 쓰레기통 통합 펌웨어
 * 
 * [기능 요약]
 * - IR 센서: 손 감지 → 뚜껑 열기
 * - 음성인식(UART4): "open" 수신 → 뚜껑 열기 (IR보다 우선)
 * - 초음파: 거리 측정 → LCD 배경색 변경 (CLOSED 상태에서만)
 * - 서보모터: 뚜껑 개폐
 * - 블루투스(USART2): 자동차 바퀴 제어 (뚜껑 로직과 독립)
 * 
 * [포트 매핑]
 * - PA1: IR 센서 (ADC1 CH1)
 * - PB0: 서보모터 PWM (TIM3 CH3)
 * - PB10/PB11: 초음파 Trig/Echo
 * - PC6/PC7: 오른쪽 모터 (motor.c 기준)
 * - PC5/PC9: 왼쪽 모터 (motor.c 기준)
 * - PC10/PC11: UART4 TX/RX (음성인식 모듈)
 * - PD5/PD6: USART2 TX/RX (블루투스, 리맵)
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
   IR + Servo (servo_motor_with_IR.c 기준)
   ========================================================= */
#define IR_SENSOR_PIN        GPIO_Pin_1      /* PA1 */
#define IR_THRESHOLD         2500

#define SERVO_CLOSE          700             /* 0도(닫힘) */
#define SERVO_OPEN           2300            /* 90도(열림) */

#define SERVO_MOVE_TIME_MS   500U

/* =========================================================
   Ultrasonic (ultrasonic_senser.c 기준)
   ========================================================= */
#define US_TRIG_PIN          GPIO_Pin_10     /* PB10 */
#define US_ECHO_PIN          GPIO_Pin_11     /* PB11 */
#define US_GPIO_PORT         GPIOB

#ifndef ORANGE
#define ORANGE               0xFD20
#endif

#define ECHO_TIMEOUT_US      30000U

/* =========================================================
   UART4 - 음성인식 모듈 (voice.c 기준)
   PC10(TX), PC11(RX), 115200bps
   ========================================================= */
#define VOICE_UART           UART4
#define VOICE_UART_BAUDRATE  115200

/* =========================================================
   USART2 - 블루투스 모터 제어 (motor.c 기준)
   PD5(TX), PD6(RX) - 리맵 사용, 9600bps
   ========================================================= */
#define BT_UART              USART2
#define BT_UART_BAUDRATE     9600

/* =========================================================
   모터 제어 핀 정의 (motor.c 기준)
   - 왼쪽 모터: PC5(+), PC9(-)
   - 오른쪽 모터: PC6(+), PC7(-)
   ========================================================= */
#define MOTOR_LEFT_FWD       GPIO_Pin_5      /* PC5 */
#define MOTOR_LEFT_BWD       GPIO_Pin_9      /* PC9 */
#define MOTOR_RIGHT_FWD      GPIO_Pin_6      /* PC6 */
#define MOTOR_RIGHT_BWD      GPIO_Pin_7      /* PC7 */
#define MOTOR_ALL_PINS       (MOTOR_LEFT_FWD | MOTOR_LEFT_BWD | MOTOR_RIGHT_FWD | MOTOR_RIGHT_BWD)

/* =========================================================
   LCD 중앙 텍스트 출력용 (기본 폰트 8x16 가정)
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
   상태/플래그 (절대 변경 금지 항목)
   - isOpen: 0=CLOSED, 1=OPEN
   - isMoving: 서보 동작 중 여부
   - voiceFlag: "open" 수신 시 1 (IRQ에서 set, main에서 소비)
   - irFlag: IR 감지 시 1
   - openStartTime: 뚜껑 열림 완료 시점 (3초 카운트용)
   ========================================================= */
static volatile int isOpen = 0;
static volatile int isMoving = 0;
static volatile int voiceFlag = 0;
static volatile int irFlag = 0;
static volatile uint32_t openStartTime = 0;

static volatile uint32_t moveDoneTime = 0;
static volatile int moveToOpen = 0;

/* =========================================================
   음성인식용 "open" 문자열 매칭 상태
   - UART4 IRQ에서 한 글자씩 수신하며 o→p→e→n 순서로 매칭
   - 매칭 완료 시 voiceFlag = 1
   ========================================================= */
static volatile uint8_t voiceMatchIdx = 0;

static TIM_OCInitTypeDef TIM_OCInitStructure;

/* =========================================================
   프로토타입
   ========================================================= */
void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void TIM_Configure(void);
void UART4_Init(void);       /* 음성인식 UART 초기화 */
void USART2_Init(void);      /* 블루투스 UART 초기화 */
void NVIC_Configure(void);   /* 인터럽트 설정 */

uint16_t Get_Adc_Value(void);
void setServoPulse(uint16_t pulse);

void TimerInit(void);
void Delay_us(uint32_t us);
uint32_t Get_Distance_cm(void);

/* =========================================================
   DWT 기반 ms tick (비블로킹 타이밍용)
   ========================================================= */
static void DWT_TickInit(void)
{
    /* DWT(Data Watchpoint and Trace) 유닛의 사이클 카운터 활성화 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t GetTickMs(void)
{
    /* CPU 사이클 수를 ms로 변환 */
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000U));
}

/* =========================================================
   UART4 인터럽트 핸들러 - 음성인식 모듈
   - "open" 문자열 수신 시 voiceFlag = 1
   - 잡음/무음일 때는 데이터 수신 자체가 없으므로 처리 안 함
   ========================================================= */
void UART4_IRQHandler(void)
{
    uint16_t rx_char;
    
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        rx_char = USART_ReceiveData(UART4);
        
        /*
         * "open" 문자열 순차 매칭 로직
         * - 상태머신 방식: o(0) → p(1) → e(2) → n(3) → 완료
         * - 중간에 틀리면 다시 처음부터
         * - 대소문자 구분 없이 매칭
         */
        switch (voiceMatchIdx)
        {
            case 0:
                if ((rx_char == 'o') || (rx_char == 'O'))
                    voiceMatchIdx = 1;
                else
                    voiceMatchIdx = 0;
                break;
                
            case 1:
                if ((rx_char == 'p') || (rx_char == 'P'))
                    voiceMatchIdx = 2;
                else
                    voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0;
                break;
                
            case 2:
                if ((rx_char == 'e') || (rx_char == 'E'))
                    voiceMatchIdx = 3;
                else
                    voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0;
                break;
                
            case 3:
                if ((rx_char == 'n') || (rx_char == 'N'))
                {
                    voiceFlag = 1;
                }
                voiceMatchIdx = ((rx_char == 'o') || (rx_char == 'O')) ? 1 : 0;
                break;
                
            default:
                voiceMatchIdx = 0;
                break;
        }
        
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

/* =========================================================
   USART2 인터럽트 핸들러 - 블루투스 모터 제어 (motor.c 기준)
   
   [명령어]
   - 'F'/'f': 전진 (양쪽 모터 정회전)
   - 'B'/'b': 후진 (양쪽 모터 역회전)
   - 'L'/'l': 좌회전 (왼쪽 역회전, 오른쪽 정회전)
   - 'R'/'r': 우회전 (왼쪽 정회전, 오른쪽 역회전)
   - 'S'/'s': 정지 (모든 모터 OFF)
   
   [특징]
   - 뚜껑 로직과 완전히 독립적으로 동작
   - 인터럽트로 즉시 반응 → 메인 루프를 방해하지 않음
   ========================================================= */
void USART2_IRQHandler(void)
{
    uint16_t cmd;
    
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        cmd = USART_ReceiveData(USART2);
        
        /*
         * 모터 제어 로직 (motor.c 그대로 사용)
         * - GPIO_SetBits: 해당 핀 HIGH
         * - GPIO_ResetBits: 해당 핀 LOW
         * - 모터 드라이버: HIGH/LOW 조합으로 방향 결정
         */
        switch (cmd)
        {
            case 'F': case 'f':  /* 전진 */
                /* 왼쪽 모터: 정회전 (8=H, 9=L) */
                GPIO_SetBits(GPIOC, MOTOR_LEFT_FWD);
                GPIO_ResetBits(GPIOC, MOTOR_LEFT_BWD);
                /* 오른쪽 모터: 정회전 (6=H, 7=L) */
                GPIO_SetBits(GPIOC, MOTOR_RIGHT_FWD);
                GPIO_ResetBits(GPIOC, MOTOR_RIGHT_BWD);
                break;
                
            case 'B': case 'b':  /* 후진 */
                /* 왼쪽 모터: 역회전 (8=L, 9=H) */
                GPIO_ResetBits(GPIOC, MOTOR_LEFT_FWD);
                GPIO_SetBits(GPIOC, MOTOR_LEFT_BWD);
                /* 오른쪽 모터: 역회전 (6=L, 7=H) */
                GPIO_ResetBits(GPIOC, MOTOR_RIGHT_FWD);
                GPIO_SetBits(GPIOC, MOTOR_RIGHT_BWD);
                break;
                
            case 'R': case 'r':  /* 좌회전 */
                /* 왼쪽 모터: 역회전 */
                GPIO_ResetBits(GPIOC, MOTOR_LEFT_FWD);
                GPIO_SetBits(GPIOC, MOTOR_LEFT_BWD);
                /* 오른쪽 모터: 정회전 */
                GPIO_SetBits(GPIOC, MOTOR_RIGHT_FWD);
                GPIO_ResetBits(GPIOC, MOTOR_RIGHT_BWD);
                break;
                
            case 'L': case 'l':  /* 우회전 */
                /* 왼쪽 모터: 정회전 */
                GPIO_SetBits(GPIOC, MOTOR_LEFT_FWD);
                GPIO_ResetBits(GPIOC, MOTOR_LEFT_BWD);
                /* 오른쪽 모터: 역회전 */
                GPIO_ResetBits(GPIOC, MOTOR_RIGHT_FWD);
                GPIO_SetBits(GPIOC, MOTOR_RIGHT_BWD);
                break;
                
            case 'S': case 's':  /* 정지 */
                GPIO_ResetBits(GPIOC, MOTOR_ALL_PINS);
                break;
                
            default:
                /* 알 수 없는 명령은 무시 */
                break;
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* =========================================================
   메인 함수 - 상태머신 기반 폴링 루프
   ========================================================= */
int main(void)
{
    uint16_t prevColor = 0xFFFFU;
    uint32_t nextUltrasonicTime = 0U;

    /* 초기 상태 설정 (절대 변경 금지) */
    isOpen = 0;
    isMoving = 0;
    voiceFlag = 0;
    irFlag = 0;
    openStartTime = 0;

    Init();

    SystemCoreClockUpdate();
    DWT_TickInit();

    LCD_Clear(BLUE);
    DrawCenteredStatus("Normal", WHITE, BLUE);

    while (1)
    {
        uint32_t now = GetTickMs();

        /* =====================================================
           (A) 서보 이동 중 처리
           - 이동 중에는 새 트리거 무시
           - 남은 플래그 정리 (재트리거 방지)
           - 블루투스 모터 제어는 IRQ에서 독립 처리되므로 영향 없음
           ===================================================== */
        if (isMoving)
        {
            voiceFlag = 0;
            irFlag = 0;

            if ((int32_t)(now - moveDoneTime) >= 0)
            {
                isMoving = 0;

                if (moveToOpen)
                {
                    isOpen = 1;
                    openStartTime = now;

                    LCD_Clear(WHITE);
                    DrawCenteredStatus("opened", BLACK, WHITE);
                    prevColor = WHITE;
                }
                else
                {
                    isOpen = 0;
                    voiceFlag = 0;
                    irFlag = 0;
                    prevColor = 0xFFFFU;
                }
            }
            continue;
        }

        /* =====================================================
           (B) OPEN 상태 처리
           - 초음파 비활성
           - 3초 후 자동 닫힘 (연장 없음)
           ===================================================== */
        if (isOpen)
        {
            DrawCenteredStatus("opened", BLACK, WHITE);

            if ((uint32_t)(now - openStartTime) >= 3000U)
            {
                isMoving = 1;
                moveToOpen = 0;

                setServoPulse(SERVO_CLOSE);
                moveDoneTime = now + SERVO_MOVE_TIME_MS;

                voiceFlag = 0;
                irFlag = 0;
            }

            continue;
        }

        /* =====================================================
           (C) CLOSED 상태 처리
           - IR/초음파 활성
           - 음성 우선 → IR 순서로 트리거 검사
           ===================================================== */
        
        /* (C-1) 음성 트리거 우선 검사 */
        if (voiceFlag == 1)
        {
            if ((isOpen == 0) && (isMoving == 0))
            {
                isMoving = 1;
                moveToOpen = 1;

                setServoPulse(SERVO_OPEN);
                moveDoneTime = now + SERVO_MOVE_TIME_MS;

                voiceFlag = 0;
            }
            continue;
        }

        /* (C-2) IR 센서 읽기 및 플래그 설정 */
        {
            uint16_t ir_value = Get_Adc_Value();
            if (ir_value > IR_THRESHOLD)
                irFlag = 1;
            else
                irFlag = 0;
        }

        /* (C-3) IR 트리거 검사 (음성 플래그가 0일 때만) */
        if ((voiceFlag == 0) && (irFlag == 1) && (isOpen == 0) && (isMoving == 0))
        {
            isMoving = 1;
            moveToOpen = 1;

            setServoPulse(SERVO_OPEN);
            moveDoneTime = now + SERVO_MOVE_TIME_MS;

            irFlag = 0;
            continue;
        }

        /* (C-4) 초음파 측정 및 LCD 업데이트 (CLOSED 상태에서만) */
        if ((int32_t)(now - nextUltrasonicTime) >= 0)
        {
            uint32_t dist_cm = Get_Distance_cm();
            uint16_t color;

            if (dist_cm == 0U)   dist_cm = 999U;
            if (dist_cm > 999U)  dist_cm = 999U;

            color = BgColorFromDistance(dist_cm);

            if (color != prevColor)
            {
                LCD_Clear(color);
                prevColor = color;
            }

            DrawCenteredStatus(StatusTextFromDistance(dist_cm), WHITE, color);

            nextUltrasonicTime = now + 60U;
        }
    }
}

/* =========================================================
   Init - 전체 초기화 통합
   ========================================================= */
void Init(void)
{
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit();

    TIM_Configure();  /* TIM3: 서보 PWM */
    TimerInit();      /* TIM2: 초음파 1us tick */
    
    UART4_Init();     /* 음성인식 UART 초기화 */
    USART2_Init();    /* 블루투스 UART 초기화 */
    NVIC_Configure(); /* 인터럽트 설정 */

    LCD_Init();

    setServoPulse(SERVO_CLOSE);
    
    /* 모터 초기 상태: 정지 */
    GPIO_ResetBits(GPIOC, MOTOR_ALL_PINS);
}

/* =========================================================
   RCC 클럭 설정
   - 기존: GPIOA/B/C/D, ADC1, TIM2/3, AFIO, UART4
   - 추가: USART2 (APB1)
   ========================================================= */
void RccInit(void)
{
    /* APB2: GPIO, ADC, AFIO */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_ADC1  |
        RCC_APB2Periph_AFIO,
        ENABLE
    );

    /* APB1: TIM2, TIM3, UART4, USART2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  /* 블루투스용 */

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}

/* =========================================================
   GPIO 설정
   - 기존: PA1(IR), PB0(서보), PB10/11(초음파), PC10/11(UART4), PD2/3(LED)
   - 추가: PD5/6(USART2 리맵), PC6/7/8/9(모터)
   ========================================================= */
void GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* IR 센서 (PA1) - Analog Input */
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 서보모터 PWM (PB0) - Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 초음파 Trig (PB10) - Output Push-Pull */
    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    /* 초음파 Echo (PB11) - Floating Input */
    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

    /* LED (PD2, PD3) - Output Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOD, GPIO_Pin_3);

    /* =========================================================
       UART4 핀 설정 - 음성인식 (voice.c 기준)
       - PC10: TX, PC11: RX (리맵 불필요)
       ========================================================= */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* =========================================================
       USART2 핀 설정 - 블루투스 (motor.c 기준)
       - PD5: TX, PD6: RX (리맵 필요)
       ========================================================= */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);  /* PA2/3 → PD5/6 리맵 */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;        /* TX */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;        /* RX */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* =========================================================
       모터 제어 핀 설정 (motor.c 기준)
       - PC6/7: 오른쪽 모터
       - PC5/9: 왼쪽 모터
       ========================================================= */
    GPIO_InitStructure.GPIO_Pin = MOTOR_ALL_PINS;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* =========================================================
   ADC 초기화 (IR 센서용)
   ========================================================= */
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

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/* =========================================================
   TIM3 설정 - 서보모터 PWM (50Hz, 20ms 주기)
   ========================================================= */
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

/* =========================================================
   UART4 초기화 - 음성인식 모듈 (voice.c 기준)
   - 115200bps, 8N1, RX 인터럽트 활성화
   ========================================================= */
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

/* =========================================================
   USART2 초기화 - 블루투스 모터 제어 (motor.c 기준)
   - 9600bps, 8N1, RX 인터럽트 활성화
   ========================================================= */
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

/* =========================================================
   NVIC 설정 - UART4 + USART2 인터럽트
   ========================================================= */
void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* UART4 인터럽트 (음성인식) */
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART2 인터럽트 (블루투스 모터) - 높은 우선순위로 즉시 반응 */
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  /* 가장 높은 우선순위 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* =========================================================
   유틸리티 함수들
   ========================================================= */
uint16_t Get_Adc_Value(void)
{
    return ADC_GetConversionValue(ADC1);
}

void setServoPulse(uint16_t pulse)
{
    TIM_SetCompare3(TIM3, pulse);
}

/* =========================================================
   TIM2 설정 - 초음파 1us 타이밍
   ========================================================= */
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
    while (TIM_GetCounter(TIM2) < us) { }
}

/* =========================================================
   초음파 거리 측정 (cm 단위)
   ========================================================= */
uint32_t Get_Distance_cm(void)
{
    uint32_t t_us;

    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
    Delay_us(2);

    GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

    TIM_SetCounter(TIM2, 0);
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == RESET)
    {
        if ((uint32_t)TIM_GetCounter(TIM2) > ECHO_TIMEOUT_US)
            return 0U;
    }

    TIM_SetCounter(TIM2, 0);
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == SET)
    {
        if ((uint32_t)TIM_GetCounter(TIM2) > ECHO_TIMEOUT_US)
            break;
    }

    t_us = (uint32_t)TIM_GetCounter(TIM2);
    
    return (t_us / 58U);
}
