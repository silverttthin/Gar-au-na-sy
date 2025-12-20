/*******************************************************************************
 * main.c - 스마트 쓰레기통 펌웨어
 * 
 * 구현 기능:
 *   1. 적외선(IR) 센서로 뚜껑 서보모터 제어
 *   2. 초음파 센서로 적재량 측정 → LCD 색상 안내
 * 
 * 핀 매핑 (모듈 테스트 코드 기준):
 *   - PA1: IR 센서 (ADC1 Channel 1)
 *   - PB0: 서보모터 PWM (TIM3 CH3)
 *   - PB10: 초음파 Trig
 *   - PB11: 초음파 Echo
 *   - PD2, PD3: LED
 * 
 * 타이머 사용:
 *   - TIM2: 초음파 us 딜레이용 (Prescaler=71 → 1us tick)
 *   - TIM3: 서보 PWM (20ms 주기, 50Hz)
 *   - SysTick: ms tick (비블로킹 타이머용)
 ******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h"

/*******************************************************************************
 * 상수 정의 (모듈 테스트 코드에서 가져옴)
 ******************************************************************************/

/* IR 센서 설정 (servo_motor_with_IR.c 기준) */
#define IR_SENSOR_PIN       GPIO_Pin_1      // PA1
#define IR_THRESHOLD        2500            // 감지 기준값

/* 서보모터 펄스폭 (servo_motor_with_IR.c 기준) */
#define SERVO_CLOSE         700             // 0도 (닫힘)
#define SERVO_OPEN          2300            // 90도 (열림)
#define SERVO_MOVE_TIME_MS  500             // 서보 동작 예상 시간 (ms)

/* 초음파 센서 설정 (ultrasonic_senser.c 기준) */
#define US_TRIG_PIN         GPIO_Pin_10     // PB10
#define US_ECHO_PIN         GPIO_Pin_11     // PB11
#define US_GPIO_PORT        GPIOB
#define ECHO_TIMEOUT_US     30000

/* LCD 설정 */
#ifndef LCD_SCREEN_W
  #define LCD_SCREEN_W      240
#endif
#ifndef LCD_SCREEN_H
  #define LCD_SCREEN_H      320
#endif
#ifndef ORANGE
  #define ORANGE            0xFD20
#endif

/* 타이밍 상수 */
#define OPEN_DURATION_MS    3000            // 뚜껑 열림 유지 시간
#define MAIN_LOOP_DELAY_MS  100             // 메인 루프 딜레이

/*******************************************************************************
 * 전역 변수
 ******************************************************************************/

/* SysTick ms 카운터 (비블로킹 타이머용) */
volatile uint32_t ms_tick = 0;

/* 상태 플래그 */
volatile uint8_t isOpen = 0;        // 0=CLOSED, 1=OPEN
volatile uint8_t isMoving = 0;      // 0=정지, 1=서보 동작중
volatile uint8_t irFlag = 0;        // IR 감지 이벤트 플래그

/* 타이밍 변수 */
uint32_t openStartTime = 0;         // 뚜껑 열림 시작 시각 (ms)
uint32_t servoStartTime = 0;        // 서보 동작 시작 시각 (ms)

/* 서보 PWM 설정용 (모듈 코드 구조체 재사용) */
TIM_OCInitTypeDef TIM_OCInitStructure;

/* LCD 이전 배경색 (불필요한 갱신 방지) */
uint16_t prevBgColor = 0xFFFF;

/*******************************************************************************
 * 함수 프로토타입
 ******************************************************************************/

/* 초기화 함수들 */
void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void TimerInit(void);
void SysTickInit(void);

/* 유틸리티 함수들 */
uint32_t GetTick(void);
void Delay_us(uint32_t us);
uint16_t Get_Adc_Value(void);
uint32_t Get_Distance_cm(void);
void setServoPulse(uint16_t pulse);

/* LCD 함수들 */
uint16_t BgColorFromDistance(uint32_t dist_cm);
void UpdateLCD(void);

/*******************************************************************************
 * SysTick 핸들러 - 1ms마다 호출되어 ms_tick 증가
 ******************************************************************************/
void SysTick_Handler(void)
{
    ms_tick++;
}

/*******************************************************************************
 * 현재 ms tick 반환
 ******************************************************************************/
uint32_t GetTick(void)
{
    return ms_tick;
}

/*******************************************************************************
 * 메인 함수
 ******************************************************************************/
int main(void)
{
    uint16_t ir_value = 0;
    
    /* 모든 주변장치 초기화 */
    Init();
    
    /* 초기 상태 설정 */
    isOpen = 0;
    isMoving = 0;
    irFlag = 0;
    
    /* 초기 화면: 닫힌 상태 → 초음파로 색상 결정 */
    prevBgColor = 0xFFFF;  // 첫 갱신 강제
    
    /* 초기 서보 위치: 닫힘 */
    setServoPulse(SERVO_CLOSE);
    
    while(1)
    {
        /*------------------------------------------------------------------
         * 1. 서보 동작 완료 체크 (비블로킹)
         *    - isMoving==1인 상태에서 SERVO_MOVE_TIME_MS 경과 시 완료 처리
         *------------------------------------------------------------------*/
        if (isMoving == 1)
        {
            if ((GetTick() - servoStartTime) >= SERVO_MOVE_TIME_MS)
            {
                isMoving = 0;
                
                if (isOpen == 1)
                {
                    /* 열기 완료: openStartTime 기록 */
                    openStartTime = GetTick();
                }
                /* 닫기 완료 시 isOpen은 이미 0으로 설정됨 */
            }
            
            /* 서보 동작 중에는 새로운 트리거 무시 → 플래그 정리 */
            irFlag = 0;
            
            /* 서보 동작 중에는 나머지 로직 건너뜀 */
            continue;
        }
        
        /*------------------------------------------------------------------
         * 2. OPEN 상태: 3초 후 자동 닫기
         *------------------------------------------------------------------*/
        if (isOpen == 1)
        {
            /* LCD: 열림 상태 → 흰색 배경 */
            if (prevBgColor != WHITE)
            {
                LCD_Clear(WHITE);
                prevBgColor = WHITE;
            }
            
            /* 3초 경과 체크 (비블로킹) */
            if ((GetTick() - openStartTime) >= OPEN_DURATION_MS)
            {
                /* 닫기 실행 */
                isMoving = 1;
                isOpen = 0;
                servoStartTime = GetTick();
                setServoPulse(SERVO_CLOSE);
                
                /* 플래그 잔재 정리 */
                irFlag = 0;
            }
            
            continue;
        }
        
        /*------------------------------------------------------------------
         * 3. CLOSED 상태: IR 센서 + 초음파 센서 동작
         *------------------------------------------------------------------*/
        
        /* 3-1. 초음파로 거리 측정 → LCD 색상 변경 */
        UpdateLCD();
        
        /* 3-2. IR 센서 값 읽기 → irFlag 설정 */
        ir_value = Get_Adc_Value();
        if (ir_value > IR_THRESHOLD)
        {
            irFlag = 1;
        }
        
        /* 3-3. IR 트리거 처리 */
        if (irFlag == 1 && isOpen == 0 && isMoving == 0)
        {
            /* 뚜껑 열기 실행 */
            isMoving = 1;
            isOpen = 1;  /* 열림 상태로 전환 (완료 후 openStartTime 기록) */
            servoStartTime = GetTick();
            setServoPulse(SERVO_OPEN);
            
            /* 이벤트 플래그 소비 */
            irFlag = 0;
        }
        
        /* 메인 루프 딜레이 (비블로킹 - 폴링 주기 조절용) */
        {
            uint32_t loopStart = GetTick();
            while ((GetTick() - loopStart) < MAIN_LOOP_DELAY_MS);
        }
    }
}

/*******************************************************************************
 * 초기화 함수들
 ******************************************************************************/

void Init(void)
{
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit();
    TimerInit();
    SysTickInit();
    
    LCD_Init();
    
    /* LED 초기화 (모듈 코드 기준) */
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOD, GPIO_Pin_3);
}

void RccInit(void)
{
    /* servo_motor_with_IR.c + ultrasonic_senser.c 통합 */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |   // IR 센서 (PA1)
        RCC_APB2Periph_GPIOB |   // 서보(PB0), 초음파(PB10,PB11)
        RCC_APB2Periph_GPIOD |   // LED
        RCC_APB2Periph_ADC1  |   // IR ADC
        RCC_APB2Periph_AFIO,
        ENABLE
    );
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // 초음파 타이밍
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // 서보 PWM
}

void GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 1. IR 센서 (PA1) - Analog Input (servo_motor_with_IR.c 기준) */
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 2. 서보모터 (PB0) - AF Push-Pull (servo_motor_with_IR.c 기준) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 3. 초음파 Trig (PB10) - Output Push-Pull (ultrasonic_senser.c 기준) */
    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);
    
    /* 4. 초음파 Echo (PB11) - Input Floating (ultrasonic_senser.c 기준) */
    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);
    
    /* 5. LED (PD2, PD3) - Output Push-Pull (servo_motor_with_IR.c 기준) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /* 초음파 Trig 초기값 LOW */
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
}

void AdcInit(void)
{
    /* servo_motor_with_IR.c 기준 그대로 */
    ADC_InitTypeDef ADC_InitStructure;
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* Channel 1 (PA1) */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
    
    /* ADC 활성화 및 캘리브레이션 */
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TimerInit(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInit_Local;
    uint16_t prescale;
    
    /*------------------------------------------------------------------
     * TIM2: 초음파 us 딜레이용 (ultrasonic_senser.c 기준)
     * 72MHz / 72 = 1MHz → 1us tick
     *------------------------------------------------------------------*/
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
    
    /*------------------------------------------------------------------
     * TIM3: 서보모터 PWM (servo_motor_with_IR.c 기준)
     * CH3 (PB0), 20ms 주기 (50Hz)
     *------------------------------------------------------------------*/
    prescale = (uint16_t)(SystemCoreClock / 1000000);  // 1MHz clock
    
    TIM_TimeBaseStructure.TIM_Period = 20000;          // 20ms
    TIM_TimeBaseStructure.TIM_Prescaler = prescale;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCInit_Local.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInit_Local.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInit_Local.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInit_Local.TIM_Pulse = SERVO_CLOSE;
    TIM_OC3Init(TIM3, &TIM_OCInit_Local);
    
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void SysTickInit(void)
{
    /* SysTick 1ms 인터럽트 설정 */
    SysTick_Config(SystemCoreClock / 1000);
}

/*******************************************************************************
 * 유틸리티 함수들
 ******************************************************************************/

/* us 딜레이 (ultrasonic_senser.c 기준 - TIM2 폴링) */
void Delay_us(uint32_t us)
{
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < us);
}

/* IR ADC 값 읽기 (servo_motor_with_IR.c 기준) */
uint16_t Get_Adc_Value(void)
{
    return ADC_GetConversionValue(ADC1);
}

/* 초음파 거리 측정 (ultrasonic_senser.c 기준, cm 반환) */
uint32_t Get_Distance_cm(void)
{
    uint32_t t_us;
    
    /* Trig: 10us 펄스 */
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
    Delay_us(2);
    GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
    
    /* Echo HIGH 대기 (타임아웃) */
    TIM_SetCounter(TIM2, 0);
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == RESET)
    {
        if (TIM_GetCounter(TIM2) > ECHO_TIMEOUT_US)
            return 0;
    }
    
    /* Echo HIGH 폭 측정 (타임아웃) */
    TIM_SetCounter(TIM2, 0);
    while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == SET)
    {
        if (TIM_GetCounter(TIM2) > ECHO_TIMEOUT_US)
            break;
    }
    
    t_us = TIM_GetCounter(TIM2);
    
    /* cm = t_us / 58 */
    return (t_us / 58);
}

/* 서보모터 펄스 설정 (servo_motor_with_IR.c 기준) */
void setServoPulse(uint16_t pulse)
{
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

/*******************************************************************************
 * LCD 함수들
 ******************************************************************************/

/* 거리 → 배경색 (ultrasonic_senser.c 기준)
 * 파란색(11~): Normal
 * 주황색(6~10): Caution
 * 빨간색(~5): Warning
 */
uint16_t BgColorFromDistance(uint32_t dist_cm)
{
    if (dist_cm <= 5)
        return RED;
    else if (dist_cm <= 10)
        return ORANGE;
    else
        return BLUE;
}

/* LCD 화면 갱신 (CLOSED 상태에서만 호출) */
void UpdateLCD(void)
{
    uint32_t dist_cm;
    uint16_t bgColor;
    
    /* 초음파 거리 측정 */
    dist_cm = Get_Distance_cm();
    
    /* 타임아웃/비정상값 처리 */
    if (dist_cm == 0) dist_cm = 999;
    if (dist_cm > 999) dist_cm = 999;
    
    /* 배경색 결정 */
    bgColor = BgColorFromDistance(dist_cm);
    
    /* 배경색이 바뀐 경우에만 화면 갱신 */
    if (bgColor != prevBgColor)
    {
        LCD_Clear(bgColor);
        prevBgColor = bgColor;
    }
}

