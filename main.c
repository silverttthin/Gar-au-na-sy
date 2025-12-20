#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
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
   상태/플래그
   ========================================================= */
static volatile int isOpen = 0;        /* 0=CLOSED, 1=OPEN */
static volatile int isMoving = 0;      /* 0/1 */
static volatile int voiceFlag = 0;     /* 이번 구현 범위에서 사용 안 함 */
static volatile int irFlag = 0;        /* 0/1 */
static volatile uint32_t openStartTime = 0;  /* ms tick */

static volatile uint32_t moveDoneTime = 0;  /* ms tick */
static volatile int moveToOpen = 0;         /* 1=open 완료 대기, 0=close 완료 대기 */

static TIM_OCInitTypeDef TIM_OCInitStructure;

/* =========================================================
   프로토타입
   ========================================================= */
void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void TIM_Configure(void);

uint16_t Get_Adc_Value(void);
void setServoPulse(uint16_t pulse);

void TimerInit(void);
void Delay_us(uint32_t us);
uint32_t Get_Distance_cm(void);

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

int main(void)
{
    uint16_t prevColor = 0xFFFFU;
    uint32_t nextUltrasonicTime = 0U;

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

        /* (A) 서보 이동 완료 처리 */
        if (isMoving)
        {
            voiceFlag = 0;
            irFlag = 0;

            if ((int32_t)(now - moveDoneTime) >= 0)
            {
                isMoving = 0;

                if (moveToOpen)
                {
                    /* 오픈 완료 */
                    isOpen = 1;
                    openStartTime = now;

                    /* OPEN 상태 LCD: 전체 하얀색 + opened 출력 */
                    LCD_Clear(WHITE);
                    DrawCenteredStatus("opened", BLACK, WHITE);  /* <-- 추가 */
                    prevColor = WHITE;
                }
                else
                {
                    /* 닫힘 완료 */
                    isOpen = 0;
                    voiceFlag = 0;
                    irFlag = 0;
                    prevColor = 0xFFFFU;
                }
            }
            continue;
        }

        /* (B) OPEN 상태: opened 유지 출력 + 3초 후 자동 닫힘 */
        if (isOpen)
        {
            /* 화면이 다른 요소로 덮일 가능성을 대비해 OPEN 동안에도 주기적으로 출력 */
            DrawCenteredStatus("opened", BLACK, WHITE);          /* <-- 추가 */

            if ((uint32_t)(now - openStartTime) >= 3000U)
            {
                isMoving = 1;
                moveToOpen = 0;

                setServoPulse(SERVO_CLOSE);
                moveDoneTime = now + SERVO_MOVE_TIME_MS;

                voiceFlag = 0;
                irFlag = 0;
            }

            /* OPEN 상태에서는 초음파 비활성 */
            continue;
        }

        /* (C) CLOSED 상태 */
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

        {
            uint16_t ir_value = Get_Adc_Value();
            if (ir_value > IR_THRESHOLD) irFlag = 1;
            else                         irFlag = 0;
        }

        if ((voiceFlag == 0) && (irFlag == 1) && (isOpen == 0) && (isMoving == 0))
        {
            isMoving = 1;
            moveToOpen = 1;

            setServoPulse(SERVO_OPEN);
            moveDoneTime = now + SERVO_MOVE_TIME_MS;

            irFlag = 0;
            continue;
        }

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
   Init / RCC / GPIO / ADC / TIM
   ========================================================= */
void Init(void)
{
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit();

    TIM_Configure();  /* TIM3: 서보 PWM */
    TimerInit();      /* TIM2: 초음파 1us tick */

    LCD_Init();

    setServoPulse(SERVO_CLOSE);
}

void RccInit(void)
{
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_ADC1  |
        RCC_APB2Periph_AFIO,
        ENABLE
    );

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}

void GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOD, GPIO_Pin_3);
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

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

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

uint16_t Get_Adc_Value(void)
{
    return ADC_GetConversionValue(ADC1);
}

void setServoPulse(uint16_t pulse)
{  
    TIM_SetCompare3(TIM3, pulse);
}

/* Ultrasonic */
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
