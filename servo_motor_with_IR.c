#include "stm32f10x.h"
#include "stm32f10x_adc.h" // ADC 헤더 추가
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"
#include "math.h"

// ==========================================
// 설정 및 상수 정의
// ==========================================
#define IR_SENSOR_PIN   GPIO_Pin_1  // PA1 사용
#define IR_THRESHOLD    2500        // 감지 기준값

// 서보모터 펄스폭 정의 (하드웨어에 따라 미세조정 필요)
#define SERVO_CLOSE     700         // 0도 (닫힘)
#define SERVO_OPEN      2300        // 90도 (열림)

// LCD UI 좌표
#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_STATUS_X    20
#define LCD_STATUS_Y    80
#define LCD_VAL_X       20
#define LCD_VAL_Y       110

// ==========================================
// 함수 프로토타입
// ==========================================
void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);         // ADC 초기화 추가
void TIM_Configure(void);
void NvicInit(void);
void ledToggle(int num);
void setServoPulse(uint16_t pulse);
uint16_t Get_Adc_Value(void); // ADC 값 읽기 추가
void Delay_ms(uint32_t ms);   // 밀리초 딜레이 함수

// 전역 변수
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
int timer_counter = 0;
char ledOn = 0;

int main(){
    uint16_t ir_value = 0;
    
    Init(); // 초기화 (ADC 포함)

    ledOn = 1; 
    LCD_Clear(WHITE);

    // 초기 화면 UI
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "SMART BIN", BLUE, WHITE);
    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "State: CLOSED", BLACK, WHITE);
    LCD_ShowString(LCD_VAL_X, LCD_VAL_Y, "IR Val:", BLACK, WHITE);
    
    // 초기 상태: 뚜껑 닫기
    setServoPulse(SERVO_CLOSE);

    while(1){
        // 1. 적외선 센서 값 읽기
        ir_value = Get_Adc_Value();

        // 값 LCD 디버깅 출력
        LCD_ShowNum(LCD_VAL_X + 80, LCD_VAL_Y, ir_value, 4, RED, WHITE);

        // 2. 로직 구현
        if (ir_value > IR_THRESHOLD) {
            // [감지됨] -> 뚜껑 열기
            LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "State: OPEN  ", RED, WHITE);
            setServoPulse(SERVO_OPEN);
            
            // 3. 3초 대기 (요청하신 기능)
            // 이 시간 동안은 센서 값을 확인하지 않고 열린 상태 유지
            Delay_ms(3000); 
            
        } else {
            // [감지 안 됨] -> 뚜껑 닫기
            // 3초 대기 후 루프가 다시 돌았을 때 여기로 오게 됨
            LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "State: CLOSED", BLUE, WHITE);
            setServoPulse(SERVO_CLOSE);
            
            // 너무 빠른 반복 방지용 짧은 딜레이
            Delay_ms(100); 
        }
    }
}

// ==========================================
// 초기화 함수들
// ==========================================
void Init(void) {
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit();      // ADC 초기화 호출
    TIM_Configure();
    NvicInit();

    LCD_Init();
    
    // LEDs 초기화
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOD, GPIO_Pin_3);
}

void RccInit(void) {
    // GPIOA(ADC용), GPIOB(PWM용), GPIOD(LED용) 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1 | 
                           RCC_APB2Periph_AFIO, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Timer2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Timer3 (PWM)
}

void GpioInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. IR Sensor (PA1) -> Analog Input
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2. Servo Motor (PB0) -> Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. LEDs (PD2, PD3) -> Output Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void AdcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 설정
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 연속 변환
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // Channel 1 (PA1) 설정
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    // ADC 활성화 및 캘리브레이션
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TIM_Configure(void) {
    // TIM2: LED 점멸 및 딜레이용 기본 타이머
    TIM_TimeBaseInitTypeDef TIM2_InitStructure;
    TIM2_InitStructure.TIM_Period = 10000;
    TIM2_InitStructure.TIM_Prescaler = 7200; 
    TIM2_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM2_InitStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // TIM3: 서보모터 PWM 제어
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure_local;
    uint16_t prescale = (uint16_t)(SystemCoreClock / 1000000); // 1MHz clock

    TIM3_InitStructure.TIM_Period = 20000; // 20ms 주기 (50Hz)
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;

    TIM_OCInitStructure_local.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure_local.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure_local.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure_local.TIM_Pulse = SERVO_CLOSE; // 초기값

    TIM_OC3Init(TIM3, &TIM_OCInitStructure_local); // CH3 (PB0)

    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void NvicInit(void){
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ==========================================
// 유틸리티 함수들
// ==========================================

// ADC 값 읽기
uint16_t Get_Adc_Value(void) {
    return ADC_GetConversionValue(ADC1);
}

// 서보모터 위치 제어
void setServoPulse(uint16_t pulse){
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse; 
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

// 단순 딜레이 함수 (소프트웨어 루프 사용)
// 72MHz 기준 대략적인 ms 딜레이
void Delay_ms(uint32_t ms) {
    for(volatile uint32_t i = 0; i < ms; i++) {
        for(volatile uint32_t j = 0; j < 6000; j++); 
    }
}

void ledToggle(int num) {
    uint16_t pin;
    switch (num) {
        case 1: pin = GPIO_Pin_2; break;
        case 2: pin = GPIO_Pin_3; break;
        default: return;
    }
    if (GPIO_ReadOutputDataBit(GPIOD, pin) == Bit_RESET) GPIO_SetBits(GPIOD, pin);
    else GPIO_ResetBits(GPIOD, pin);
}

void TIM2_IRQHandler(void) {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        timer_counter++;
        // 동작 확인용 LED 깜빡임 (2초마다)
        if (ledOn && (timer_counter % 20 == 0)) { // 빈도 조절
             ledToggle(2);
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
