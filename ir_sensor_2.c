#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h" // ADC 헤더 필수
#include "misc.h"
#include "lcd.h"

// ==========================================
// 핀 설정
// ==========================================
// 적외선 센서 (PA1 - ADC Channel 1)
#define IR_SENSOR_PIN   GPIO_Pin_1

// ==========================================
// LCD UI 좌표
// ==========================================
#define LCD_TITLE_X     20
#define LCD_TITLE_Y     50
#define LCD_IR_VAL_X    20
#define LCD_IR_VAL_Y    100
#define LCD_STATE_X     20
#define LCD_STATE_Y     130

// ==========================================
// 함수 선언
// ==========================================
void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
uint16_t Get_Adc_Value(void);

int main(){
    uint16_t ir_value = 0;

    Init(); // 초기화

    LCD_Clear(WHITE);
    LCD_ShowString(LCD_TITLE_X, LCD_TITLE_Y, "IR SENSOR TEST", BLUE, WHITE);
    LCD_ShowString(LCD_IR_VAL_X, LCD_IR_VAL_Y, "ADC VAL:", BLACK, WHITE);

    while(1){
        // 1. 적외선 센서 값 읽기 (0 ~ 4095)
        ir_value = Get_Adc_Value();

        // 2. 값 LCD 출력
        LCD_ShowNum(LCD_IR_VAL_X + 90, LCD_IR_VAL_Y, ir_value, 4, RED, WHITE);

        // 3. 값에 따른 상태 판단 (임계값 튜닝 필요)
        // 예: 값이 2000 이상이면 물체 감지 (센서 특성에 따라 부등호 반대일 수 있음)
        if(ir_value > 2000) { 
             LCD_ShowString(LCD_STATE_X, LCD_STATE_Y, "DETECTED!   ", RED, WHITE);
        } else {
             LCD_ShowString(LCD_STATE_X, LCD_STATE_Y, "NO OBJECT   ", GREEN, WHITE);
        }

        // 딜레이
        for(volatile int i=0; i<50000; i++);
    }
}

void Init(void) {
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit(); // ADC 초기화 추가
    LCD_Init();
}

void RccInit(void) {
    // GPIOA(PA1용) 및 ADC1 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
}

void GpioInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // PA1 (ADC 입력 모드)
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog Input
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void AdcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC 설정
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;     // 1개 채널만 사용
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 연속 변환 모드 (계속 읽음)
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // 채널 1 (PA1) 설정, 샘플링 타임 설정
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

    // ADC 활성화 및 보정(Calibration)
    ADC_Cmd(ADC1, ENABLE);
    
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // 변환 시작
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t Get_Adc_Value(void) {
    // 변환된 값 반환
    return ADC_GetConversionValue(ADC1);
}