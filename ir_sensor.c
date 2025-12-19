#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h" // LCD 라이브러리가 포함되어 있어야 합니다.

// ==========================================
// 설정: LCD 위치 및 GPIO 핀
// ==========================================
#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_PIR_LABEL_X 20
#define LCD_PIR_LABEL_Y 90
#define LCD_STATUS_X    20
#define LCD_STATUS_Y    120

// PIR 센서 핀 설정 (GPIOB PB10)
#define PIR_SENSOR_PIN  GPIO_Pin_10
#define PIR_GPIO_PORT   GPIOB

// ==========================================
// 함수 원형 선언
// ==========================================
void Init(void);
void RccInit(void);
void GpioInit(void);
void TimerInit(void);
void Delay_us(uint16_t us);
void Delay_ms(uint32_t ms); // 긴 딜레이를 위한 밀리초 함수 추가

// 색상 배열 (필요시 사용)
const int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

// ==========================================
// 메인 함수
// ==========================================
int main(){
    Init();

    LCD_Clear(WHITE);

    // 고정된 UI 문자열 출력
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "THU_TEAM04", BLUE, WHITE);
    LCD_ShowString(LCD_PIR_LABEL_X, LCD_PIR_LABEL_Y, "PIR Motion Sensor", BLACK, WHITE);

    while(1){
        // 1. 센서 값 읽기 (PB10)
        // HC-SR501은 움직임 감지 시 HIGH(1) 출력
        if (GPIO_ReadInputDataBit(PIR_GPIO_PORT, PIR_SENSOR_PIN) == Bit_SET) {
            
            // [감지됨] 빨간색 글씨
            LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "MOTION DETECTED!", RED, WHITE);
            
        } else {
            
            // [감지 안됨] 파란색 글씨
            // 뒤에 공백("     ")을 두어 이전 글자(DETECTED!)의 잔상을 지움
            LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "Scanning...     ", BLUE, WHITE);
            
        }
        
        // 2. 루프 딜레이
        // 기존 Delay_us(100000)은 타이머 오버플로우가 발생하므로
        // 새로 만든 Delay_ms(100)을 사용합니다.
        Delay_ms(100); 
    }
}

// ==========================================
// 초기화 함수들
// ==========================================
void Init(void) {
    SystemInit();
    RccInit();
    GpioInit();
    TimerInit(); 
    
    LCD_Init(); // LCD 하드웨어 초기화 (내부에서 GPIO 클럭 켜는지 확인 필요)
}

void RccInit(void) {
    // 타이머2(Delay용)와 GPIOB(센서용) 클럭 활성화
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GpioInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // PIR Sensor Pin (PB10) 설정
    // Input Pull-Down: 센서 연결이 끊기거나 신호가 없을 때 0V(Low)를 유지하도록 함
    GPIO_InitStructure.GPIO_Pin = PIR_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_Init(PIR_GPIO_PORT, &GPIO_InitStructure);
}

void TimerInit(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // SystemCoreClock 72MHz 기준
    // Prescaler 71 -> (72MHz / (71+1)) = 1MHz (1us 틱)
    TIM_TimeBaseStructure.TIM_Prescaler = 71; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // Period를 최대값(0xFFFF)으로 설정하여 프리런닝 모드로 동작
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM2, ENABLE);
}

// ==========================================
// 시간 지연 함수들
// ==========================================

// [수정됨] 마이크로초 단위 딜레이
// 주의: TIM2는 16비트이므로 최대 약 65ms(65535us)까지만 측정 가능
void Delay_us(uint16_t us) {
    TIM_SetCounter(TIM2, 0); // 카운터 0으로 리셋
    while (TIM_GetCounter(TIM2) < us); // 목표값 도달까지 대기
}

// [신규 추가] 밀리초 단위 딜레이
// 1ms를 반복 호출하여 긴 시간을 안전하게 지연시킴
void Delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        Delay_us(1000); // 1ms = 1000us
    }
}