#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h" // 타이머 헤더 추가
#include "misc.h"
#include "lcd.h"
#include "touch.h"

// LCD 좌표 설정
#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_COORD_X_X   40
#define LCD_COORD_X_Y   70
#define LCD_COORD_Y_X   40
#define LCD_COORD_Y_Y   90
#define LCD_DIST_VAL_X  20  // 위치 이름 변경 (LUX -> DIST)
#define LCD_DIST_VAL_Y  110

// 초음파 센서 핀 설정 (GPIOB 사용)
#define US_TRIG_PIN     GPIO_Pin_10
#define US_ECHO_PIN     GPIO_Pin_11
#define US_GPIO_PORT    GPIOB

void Init(void);
void RccInit(void);
void GpioInit(void);
void TimerInit(void); // ADC 대신 타이머 초기화
void Delay_us(uint32_t us);
uint32_t Get_Distance(void);

const int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

int main(){
    uint16_t pos_x, pos_y;
    uint16_t pix_x, pix_y;
    uint32_t distance = 0;

    Init();

    LCD_Clear(WHITE);

    // 팀 이름 출력
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "THU_TEAM04", BLUE, WHITE);
    LCD_ShowString(LCD_DIST_VAL_X, LCD_DIST_VAL_Y, "DIST(cm):", BLUE, WHITE);

    while(1){
        // 1. 초음파 거리 측정
        distance = Get_Distance();

        // 2. 거리 값 LCD 출력
        // 이전 값이 남아있지 않게 하기 위해 공백으로 덮어쓰거나 배경색으로 지우는 로직이 있으면 좋음
        LCD_ShowNum(LCD_DIST_VAL_X + 80, LCD_DIST_VAL_Y, distance, 4, RED, WHITE);

        // 3. 터치 좌표 읽기
        Touch_GetXY(&pos_x, &pos_y, 1);
        Convert_Pos(pos_x, pos_y, &pix_x, &pix_y);

        // 4. 좌표 값 출력
        LCD_ShowString(LCD_COORD_X_X, LCD_COORD_X_Y, "X:", BLUE, WHITE);
        LCD_ShowNum(LCD_COORD_X_X + 20, LCD_COORD_X_Y, (u32)pix_x, 3, BLUE, WHITE);

        LCD_ShowString(LCD_COORD_Y_X, LCD_COORD_Y_Y, "Y:", BLUE, WHITE);
        LCD_ShowNum(LCD_COORD_Y_X + 20, LCD_COORD_Y_Y, (u32)pix_y, 3, BLUE, WHITE);

        // 5. 터치 지점에 원 그리기 (터치가 감지되었을 때만 그리는 것이 좋음)
        // 터치 좌표가 유효한 범위인지 간단히 체크 (예: 0이 아니면)
        if(pix_x > 0 && pix_y > 0) {
            LCD_DrawCircle(pix_x, pix_y, 5);
        }
        
        // 너무 빠른 측정을 방지하기 위한 딜레이 (약 50ms~100ms)
        // Delay_us(50000); 
    }
}

void Init(void) {
    SystemInit();
    RccInit();
    GpioInit();
    TimerInit(); // ADC 대신 타이머 초기화
    // NvicInit(); // 초음파는 폴링 방식을 사용하므로 인터럽트 불필요 (터치용 필요시 유지)

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
}

void RccInit(void) {
    // 타이머2와 GPIOB 클럭 활성화
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GpioInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Trig Pin (PB10) -> Output Push-Pull
    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

    // Echo Pin (PB11) -> Input Floating (or Pull-Down)
    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);
    
    // Trig 핀 초기화 (Low)
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
}

// 타이머 초기화 (1us 단위 카운팅을 위해 설정)
void TimerInit(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // SystemCoreClock은 보통 72MHz
    // Prescaler = 72 - 1 = 71 => 1MHz (1us)
    TIM_TimeBaseStructure.TIM_Prescaler = 71; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 최대 카운트
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM2, ENABLE);
}

// 마이크로초 단위 딜레이 (TIM2 사용)
void Delay_us(uint32_t us) {
    TIM_SetCounter(TIM2, 0);
    while (TIM_GetCounter(TIM2) < us);
}

// 거리 측정 함수 (반환값: cm)
uint32_t Get_Distance(void) {
    uint32_t time = 0;
    
    // 1. Trig 신호 발생 (10us High Pulse)
    GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

    // 2. Echo 신호가 High가 될 때까지 대기 (Timeout 적용)
    // 무한 루프 방지를 위해 카운터 등을 사용할 수 있음
    while(GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == RESET);

    // 3. 시간 측정 시작
    TIM_SetCounter(TIM2, 0);

    // 4. Echo 신호가 Low가 될 때까지 대기 (펄스 폭 측정)
    while(GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == SET);

    // 5. 시간 읽기
    time = TIM_GetCounter(TIM2);

    // 6. 거리 계산 (음속: 340m/s -> 34000cm/s -> 0.034cm/us)
    // 왕복이므로 2로 나눔. time * 0.034 / 2  => time / 58 (약식)
    return (time / 58);
}