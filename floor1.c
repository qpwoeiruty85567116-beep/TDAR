// 자율주행 + 진동/화재/빗물 + 긴급 스위치 + PF0 정지 신호 통합 버전
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define SOUND_VELOCITY 340UL // 소리 속도 (m/sec)

// ===== 서보 각도 매크로 (실제 동작 기준) =====
#define SERVO_CENTER 200   // 직진 근처
#define SERVO_LEFT    90   // 실제 왼쪽으로 조향
#define SERVO_RIGHT  285   // 실제 오른쪽으로 조향

// 전면부 왼쪽 대각선 초음파 센서 (포트 F)
#define TRIGDIAL 6    // Trigger 신호 (출력 = PF6)
#define ECHODIAL 7    // Echo 신호 (입력 = PF7)
// 전면부 오른쪽 대각선 초음파 센서 (포트 F)
#define TRIGDIAR 2    // Trigger 신호 (출력 = PF2)
#define ECHODIAR 3    // Echo 신호 (입력 = PF3)

// 전면부 중앙 초음파 센서 (포트 E)
#define TRIGF 6       // Trigger 신호 (출력 = PE6)
#define ECHOF 7       // Echo 신호 (입력 = PE7)

// 왼쪽 면 초음파 센서 (포트 E)
#define TRIGFLL 4     // Trigger 신호 (출력 = PE4)
#define ECHOFLL 5     // Echo 신호 (입력 = PE5)
// 오른쪽 면 초음파 센서 (포트 E)
#define TRIGFRR 0     // Trigger 신호 (출력 = PE0)
#define ECHOFRR 1     // Echo 신호 (입력 = PE1)

// 후면부 왼쪽 초음파 센서 (포트 D)
#define TRIGBL 6      // Trigger 신호 (출력 = PD6)
#define ECHOBL 7      // Echo 신호 (입력 = PD7)
// 후면부 중앙 초음파 센서 (포트 D)
#define TRIGB 4       // Trigger 신호 (출력 = PD4)
#define ECHOB 5       // Echo 신호 (입력 = PD5)
// 후면부 오른쪽 초음파 센서 (포트 D)
#define TRIGBR 2      // Trigger 신호 (출력 = PD2)
#define ECHOBR 3      // Echo 신호 (입력 = PD3)

// 진동/화재 센서 & 스위치/LED
#define VIB_DDR     DDRC
#define VIB_PINR    PINC
#define VIB_FRONT   PC4

#define FIRE_DDR    DDRC
#define FIRE_PINR   PINC
#define FIRE_SENSOR PC5

#define SW_EMERGENCY PC0   // 긴급 회피 스위치 (PC0?GND)
#define LED_PIN      PA2   // 상태 LED

// 빗물 센서 (ADC1)
#define RAIN_ADC_CH   1
const uint16_t RAIN_TH_WET = 200;   // 빗물 기준 (필요시 조절)

// PF0 정지 신호 (ADC0) - 2V 이상이면 정지
#define STOP_SIGNAL_CH 0
// 5V 기준 2V → (2/5)*1023 ≒ 409, 약간 여유를 두고 410
const uint16_t STOP_TH_ADC = 410;

// ====== 함수 선언 ======
unsigned distanceDIA(int TRIG, int ECHO);
unsigned distanceFLR(int TRIG, int ECHO);
unsigned distanceBLR(int TRIG, int ECHO);

static void     ADC_init(void);
static uint16_t ADC_read(uint8_t channel);

// ====== 전역 상태 변수 ======
static unsigned char front_prev  = 0;
static unsigned char front_count = 0;
static unsigned char vib_done    = 0;
static unsigned char fire_prev   = 0;

// 긴급 스위치 1회용 플래그
static unsigned char emergency_done = 0;

// PF0가 한 번이라도 High(정지 조건)였는지 체크
static unsigned char pf0_high_ever = 0;

// ====== ADC 함수 구현 ======
static void ADC_init(void){
    ADMUX  = (1<<REFS0);                     // AVCC 기준 전압
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // ADC Enable, prescale=64
}

static uint16_t ADC_read(uint8_t channel){
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADC;
}

// ====== 메인 ======
int main(void)
{
    unsigned int speed      = 145;  // 전진 기본 속도
    unsigned int turnspeed  = 20;   // 회전 시 추가 속도
    unsigned int backspeed  = 165;  // 후진 기본 속도

    unsigned int DIAL;              // 전면부 왼쪽 대각 거리
    unsigned int DIAR;              // 전면부 오른쪽 대각 거리
    unsigned int distance;          // 전면부 중앙 거리
    unsigned int LEFT;              // 왼쪽 면 거리
    unsigned int RIGHT;             // 오른쪽 면 거리

    unsigned int BACKL;             // 후면부 왼쪽 거리
    unsigned int BACK;              // 후면부 중앙 거리
    unsigned int BACKR;             // 후면부 오른쪽 거리

    static unsigned int servo_cmd = SERVO_CENTER; // 현재 조향 명령
    signed char escape_side;                      // -1: 왼쪽, +1: 오른쪽, 0: 애매
    unsigned int reverse_steer;                  // 후진 시 사용한 서보 방향

    // 모터 및 서보 출력 설정
    DDRB = 0x30;     // PB4(OC0), PB5(OC1A) 출력
    DDRA = 0x07;     // PA0, PA1, PA2(LED) 출력

    // 진동/화재/스위치 입력 설정
    VIB_DDR  &= ~((1 << VIB_FRONT) | (1 << FIRE_SENSOR) | (1 << SW_EMERGENCY));
    // 긴급 스위치는 내부 풀업 (PC0?GND 스위치 가정)
    PORTC    |= (1 << SW_EMERGENCY);

    // 모터 PWM (Timer0) 설정: Fast PWM, non-inverting, prescale=1
    TCCR0 = (1<<WGM01) | (1<<WGM00) | (1<<COM01) | (1<<CS00);
    TCNT0 = 0;

    // 초음파 포트 설정: TRIG = 출력(1), ECHO = 입력(0)
    DDRD = 0x55;     // 0101 0101
    DDRE = 0x55;
    DDRF = 0x54;     // 0101 0100 (PF0: ADC용 입력 유지)

    // 서보모터 (Timer1) 설정
    TCCR1A = 0x82;
    TCCR1B = 0x1B;
    ICR1   = 4999;           // 20ms 주기
    OCR1A  = SERVO_CENTER;   // 직진

    // 화재 센서 초기 상태
    fire_prev = (FIRE_PINR & (1 << FIRE_SENSOR)) != 0;

    // ADC 초기화
    ADC_init();

    while(1){

        // =====================================================================
        // [0] PF0 정지 신호: 2V 이상 → 차량 정지 + LED 0.5초 간격 깜빡
        //     PF0가 LOW(2V 미만)로 돌아오면 LED 끄고 주행 재개
        // =====================================================================
        if (ADC_read(STOP_SIGNAL_CH) > STOP_TH_ADC)
        {
            // PF0가 한 번이라도 High였다는 의미 → 이후 속도 145 → 130 유지
            pf0_high_ever = 1;

            // 모터 정지
            OCR0 = 0;
            PORTA &= ~(1 << PA0);
            PORTA &= ~(1 << PA1);

            // PF0가 다시 LOW 될 때까지 LED를 0.5초 간격으로 깜빡
            while (ADC_read(STOP_SIGNAL_CH) > STOP_TH_ADC) {
                PORTA ^= (1 << LED_PIN);
                _delay_ms(500);
            }
            // LOW로 돌아오면 LED 끄기
            PORTA &= ~(1 << LED_PIN);
        }

        // =====================================================================
        // [1] 빗물 센서: 비 많이 오면 속도 약간 감소
        //     + PF0가 한 번이라도 High였으면 이후 기본속도는 145 → 130
        // =====================================================================
        {
            uint32_t rsum = 0;
            for(uint8_t i = 0; i < 8; i++){
                rsum += ADC_read(RAIN_ADC_CH);
                _delay_ms(1);
            }
            uint16_t rain = (uint16_t)(rsum / 8);

            if(rain > RAIN_TH_WET) {
                // 비가 오면 원래도 130
                speed = 130;
            }
            else {
                // 비가 안 와도 PF0가 한 번이라도 High였으면 130 유지
                if (pf0_high_ever)
                    speed = 130;
                else
                    speed = 145;
            }
        }

        // =====================================================================
        // [2] 전면부 거리 측정 (중앙 + 대각)
        // =====================================================================
        distance = distanceFLR(TRIGF, ECHOF);     // 전면부 중앙 거리
        _delay_ms(10);
        DIAL = distanceDIA(TRIGDIAL, ECHODIAL);   // 전면부 대각선 왼쪽
        _delay_ms(10);
        DIAR = distanceDIA(TRIGDIAR, ECHODIAR);   // 전면부 대각선 오른쪽
        _delay_ms(10);

        // 기본 전진 방향 (01)
        PORTA |=  (1 << PA0);
        PORTA &= ~(1 << PA1);

        // 좌/우 측면 거리 (도망 방향 판단용)
        LEFT  = distanceFLR(TRIGFLL, ECHOFLL);
        _delay_ms(10);
        RIGHT = distanceFLR(TRIGFRR, ECHOFRR);
        _delay_ms(10);

        escape_side = 0;

        // =====================================================================
        // [3] 진동 이벤트 (전면 충격 감지 → 1회만 특수 패턴)
        // =====================================================================
        {
            unsigned char front_raw = (VIB_PINR & (1 << VIB_FRONT)) != 0;

            if (!vib_done) {
                // 상승 에지 카운트
                if (!front_prev && front_raw) {
                    if (front_count < 255) front_count++;
                }

                if (front_count >= 3) {
                    vib_done = 1;

                    // 예시 패턴: 우측 회피 후 정지
                    PORTA |=  (1 << PA0);
                    PORTA &= ~(1 << PA1);
                    OCR0 = 180;

                    servo_cmd = SERVO_RIGHT;
                    OCR1A = servo_cmd;
                    for (unsigned char i = 0; i < 15; i++) _delay_ms(100);

                    servo_cmd = SERVO_CENTER;
                    OCR1A = servo_cmd;

                    PORTA &= ~(1 << PA0);
                    PORTA &= ~(1 << PA1);
                    OCR0 = 0;

                    _delay_ms(6000);     // 6초간 정지

                    front_count = 0;
                    front_prev  = front_raw;
                    continue;            // 다음 루프로
                }
            }
            front_prev = front_raw;
        }

        // =====================================================================
        // [4] 화재 이벤트 (PC5 상승 에지 시 특정 패턴 1회)
        // =====================================================================
        {
            unsigned char fire_raw = (FIRE_PINR & (1 << FIRE_SENSOR)) != 0;

            if (!fire_prev && fire_raw) {
                // 예시 패턴: 후진 + 우측 조향
                PORTA |=  (1 << PA1);
                PORTA &= ~(1 << PA0);
                OCR0 = 190;

                servo_cmd = SERVO_RIGHT;
                OCR1A = servo_cmd;
                for (unsigned char i = 0; i < 15; i++) _delay_ms(100);

                servo_cmd = SERVO_CENTER;
                OCR1A = servo_cmd;

                PORTA &= ~(1 << PA0);
                PORTA &= ~(1 << PA1);
                OCR0 = 0;

                _delay_ms(3000);     // 3초 정지
                fire_prev = fire_raw;
                continue;
            }
            fire_prev = fire_raw;
        }

        // =====================================================================
        // [5] 긴급 스위치 (PC0?GND) + 정면 거리 기반 "특수 회피" 루틴
        // =====================================================================
        if ( ((PINC & (1 << SW_EMERGENCY)) == 0) && (emergency_done == 0) )
        {
            if (distance < 350)   // 35cm 이내면 긴급 회피 발동
            {
                emergency_done = 1;   // 1회용 (테스트용이면 잠깐 주석 처리 가능)

                // --- (1) 급정지 ---
                OCR0 = 0;
                PORTA &= ~(1 << PA0);
                PORTA &= ~(1 << PA1);

                uint8_t wait_timer  = 0;
                uint8_t path_cleared = 0;

                // --- (2) 최대 5초 대기 → LED 깜빡이며 앞이 비는지 확인 ---
                // 여기 깜빡 속도는 기존처럼 0.5초 간격
                while (wait_timer < 10) {   // 500ms * 10 = 5초
                    PORTA ^= (1 << LED_PIN);
                    _delay_ms(500);
                    wait_timer++;

                    distance = distanceFLR(TRIGF, ECHOF);
                    if (distance > 350) {
                        path_cleared = 1;
                        break;
                    }
                }

                if (path_cleared) {
                    // ---- [상황 A] 앞이 비었음 → 서서히 재가속 ----
                    PORTA &= ~(1 << LED_PIN);

                    PORTA |=  (1 << PA0);
                    PORTA &= ~(1 << PA1);

                    for (int i = 0; i <= (int)speed; i++) {
                        OCR0 = (uint8_t)i;
                        _delay_ms(14);
                    }
                } else {
                    // ---- [상황 B] 앞이 안 비었음 → 뒤 상황 보고 후진 or stay ----

                    // 먼저 뒤 중앙 거리 확인
                    BACK = distanceBLR(TRIGB, ECHOB);

                    // 뒤 중앙 15cm 이내면 후진하지 말고 STAY
                    if (BACK < 150) {
                        // 제자리 대기, LED 0.5초 간격으로 깜빡이며
                        OCR0 = 0;
                        PORTA &= ~(1 << PA0);
                        PORTA &= ~(1 << PA1);

                        while (1) {
                            PORTA ^= (1 << LED_PIN);
                            _delay_ms(500);

                            distance = distanceFLR(TRIGF, ECHOF);
                            BACK     = distanceBLR(TRIGB, ECHOB);

                            // 앞이 충분히 비거나(>35cm) 뒤가 충분히 비면(>15cm) 탈출
                            if (distance > 350 || BACK > 150) break;
                        }

                        PORTA &= ~(1 << LED_PIN);

                        if (distance > 350) {
                            // 앞이 먼저 비면 → 천천히 전진 시작
                            PORTA |=  (1 << PA0);
                            PORTA &= ~(1 << PA1);
                            for (int i = 0; i <= (int)speed; i++) {
                                OCR0 = (uint8_t)i;
                                _delay_ms(14);
                            }
                        }
                        else if (BACK > 150) {
                            // 뒤가 비고 앞은 여전히 막혀있으면 → 살짝 후진만
                            PORTA |=  (1 << PA1);
                            PORTA &= ~(1 << PA0);
                            OCR0  = backspeed;
                            _delay_ms(400);     // 0.4초 정도만 후진
                            OCR0  = 0;
                        }

                        // 이후는 기본 주행 로직에 맡김
                        continue;
                    }

                    // *** 기존 후진 + 탐색 패턴 (여기서는 뒤가 15cm 이상 확보된 경우) ***

                    // (a) 직진 방향으로 후진, LED 0.5초 간격 깜빡 (총 1초)
                    PORTA |=  (1 << PA1);
                    PORTA &= ~(1 << PA0);
                    OCR0  = 160;
                    OCR1A = SERVO_CENTER;

                    for (uint8_t i = 0; i < 2; i++) { // 500ms * 2 = 1초
                        PORTA ^= (1 << LED_PIN);
                        _delay_ms(500);
                    }

                    // (b) 정지 + 1초 동안 주변 탐색 (LEFT/RIGHT/DIAL/DIAR), LED 0.5초 간격
                    OCR0 = 0;
                    PORTA &= ~(1 << PA1);   // 완전 정지

                    for (uint8_t i = 0; i < 2; i++) { // 500ms * 2 = 1초
                        PORTA ^= (1 << LED_PIN);
                        _delay_ms(500);

                        LEFT  = distanceFLR(TRIGFLL, ECHOFLL);
                        RIGHT = distanceFLR(TRIGFRR, ECHOFRR);
                        DIAL  = distanceDIA(TRIGDIAL, ECHODIAL);
                        DIAR  = distanceDIA(TRIGDIAR, ECHODIAR);
                    }

                    // (c) 왼쪽합 vs 오른쪽합 비교
                    servo_cmd = SERVO_CENTER;

                    unsigned int sum_left  = LEFT + DIAL;
                    unsigned int sum_right = RIGHT + DIAR;

                    if (sum_right > sum_left + 40) {
                        servo_cmd = SERVO_RIGHT;
                    } else if (sum_left > sum_right + 40) {
                        servo_cmd = SERVO_LEFT;
                    }

                    OCR1A = servo_cmd;
                    _delay_ms(200);   // 조향 시간 확보

                    // (d) 선택한 방향으로 0.5초 탈출 전진
                    PORTA &= ~(1 << LED_PIN);
                    PORTA |=  (1 << PA0);
                    PORTA &= ~(1 << PA1);

                    OCR0 = speed + turnspeed;
                    _delay_ms(500);

                    // 특수 루틴 이후에는 일반 주행으로 복귀
                }

                // 긴급 스위치 루틴을 수행했으니 이 루프는 종료
                continue;
            }
        }

        // =====================================================================
        // [6] 기본 자율주행 (완성 코드 로직)
        // =====================================================================
        if( (distance < 360) || (DIAL < 260) || (DIAR < 260) )
{

            // 1) 도망 방향 판단
            if( RIGHT > LEFT + 40 ){
                escape_side = +1;   // 오른쪽으로 도망
            }
            else if( LEFT > RIGHT + 40 ){
                escape_side = -1;   // 왼쪽으로 도망
            }
            else{
                if( DIAR > DIAL + 40 ){
                    escape_side = +1;
                }
                else if( DIAL > DIAR + 40 ){
                    escape_side = -1;
                }
            }

            // 2) 매우 가까운 경우: 반대 조향 후진 + 도망 방향으로 전진
            if( (DIAL < 130) || (DIAR < 130) || (distance < 180) )
              {

                // 후진 (escape_side 반대 방향 조향)
                PORTA |=  (1 << PA1);
                PORTA &= ~(1 << PA0);

                if( escape_side == +1 ){
                    reverse_steer = SERVO_LEFT;
                }
                else if( escape_side == -1 ){
                    reverse_steer = SERVO_RIGHT;
                }
                else{
                    reverse_steer = SERVO_CENTER;
                }

                servo_cmd = reverse_steer;
                OCR1A = servo_cmd;
                OCR0  = backspeed;

                // 후방 장애물 체크
                BACKL = distanceBLR(TRIGBL, ECHOBL);
                _delay_ms(10);
                BACK  = distanceBLR(TRIGB,  ECHOB);
                _delay_ms(10);
                BACKR = distanceBLR(TRIGBR, ECHOBR);
                _delay_ms(10);

                if( (BACKL < 80) || (BACK < 80) || (BACKR < 80) ){
                    OCR0 = 0;
                }else{
                    _delay_ms(800);
                    OCR0 = 0;
                }

                // 전진 (후진 조향의 반대 방향)
                PORTA |=  (1 << PA0);
                PORTA &= ~(1 << PA1);

                if( reverse_steer == SERVO_LEFT ){
                    servo_cmd = SERVO_RIGHT;
                }
                else if( reverse_steer == SERVO_RIGHT ){
                    servo_cmd = SERVO_LEFT;
                }
                else{
                    servo_cmd = SERVO_CENTER;
                }

                OCR1A = servo_cmd;
                OCR0  = speed + turnspeed;
                _delay_ms(200);

                continue;
            }
            // 3) 아직 여유가 있는 경우: 선제 회피
            else{
                if( escape_side == -1 ){
                    servo_cmd = SERVO_LEFT;
                    OCR0 = speed + turnspeed;
                    _delay_ms(20);
                }
                else if( escape_side == +1 ){
                    servo_cmd = SERVO_RIGHT;
                    OCR0 = speed + turnspeed;
                    _delay_ms(20);
                }
                else{
                    if( DIAL > DIAR ){
                        servo_cmd = SERVO_LEFT;
                        OCR0 = speed + turnspeed;
                        _delay_ms(10);
                    }
                    else if( DIAR > DIAL ){
                        servo_cmd = SERVO_RIGHT;
                        OCR0 = speed + turnspeed;
                        _delay_ms(10);
                    }
                    else{
                        servo_cmd = SERVO_CENTER;
                        OCR0 = speed;
                        _delay_ms(20);
                    }
                }
            }
        }
        // 대각만 가까운 경우
        else if( DIAR < 270 ){
            servo_cmd = SERVO_LEFT;
            OCR0 = speed + turnspeed;
            _delay_ms(20);
        }
        else if( DIAL < 270 ){
            servo_cmd = SERVO_RIGHT;
            OCR0 = speed + turnspeed;
            _delay_ms(20);
        }
        else{
            // 아무 장애물 없을 때
            servo_cmd = SERVO_CENTER;
            OCR0 = speed;
            _delay_ms(20);
        }

        // 서보 최종 반영
        OCR1A = servo_cmd;

    } // while(1)
} // main

// ===== 초음파 거리 함수 =====
unsigned distanceFLR(int TRIG, int ECHO){
    TCCR3B = 0x03;                 // Timer/Counter3, prescale 64 → 4us tick
    PORTE &= ~(1 << TRIG);         // Trig = LOW
    _delay_us(10);
    PORTE |=  (1 << TRIG);         // Trig = HIGH
    _delay_us(10);
    PORTE &= ~(1 << TRIG);         // Trig = LOW
    while(!(PINE & (1 << ECHO)));  // Echo HIGH 대기
    TCNT3 = 0x0000;
    while(PINE & (1 << ECHO));     // Echo LOW 대기
    TCCR3B = 0x00;                 // Timer 정지
    return (unsigned int)(SOUND_VELOCITY * (TCNT3 * 4UL / 2UL) / 1000UL);
}

unsigned distanceBLR(int TRIG, int ECHO){
    TCCR3B = 0x03;
    PORTD &= ~(1 << TRIG);
    _delay_us(10);
    PORTD |=  (1 << TRIG);
    _delay_us(10);
    PORTD &= ~(1 << TRIG);
    while(!(PIND & (1 << ECHO)));
    TCNT3 = 0x0000;
    while(PIND & (1 << ECHO));
    TCCR3B = 0x00;
    return (unsigned int)(SOUND_VELOCITY * (TCNT3 * 4UL / 2UL) / 1000UL);
}

unsigned distanceDIA(int TRIG, int ECHO){
    TCCR3B = 0x03;
    PORTF &= ~(1 << TRIG);
    _delay_us(10);
    PORTF |=  (1 << TRIG);
    _delay_us(10);
    PORTF &= ~(1 << TRIG);
    while(!(PINF & (1 << ECHO)));
    TCNT3 = 0x0000;
    while(PINF & (1 << ECHO));
    TCCR3B = 0x00;
    return (unsigned int)(SOUND_VELOCITY * (TCNT3 * 4UL / 2UL) / 1000UL);
}
