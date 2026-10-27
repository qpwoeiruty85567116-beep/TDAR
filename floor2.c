#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

/* ================= 핀 매핑 =================
   LCD(ST7735S): PB1=SCK, PB2=MOSI, PB4=CS, PB5=DC, PB6=RST, PB0=SS(마스터 유지)
   TCS34725:     PD0=SCL, PD1=SDA
   CdS 조도센서: PF2(ADC2) 분압 입력
   LED:          PA2
*/
#define LCD_CS    PB4
#define LCD_RST   PB6
#define LCD_DC    PB5
#define SPI_MOSI  PB2
#define SPI_SCK   PB1
#define SPI_SS    PB0

/* ===== 조도센서 + LED ===== */
#define LED_DDR   DDRA
#define LED_PORT  PORTA
#define LED_PIN   PA2

#define CDS_ADC_CH         2      // PF2 = ADC2
#define ADC_THRESHOLD_0_9V 184    // 0.9V 기준 ADC 값

/* ===== L298N 모터 드라이버 =====
   ENA: PB7(OC2 PWM)  ← MCU에서만 연결
   ENB: ENA와 점퍼선으로 묶어서 동일 PWM 사용 (보드 간 점퍼)
   모터 A(OUT1,OUT2): IN1, IN2
   모터 B(OUT3,OUT4): IN3, IN4 (A와 반대 방향으로 회전)
*/
#define ENA_DDR   DDRB
#define ENA_PORT  PORTB
#define ENA_PIN   PB7

#define IN_DDR    DDRD
#define IN_PORT   PORTD
#define IN1_PIN   PD2   // 모터 A
#define IN2_PIN   PD3
#define IN3_PIN   PD4   // 모터 B (반대방향용)
#define IN4_PIN   PD5

// Vs?6V 기준 평균 5V 근처를 노린 duty 값 (필요하면 튜닝)
#define MOTOR_PWM_DUTY_4V 170

/* ===== 적외선 센서 (TCRT5000 모듈 AO) =====
   적외선 센서 1: AO → PF1 (ADC1)
   적외선 센서 2: AO → PF3 (ADC3)
*/
#define IR_ADC_CH1       1   // PF1 = ADC1
#define IR_ADC_CH2       3   // PF3 = ADC3
#define IR_THRESHOLD     160 // 이 값 이상이면 "검정"으로 판정 (환경에 맞게 튜닝)

/* ================= SPI(LCD) ================= */
static inline void spi_init(void){
    DDRB |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<LCD_CS)|(1<<LCD_DC)|(1<<LCD_RST)|(1<<SPI_SS);
    PORTB |= (1<<LCD_CS)|(1<<SPI_SS);
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // Mode0, SCK=F_CPU/16
}
static inline void spi_send(uint8_t v){ SPDR=v; while(!(SPSR&(1<<SPIF))); }

static inline void cs_low(void){  PORTB &= ~(1<<LCD_CS); }
static inline void cs_high(void){ PORTB |=  (1<<LCD_CS); }
static inline void dc_cmd(void){  PORTB &= ~(1<<LCD_DC); }
static inline void dc_data(void){ PORTB |=  (1<<LCD_DC); }

static void lcd_cmd(uint8_t c){ cs_low(); dc_cmd(); spi_send(c); cs_high(); }
static void lcd_data(uint8_t d){ cs_low(); dc_data(); spi_send(d); cs_high(); }
static void lcd_data_buf(const uint8_t* p, uint32_t n){
    cs_low(); dc_data(); while(n--) spi_send(*p++); cs_high();
}

/* ================= LCD(ST7735S) ================= */
static void lcd_hw_reset(void){
    PORTB &= ~(1<<LCD_RST); _delay_ms(50);
    PORTB |=  (1<<LCD_RST); _delay_ms(120);
}
static void lcd_set_addr(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1){
    lcd_cmd(0x2A); uint8_t col[] = {0x00,x0, 0x00,x1}; lcd_data_buf(col,4);
    lcd_cmd(0x2B); uint8_t row[] = {0x00,y0, 0x00,y1}; lcd_data_buf(row,4);
    lcd_cmd(0x2C);
}
static inline uint16_t BGR565(uint8_t r,uint8_t g,uint8_t b){
    return ((b & 0xF8)<<8) | ((g & 0xFC)<<3) | (r>>3);
}
static void lcd_fill(uint16_t color){
    lcd_set_addr(0,0,127,159);
    cs_low(); dc_data();
    for(uint32_t i=0;i<128UL*160UL;i++){ spi_send(color>>8); spi_send(color&0xFF); }
    cs_high();
}
static void lcd_init(void){
    lcd_hw_reset();
    lcd_cmd(0x01); _delay_ms(150);
    lcd_cmd(0x11); _delay_ms(120);
    lcd_cmd(0x3A); lcd_data(0x05);
    lcd_cmd(0x36); lcd_data(0xC8);
    lcd_cmd(0x29); _delay_ms(20);
    lcd_fill(BGR565(0,0,0));
}

/* ================== 5x7 폰트 ================== */
typedef struct { char ch; uint8_t col[5]; } Glyph5x7;
static const Glyph5x7 GLYPHS[] = {
  // 기존 글자들
  {'R', {0x7F,0x09,0x19,0x29,0x46}},
  {'E', {0x7F,0x49,0x49,0x49,0x41}},
  {'D', {0x7F,0x41,0x41,0x22,0x1C}},
  {'G', {0x3E,0x41,0x49,0x49,0x3A}},
  {'N', {0x7F,0x04,0x08,0x10,0x7F}},
  {'B', {0x7F,0x49,0x49,0x49,0x36}},
  {'L', {0x7F,0x40,0x40,0x40,0x40}},
  {'U', {0x3F,0x40,0x40,0x40,0x3F}},

  // WHITE, BLACK용 추가
  {'A', {0x7E,0x09,0x09,0x09,0x7E}},   // A
  {'C', {0x3E,0x41,0x41,0x41,0x22}},   // C
  {'H', {0x7F,0x08,0x08,0x08,0x7F}},   // H
  {'I', {0x41,0x41,0x7F,0x41,0x41}},   // I
  {'K', {0x7F,0x08,0x14,0x22,0x41}},   // K
  {'T', {0x01,0x01,0x7F,0x01,0x01}},   // T
  {'W', {0x7F,0x20,0x18,0x20,0x7F}},   // W

  // 공백
  {' ', {0x00,0x00,0x00,0x00,0x00}},
};

static const uint8_t* glyph_lookup(char c){
    if(c>='a' && c<='z') c = (char)(c - 'a' + 'A');
    for(uint8_t i=0;i<sizeof(GLYPHS)/sizeof(GLYPHS[0]); i++)
        if(GLYPHS[i].ch == c) return GLYPHS[i].col;
    return GLYPHS[sizeof(GLYPHS)/sizeof(GLYPHS[0]) - 1].col; // space
}

static void lcd_draw_char(uint8_t x, uint8_t y, char ch, uint16_t fg, uint8_t scale){
    const uint8_t* glyph = glyph_lookup(ch);
    for(uint8_t col=0; col<5; col++){
        uint8_t line = glyph[col];
        for(uint8_t row=0; row<7; row++){
            if(line & (1<<row)){
                for(uint8_t dx=0; dx<scale; dx++){
                    for(uint8_t dy=0; dy<scale; dy++){
                        uint8_t px = x + col*scale + dx;
                        uint8_t py = y + row*scale + dy;
                        if(px>127 || py>159) continue;
                        lcd_set_addr(px, py, px, py);
                        cs_low(); dc_data();
                        spi_send(fg>>8); spi_send(fg&0xFF);
                        cs_high();
                    }
                }
            }
        }
    }
}

static void lcd_print(uint8_t x, uint8_t y, const char* s, uint16_t fg, uint8_t scale){
    while(*s){
        lcd_draw_char(x, y, *s, fg, scale);
        x += (6*scale);
        s++;
        if(x >= 128-6*scale) { x = 0; y += 8*scale; }
        if(y >= 160-8*scale) break;
    }
}

/* ================= I2C(TWI) ================= */
static void twi_init(void){
    TWSR=0x00;
    TWBR=((F_CPU/100000UL)-16)/2;  // 100kHz
    PORTD|=(1<<PD0)|(1<<PD1);      // 풀업
}
static void twi_start(void){
    TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while(!(TWCR&(1<<TWINT)));
}
static void twi_stop(void){
    TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}
static void twi_write(uint8_t d){
    TWDR=d;
    TWCR=(1<<TWINT)|(1<<TWEN);
    while(!(TWCR&(1<<TWINT)));
}
static uint8_t twi_read_ack(void){
    TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while(!(TWCR&(1<<TWINT)));
    return TWDR;
}
static uint8_t twi_read_nack(void){
    TWCR=(1<<TWINT)|(1<<TWEN);
    while(!(TWCR&(1<<TWINT)));
    return TWDR;
}

/* ================= TCS34725 ================= */
#define TCS_ADDR 0x29
#define TCS_CMD  0x80
#define TCS_ENABLE  0x00
#define TCS_ATIME   0x01
#define TCS_CONTROL 0x0F
#define TCS_CDATAL  0x14
#define TCS_RDATAL  0x16
#define TCS_GDATAL  0x18
#define TCS_BDATAL  0x1A
#define TCS_PON 0x01
#define TCS_AEN 0x02

static void tcs_write8(uint8_t reg,uint8_t val){
    twi_start(); twi_write((TCS_ADDR<<1)|0); twi_write(TCS_CMD|reg); twi_write(val); twi_stop();
}
static uint16_t tcs_read16(uint8_t reg){
    uint8_t b[2];
    twi_start(); twi_write((TCS_ADDR<<1)|0); twi_write(TCS_CMD|reg);
    twi_start(); twi_write((TCS_ADDR<<1)|1);
    b[0]=twi_read_ack(); b[1]=twi_read_nack(); twi_stop();
    return (uint16_t)b[0]|((uint16_t)b[1]<<8);
}
static void tcs_init(void){
    tcs_write8(TCS_ENABLE,TCS_PON); _delay_ms(3);
    tcs_write8(TCS_ENABLE,TCS_PON|TCS_AEN);
    tcs_write8(TCS_ATIME,0xC0); tcs_write8(TCS_CONTROL,0x00);
    _delay_ms(160);
}
static void tcs_read_rgbc(uint16_t* r,uint16_t* g,uint16_t* b,uint16_t* c){
    _delay_ms(160);
    *c=tcs_read16(TCS_CDATAL);
    *r=tcs_read16(TCS_RDATAL);
    *g=tcs_read16(TCS_GDATAL);
    *b=tcs_read16(TCS_BDATAL);
}

/* ================= 색상 보정 ================= */
#define R_GAIN 1.30f
#define G_GAIN 1.00f
#define B_GAIN 1.10f
static void rgb_norm_gain(uint16_t r16,uint16_t g16,uint16_t b16,uint16_t c16,
                          uint8_t* R,uint8_t* G,uint8_t* B){
    if(!c16){*R=*G=*B=0;return;}
    float r=(float)r16*255.0f/(float)c16;
    float g=(float)g16*255.0f/(float)c16;
    float b=(float)b16*255.0f/(float)c16;
    r*=R_GAIN; g*=G_GAIN; b*=B_GAIN;
    if(r>255)r=255;if(g>255)g=255;if(b>255)b=255;
    *R=(uint8_t)r; *G=(uint8_t)g; *B=(uint8_t)b;
}


/* ================= ADC (공용 + ADC0/2 래퍼) ================= */
static void ADC_init(void){
    ADMUX  = (1<<REFS0);                       // Vref=AVCC(5V)
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);  // enable, clk/64
}
static uint16_t ADC_read(uint8_t ch){          // 공용 함수
    ADMUX = (ADMUX & 0xF0) | (ch & 0x07);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADC;
}
static uint16_t ADC_read0(void){               // PF0 = ADC0
    return ADC_read(0);
}
static uint16_t ADC_read2(void){               // PF2 = ADC2 (CdS)
    return ADC_read(2);
}

/* ================= Servo (Timer3, OC3A=PE3 / OC3B=PE4) ================= */
static int16_t trim_us_A = 0;
static int16_t trim_us_B = 0;

static inline uint16_t clamp_us(int32_t us){
    if(us < 600)  us = 600;
    if(us > 2400) us = 2400;
    return (uint16_t)us;
}
static inline uint16_t mirror_us(uint16_t center, uint16_t target){
    int32_t m = (int32_t)center + ((int32_t)center - (int32_t)target);
    return clamp_us(m);
}
static inline void servo_init_PE3_PE4(void){
    DDRE  |= (1<<PE3) | (1<<PE4);                 // OC3A/OC3B 출력
    TCCR3A = (1<<WGM31) | (1<<COM3A1) | (1<<COM3B1); // Fast PWM, 비반전 A/B
    TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS31);    // TOP=ICR3, 분주=8
    ICR3   = 40000;                                 // 20ms (0.5us * 40000)
}
static inline void set_pulse_A_us(uint16_t us){
    int32_t v = (int32_t)us + trim_us_A;
    OCR3A = clamp_us(v) * 2;                        // 0.5us/tick
}
static inline void set_pulse_B_us(uint16_t us){
    int32_t v = (int32_t)us + trim_us_B;
    OCR3B = clamp_us(v) * 2;                        // 0.5us/tick
}

/* ================= L298N 모터 제어 (Timer2 PWM, PB7) ================= */
static inline void motor_init(void){
    ENA_DDR  |= (1<<ENA_PIN);                           // ENA(PB7) 출력
    IN_DDR   |= (1<<IN1_PIN)|(1<<IN2_PIN)
              | (1<<IN3_PIN)|(1<<IN4_PIN);              // IN1~IN4 출력

    // Timer2: Fast PWM, non-inverting, prescaler = 8
    TCCR2 = (1<<WGM21)|(1<<WGM20)|(1<<COM21)|(1<<CS21);

    OCR2  = 0;                                          // 초기 정지
    IN_PORT &= ~((1<<IN1_PIN)|(1<<IN2_PIN)
               | (1<<IN3_PIN)|(1<<IN4_PIN));
}

// 두 모터를 서로 반대 방향으로 회전
static inline void motor_forward_pwm(uint8_t duty){
    // 모터 A: 정방향
    IN_PORT |=  (1<<IN1_PIN);
    IN_PORT &= ~(1<<IN2_PIN);

    // 모터 B: 역방향
    IN_PORT &= ~(1<<IN3_PIN);
    IN_PORT |=  (1<<IN4_PIN);

    OCR2 = duty;
}

static inline void motor_brake(void){
    OCR2 = 0;
    IN_PORT &= ~((1<<IN1_PIN)|(1<<IN2_PIN)
               | (1<<IN3_PIN)|(1<<IN4_PIN));
}

/* ===== 컬러 상태 플래그 (지금은 사용 안 함, 그대로 둠) ===== */
static bool green_seen = false;
static bool white_mode = false;

/* ===== 적외선 출발/종료 상태 =====
   ir_phase:
     0 : 아직 검정선 한 번도 안 봄
     1 : 첫 번째 검정선 지난 이후 (RUN 구간)
     2 : 두 번째 검정선 지난 이후 (완전 STOP 구간)
*/
static uint8_t ir_phase = 0;
static bool    ir_prev_black = false;

/* ===== 색 ID (같은 색이 계속 나올 땐 LCD를 다시 안 그리기 위해) ===== */
enum {
    COLOR_ID_RED = 0,
    COLOR_ID_GREEN,
    COLOR_ID_BLUE,
    COLOR_ID_WHITE,
    COLOR_ID_BLACK
};
static uint8_t current_color_id = 0xFF;

/* ================= 메인 ================= */
#define COLOR_RED   BGR565(255,0,0)
#define COLOR_GREEN BGR565(0,255,0)
#define COLOR_BLUE  BGR565(0,0,255)
#define COLOR_WHITE BGR565(255,255,255)
#define COLOR_BLACK BGR565(0,0,0)

int main(void){
    // 주변기기 초기화
    spi_init();
    lcd_init();
    twi_init();
    tcs_init();
    ADC_init();
    servo_init_PE3_PE4();
    motor_init();
    // UART0_init(9600); // 필요시

    // LED 핀 출력 설정
    LED_DDR  |= (1<<LED_PIN);
    LED_PORT &= ~(1<<LED_PIN);    // 초기 OFF

    // 서보 각도(펄스폭)
    const uint16_t CENTER_US = 1500;   // 원위치
    const uint16_t A_120_US  = 1200;   // PE3(OC3A)
    const uint16_t B_120_US  = mirror_us(CENTER_US, A_120_US); // PE4(OC3B) 반대각

    // 초기 위치: 둘 다 CENTER
    set_pulse_A_us(CENTER_US);
    set_pulse_B_us(CENTER_US);

    // PF0용 히스테리시스 예시 값
    const uint16_t TH_LOW  = 388;
    const uint16_t TH_HIGH = 430;

    bool state_high = false;
    bool prev_state = state_high;

    while(1){
        /* ===== 1) ADC0(PF0) 읽어서 PF0 HIGH/LOW 상태 업데이트 ===== */
        uint32_t s=0;
        for(int i=0;i<8;i++){
            s += ADC_read0();
            _delay_ms(1);
        }
        uint16_t x = s/8;

        // 히스테리시스
        if(state_high){
            if(x < TH_LOW)  state_high = false;
        }else{
            if(x > TH_HIGH) state_high = true;
        }

        if(state_high != prev_state){
            // prints(state_high ? "PF0 HIGH\r\n" : "PF0 LOW\r\n");
            prev_state = state_high;
        }

        /* ===== 1.2) 적외선(AO: PF1, PF3) 읽어서 출발/종료 1회성 상태 갱신 =====
           - 두 센서 중 하나라도 IR_THRESHOLD 이상이면 "검정"으로 봄
           - black의 rising edge마다 상태 전이:
             1번째 edge: ir_phase 0 -> 1 (RUN 구간 진입)
             2번째 edge: ir_phase 1 -> 2 (STOP 구간 진입)
             이후(phase=2)는 더 이상 변화 없음
        */
        uint32_t sum_ir1 = 0;
        uint32_t sum_ir3 = 0;
        for(uint8_t i=0;i<8;i++){
            sum_ir1 += ADC_read(IR_ADC_CH1);  // PF1 = ADC1
            sum_ir3 += ADC_read(IR_ADC_CH2);  // PF3 = ADC3
            _delay_ms(1);
        }
        uint16_t ir1 = sum_ir1 / 8;
        uint16_t ir3 = sum_ir3 / 8;

        bool is_black = (ir1 >= IR_THRESHOLD) || (ir3 >= IR_THRESHOLD);

        if(is_black && !ir_prev_black){
            if(ir_phase == 0){
                ir_phase = 1;  // 첫 번째 검정선 통과 → RUN
            }else if(ir_phase == 1){
                ir_phase = 2;  // 두 번째 검정선 통과 → STOP
            }
            // ir_phase가 2 이후에는 더 이상 바꾸지 않음
        }
        ir_prev_black = is_black;

        /* ===== 1.3) 최종 동작 결정 (PF0 우선 + 적외선 1회성) =====
           우선순위:
             1) PF0 HIGH면 무조건 ON
             2) PF0 LOW이면 ir_phase == 1 일 때만 ON (출발선~종료선 사이)
        */
        bool final_run = false;

        if(state_high){
            final_run = true;  // PF0 HIGH → 최우선 ON
        }else{
            if(ir_phase == 1){
                final_run = true;  // 출발선과 종료선 사이 구간
            }else{
                final_run = false; // 시작 전 또는 종료 후
            }
        }

        // 서보 + 모터에 최종 반영
        if(final_run){
            set_pulse_A_us(A_120_US);
            set_pulse_B_us(B_120_US);
            motor_forward_pwm(MOTOR_PWM_DUTY_4V);
        }else{
            set_pulse_A_us(CENTER_US);
            set_pulse_B_us(CENTER_US);
            motor_brake();
        }

        /* ===== 1.5) ADC2(CdS) 읽어서 LED 제어 ===== */
        {
            uint32_t sum = 0;
            for(uint8_t i=0;i<8;i++){
                sum += ADC_read2();
                _delay_ms(1);
            }
            uint16_t val = sum / 8;

            // 0.9V 이상이면 LED ON
            if(val >= ADC_THRESHOLD_0_9V)
                LED_PORT |=  (1<<LED_PIN);
            else
                LED_PORT &= ~(1<<LED_PIN);
        }

        /* ===== 2) 컬러센서 읽어서 LCD에 색/글자 출력 ===== */
        uint16_t r16,g16,b16,c16;
        tcs_read_rgbc(&r16,&g16,&b16,&c16);

        // ---- 여기부터: C 구간 + R/G/B 우세 조건으로 색 결정 ----
        // C 값 기준 (테스트 결과):
        //   Red   : C ?  780  근처
        //   Green : C ? 1446  근처
        //   Blue  : C ? 1970  근처
        //   White : C ? 2060  근처
        //
        // C 구간을 중간값 기준으로 나누면:
        //   Red   : C < 1100
        //   Green : 1100 ≤ C < 1700
        //   Blue  : 1700 ≤ C < 2020
        //   White : C ≥ 2020

        const uint16_t C_RED_MAX    = 1100; // Red : C < 1100
        const uint16_t C_GREEN_MIN  = 1100; // Green : 1100 ≤ C < 1700
        const uint16_t C_GREEN_MAX  = 1700;
        const uint16_t C_BLUE_MIN   = 1700; // Blue : 1700 ≤ C < 2020
        const uint16_t C_BLUE_MAX   = 2020;
        const uint16_t C_WHITE_MIN  = 2020; // White : C ≥ 2020

        const char* name;
        uint16_t    bg;
        uint8_t     new_color_id;

        // RED: C < 1100 이면서, R이 G,B보다 큰 경우
        if (c16 < C_RED_MAX &&
            (r16 > g16) && (r16 > b16)) {

            name         = "RED";
            bg           = COLOR_RED;
            new_color_id = COLOR_ID_RED;
        }
        // GREEN: 1100 ≤ C < 1700 이면서, G가 R,B보다 큰 경우
        else if (c16 >= C_GREEN_MIN && c16 < C_GREEN_MAX &&
                 (g16 > r16) && (g16 > b16)) {

            name         = "GREEN";
            bg           = COLOR_GREEN;
            new_color_id = COLOR_ID_GREEN;
        }
        // BLUE: 1700 ≤ C < 2020 이면서, B가 R,G보다 큰 경우
        else if (c16 >= C_BLUE_MIN && c16 < C_BLUE_MAX &&
                 (b16 > r16) && (b16 > g16)) {

            name         = "BLUE";
            bg           = COLOR_BLUE;
            new_color_id = COLOR_ID_BLUE;
        }
        // WHITE: C ≥ 2020
        else if (c16 >= C_WHITE_MIN) {

            name         = "WHITE";
            bg           = COLOR_WHITE;
            new_color_id = COLOR_ID_WHITE;
        }
        // 나머지 애매한 값들은 전부 WHITE로 처리
        else {
            name         = "WHITE";
            bg           = COLOR_WHITE;
            new_color_id = COLOR_ID_WHITE;
        }
        // ---- 여기까지 색 판정 로직 ----

        // 색이 바뀐 경우에만 LCD 전체 업데이트 → 깜빡임/흰줄 방지
        if (new_color_id != current_color_id) {
            current_color_id = new_color_id;

            // 텍스트 색: 배경이 검정이면 흰색, 나머지는 검정
            uint16_t text_color = BGR565(0,0,0);
            if (bg == COLOR_BLACK) {
                text_color = COLOR_WHITE;
            }

            // 글자 가운데 정렬
            uint8_t scale = 3;
            uint8_t len = 0;
            const char* p = name;
            while (*p++) len++;
            uint8_t text_w = len * 6 * scale;    // 5px + 1px 간격
            uint8_t x0 = (uint8_t)((128 - text_w) / 2);
            uint8_t y0 = 70;

            lcd_fill(bg);
            lcd_print(x0, y0, name, text_color, scale);
        }

        _delay_ms(150);
    }
}
