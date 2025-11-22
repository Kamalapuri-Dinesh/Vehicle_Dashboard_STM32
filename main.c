/* ev_dashboard_12v_final.c - V32 (16MHz HSI Clock Corrected) */
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
/* =================== CONFIG =================== */
#define PCF8574_ADDR        0x27
#define P_RS  (1<<0)
#define P_RW  (1<<1)
#define P_EN  (1<<2)
#define P_BL  (1<<3)
#define LCD_BACKLIGHT P_BL
#define ADC_VREF_MV         3300UL
#define ADC_MAX             4095UL
/* Voltage & Current Config */
#define V_SCALE_FACTOR      5UL
#define ACS_SENS_MV_PER_A   185
#define ACS_ZERO_VOLTAGE    2500
/* === MOTOR & SENSOR CONFIG === */
#define PULSES_PER_REV      32U      /* 32 Metal Teeth */
#define GEAR_RATIO          1U
#define WHEEL_DIAM_MM       60UL
#define CIRCUMFERENCE_MM    ((31416UL * WHEEL_DIAM_MM) / 10000UL)
/* Battery Config */
#define VOLTAGE_FULL_MV     12600UL
#define VOLTAGE_EMPTY_MV    10500UL
#define VOLTAGE_VERY_EMPTY_MV 10000UL
/* UI & Timing Config */
/* Adjusted for 16MHz Clock reliability */
#define LOOP_UPDATE_MS      25U      /* 40Hz Logic Loop */
#define RPM_CALC_WINDOW_MS  100U     /* 10Hz RPM Updates */
#define DISPLAY_REFRESH_MS  250U     /* 4Hz Screen Refresh */
/* ================ GLOBALS ================ */
volatile uint32_t system_ms = 0;
volatile uint32_t pulse_count = 0;
volatile uint32_t last_valid_pulse_time = 0;
uint32_t odometer_m = 0;
uint32_t odometer_mm_remainder = 0;
// Filtered Values
uint32_t lpf_vin_mv = 0;
int32_t lpf_current_ma = 0;
/* ================ DELAYS (Calibrated for 16MHz) ================ */
// At 16MHz, ~16 cycles per us. C loop overhead reduces this.
// Multiplier lowered significantly for 16MHz.
static void small_delay_us(uint32_t us) { for(uint32_t i=0;i<us*2;i++) __NOP(); }
static void small_delay_ms(uint32_t ms) { for(uint32_t j=0;j<ms;j++) for(uint32_t i=0;i<1000;i++) __NOP(); }
/* ================ TIM2: 1 ms tick (16MHz Source) ================ */
static void TIM2_Init_1ms(void){
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   // Clock = 16MHz.
   // We want 1ms interrupt (1kHz).
   // PSC = 1600 - 1  -> Clock becomes 10 kHz
   // ARR = 10 - 1    -> 10 counts of 10kHz = 1ms
   TIM2->PSC = 1600 - 1;
   TIM2->ARR = 10 - 1;
   TIM2->DIER |= TIM_DIER_UIE;
   TIM2->CR1 |= TIM_CR1_CEN;
   NVIC_EnableIRQ(TIM2_IRQn);
}
void TIM2_IRQHandler(void){
   if(TIM2->SR & TIM_SR_UIF){
       TIM2->SR &= ~TIM_SR_UIF;
       system_ms++;
   }
}
/* ================ BUTTON PC13 INIT ================ */
static void Button_Init(void){
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
   GPIOC->MODER &= ~(3U << (13 * 2));
   GPIOC->PUPDR &= ~(3U << (13 * 2));
}
/* ================ LCD I2C (16MHz APB1) ================ */
static void I2C1_Init(void){
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
   RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
   GPIOB->MODER |= (2<<(8*2)) | (2<<(9*2));
   GPIOB->OTYPER |= (1<<8) | (1<<9);
   GPIOB->PUPDR |= (1<<(8*2)) | (1<<(9*2));
   GPIOB->AFR[1] |= (4<<0) | (4<<4);
   I2C1->CR1 = I2C_CR1_SWRST; I2C1->CR1 = 0;
   // APB1 Freq = 16MHz
   I2C1->CR2 = 16;
   // CCR Calculation for 100kHz:
   // Thigh + Tlow = 10us. Thigh = 5us.
   // PCLK1 = 62.5ns.
   // CCR = 5000ns / 62.5ns = 80.
   I2C1->CCR = 80;
   // TRISE = (1000ns / 62.5ns) + 1 = 16 + 1 = 17
   I2C1->TRISE = 17;
   I2C1->CR1 |= I2C_CR1_PE;
}
static int I2C1_WriteByte(uint8_t addr, uint8_t data){
   uint32_t timeout = 0xFFFF;
   I2C1->CR1 |= I2C_CR1_START;
   while(!(I2C1->SR1 & I2C_SR1_SB) && timeout--) if(!timeout) return -1;
   I2C1->DR = (addr << 1);
   timeout = 0xFFFF;
   while(!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--) if(!timeout) return -2;
   (void)I2C1->SR2;
   timeout = 0xFFFF;
   while(!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) if(!timeout) return -3;
   I2C1->DR = data;
   timeout = 0xFFFF;
   while(!(I2C1->SR1 & I2C_SR1_BTF) && timeout--) if(!timeout) return -4;
   I2C1->CR1 |= I2C_CR1_STOP;
   return 0;
}
static void expanderWrite(uint8_t b){ I2C1_WriteByte(PCF8574_ADDR, b); }
static void lcd_pulse(uint8_t data){ expanderWrite(data|P_EN); small_delay_us(1); expanderWrite(data&~P_EN); small_delay_us(50); }
static void lcd_write4(uint8_t nib, uint8_t rs){ uint8_t o=((nib&0x0F)<<4)|LCD_BACKLIGHT|(rs?P_RS:0); expanderWrite(o&~P_EN); lcd_pulse(o); }
static void lcd_send(uint8_t b, uint8_t rs){ lcd_write4(b>>4, rs); lcd_write4(b&0x0F, rs); }
static void lcd_cmd(uint8_t c){ lcd_send(c,0); if(c==1||c==2) small_delay_ms(2); else small_delay_us(50); }
static void lcd_data(uint8_t d){ lcd_send(d,1); small_delay_us(50); }
static void lcd_init(void){ small_delay_ms(50); expanderWrite(LCD_BACKLIGHT); small_delay_ms(5); lcd_write4(3,0); small_delay_ms(5); lcd_write4(3,0); small_delay_ms(1); lcd_write4(3,0); lcd_write4(2,0); lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_cmd(0x01); }
static void lcd_set_cursor(uint8_t r, uint8_t c){ lcd_cmd((r?0xC0:0x80)+c); }
static void lcd_print(const char *s){ while(*s) lcd_data(*s++); }
static void lcd_clear_row(uint8_t r){ lcd_set_cursor(r,0); lcd_print("                "); }
/* ================ ADC ================ */
static void ADC1_Init_local(void){
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
   GPIOA->MODER |= (3<<(0*2)) | (3<<(1*2));
   // At 16MHz, Div2 (00) gives 8MHz ADC clock (Valid).
   // Div4 (01) gives 4MHz. We'll stick to Div2 for slightly faster conversion.
   ADC->CCR &= ~ADC_CCR_ADCPRE; // Clear bits -> Div2
   ADC1->SMPR2 |= (7<<0) | (7<<3);
   ADC1->CR2 |= ADC_CR2_ADON;
   small_delay_ms(10);
}
static uint16_t adc_read_ch(uint8_t ch){
   ADC1->SQR3 = ch;
   ADC1->CR2 |= ADC_CR2_SWSTART;
   while(!(ADC1->SR & ADC_SR_EOC));
   return (uint16_t)ADC1->DR;
}
static uint32_t adc_raw_to_mv(uint16_t raw){ return (uint32_t)raw * ADC_VREF_MV / ADC_MAX; }
/* ================ RPM EXTI PA6 ================ */
static void RPM_EXTI_Init(void){
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   GPIOA->MODER &= ~(3 << (6*2));
   GPIOA->PUPDR &= ~(3 << (6*2));
   GPIOA->PUPDR |=  (1 << (6*2)); // Pull-down
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
   SYSCFG->EXTICR[1] &= ~(0xF << 8); // PA6
   EXTI->IMR |= (1<<6);
   EXTI->RTSR |= (1<<6); // Rising edge
   NVIC_EnableIRQ(EXTI9_5_IRQn);
}
void EXTI9_5_IRQHandler(void){
   if (EXTI->PR & (1<<6)){
       EXTI->PR = (1<<6);
       // Debounce 1ms is sufficient at 16MHz logic speed
       uint32_t now_isr = system_ms;
       if ((now_isr - last_valid_pulse_time) >= 1) {
           pulse_count++;
           last_valid_pulse_time = now_isr;
       }
   }
}
/* ================ SENSORS ================ */
static uint32_t read_voltage_mv(void) {
   uint32_t sum = 0;
   for(int i=0;i<16;i++) { sum += adc_read_ch(0); small_delay_us(20); }
   return adc_raw_to_mv(sum/16) * V_SCALE_FACTOR;
}
static int32_t read_current_ma(uint32_t acs_zero_mv) {
   uint32_t sum = 0;
   for(int i=0;i<32;i++) { sum += adc_read_ch(1); small_delay_us(20); }
   int32_t mv = (int32_t)adc_raw_to_mv(sum/32) - (int32_t)acs_zero_mv;
   mv = abs(mv);
   if(mv < 15) mv = 0; // Deadband
   return (mv * 1000) / ACS_SENS_MV_PER_A;
}
static uint32_t voltage_to_percent(uint32_t mv) {
   if(mv >= VOLTAGE_FULL_MV) return 100;
   if(mv <= VOLTAGE_EMPTY_MV) return 0;
   return ((mv - VOLTAGE_EMPTY_MV) * 100) / (VOLTAGE_FULL_MV - VOLTAGE_EMPTY_MV);
}
/* ================ MAIN ================ */
int main(void){
   TIM2_Init_1ms();
   I2C1_Init();
   lcd_init();
   ADC1_Init_local();
   RPM_EXTI_Init();
   Button_Init();
   lcd_clear_row(0); lcd_print("CALIBRATING...");
   uint32_t zero_sum = 0;
   for(int i=0;i<100;i++) { zero_sum += adc_raw_to_mv(adc_read_ch(1)); small_delay_ms(2); }
   uint32_t acs_zero = zero_sum / 100;
   lpf_vin_mv = read_voltage_mv();
   uint32_t rpm_timer_ms = 0;
   uint32_t display_timer_ms = 0;
   uint32_t last_loop_ms = system_ms;
   uint32_t last_rpm_pulses = 0;
   uint32_t rpm_display = 0;
   uint32_t speed_display = 0;
   uint8_t page = 0;
   uint8_t button_prev_state = 1;
   uint32_t odo_last_pulses = 0;
   while(1){
       uint32_t now = system_ms;
       uint32_t dt = now - last_loop_ms;
       if (dt < LOOP_UPDATE_MS) { continue; }
       last_loop_ms = now;
       // 1. BUTTON
       uint8_t button_state = (GPIOC->IDR & (1 << 13)) ? 1 : 0;
       if (button_prev_state == 1 && button_state == 0) {
           page = (page + 1) % 4;
           small_delay_ms(200);
           display_timer_ms = DISPLAY_REFRESH_MS;
       }
       button_prev_state = button_state;
       // 2. SENSORS
       uint32_t raw_v = read_voltage_mv();
       lpf_vin_mv = (lpf_vin_mv * 9 + raw_v) / 10;
       int32_t raw_i = read_current_ma(acs_zero);
       lpf_current_ma = (lpf_current_ma * 8 + raw_i * 2) / 10;
       // 3. ODOMETER
       uint32_t curr_pulses = pulse_count;
       uint32_t diff = curr_pulses - odo_last_pulses;
       odo_last_pulses = curr_pulses;
       if(diff > 0 && lpf_vin_mv > 500) {
           uint64_t dist_mm = (uint64_t)diff * CIRCUMFERENCE_MM / PULSES_PER_REV;
           uint64_t total_mm = odometer_mm_remainder + dist_mm;
           odometer_m += total_mm / 1000;
           odometer_mm_remainder = total_mm % 1000;
       }
       // 4. RPM CALCULATION
       rpm_timer_ms += dt;
       if (rpm_timer_ms >= RPM_CALC_WINDOW_MS) {
           uint32_t p_now = pulse_count;
           uint32_t p_delta = p_now - last_rpm_pulses;
           last_rpm_pulses = p_now;
           uint32_t raw_rpm = 0;
           if (lpf_vin_mv > 500) {
                // Use 64-bit math cast to prevent overflow even if counts are high
                raw_rpm = (uint32_t)( ((uint64_t)p_delta * 60000ULL) / (PULSES_PER_REV * rpm_timer_ms) );
                // Relaxed Reality Check for 16MHz timing
                uint32_t max_phys_rpm = (lpf_vin_mv / 25) + 250;
                if (raw_rpm > max_phys_rpm) raw_rpm = 0;
           } else {
                raw_rpm = 0;
           }
           if (raw_rpm > 350) raw_rpm = 350;
           // Lighter filter for responsiveness
           rpm_display = (rpm_display * 6 + raw_rpm * 4) / 10;
           if(rpm_display < 3) rpm_display = 0;
           uint32_t wheel_rpm = rpm_display / GEAR_RATIO;
           speed_display = (wheel_rpm * CIRCUMFERENCE_MM * 60) / 1000000UL;
           rpm_timer_ms = 0;
       }
       // 5. DISPLAY
       display_timer_ms += dt;
       if (display_timer_ms >= DISPLAY_REFRESH_MS) {
           display_timer_ms = 0;
           lcd_clear_row(0); lcd_set_cursor(0,0);
           lcd_clear_row(1); lcd_set_cursor(1,0);
           char L1[17], L2[17];
           if (page == 0) {
               uint32_t v_int = lpf_vin_mv/1000;
               uint32_t v_fr = (lpf_vin_mv%1000)/10;
               snprintf(L1, 16, "V:%lu.%02luV ", v_int, v_fr);
               snprintf(L2, 16, "Odo:%lum", odometer_m);
           } else if (page == 1) {
               snprintf(L1, 16, "RPM: %lu", rpm_display);
               snprintf(L2, 16, "Spd: %lu km/h", speed_display);
           } else if (page == 2) {
               int32_t a_abs = abs(lpf_current_ma);
               snprintf(L1, 16, "Curr: %lu.%02luA", a_abs/1000, (a_abs%1000)/10);
               snprintf(L2, 16, "----------------");
           } else {
                snprintf(L1, 16, "Dist: %lu km", odometer_m/1000);
                snprintf(L2, 16, "      %lu m", odometer_m%1000);
           }
           lcd_set_cursor(0,0); lcd_print(L1);
           lcd_set_cursor(1,0); lcd_print(L2);
       }
   }
   return 0;
}
