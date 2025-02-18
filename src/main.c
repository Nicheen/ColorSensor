/*

Made by Gustav Benkowski and Noel Persson

*/
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <./res/lcd.h>
#include <./res/i2cmaster.h>



/*        B-PINS (8/8 USED)       
            PB0 - PB7           */
#define LED1_RED     PB1  // Pin 15
#define LED1_GREEN   PB2  // Pin 16
#define LED1_BLUE    PB3  // Pin 17
// PB4 - PB7 are used for LCD

/*       C-PINS (1/7 USED)        
            PC0 - PC6           */
#define POT         PC3  // Pin 26

/*       D-PINS (5/7 USED)        
            PD0 - PD7           */
#define SENSOR_LED  PD0  // Pin 2
#define BTN         PD2  // Pin 4
#define LED2_RED    PD3  // Pin 5
#define LED2_GREEN  PD5  // Pin 11
#define LED2_BLUE   PD6  // Pin 12

#define DEBOUNCE_TIME 50
#define FREQ_TIMEOUT 1000
#define SMOOTHING_FACTOR 2

/* Function pointer primitive */ 
typedef void (*state_func_t)(void);

typedef struct {
    uint8_t id;
    state_func_t Enter;
    state_func_t Do;
    state_func_t Exit;
    uint16_t delay_ms1;
} state_t;

typedef enum {
    b1_evt = 0,
    no_evt = 1
} event_t;

volatile event_t evt = no_evt;

volatile uint16_t timer_count = 0;
volatile uint8_t last_interrupt_time = 0;

/* Button Interrupts */

// Timer0 overflow for system timing
ISR(TIMER0_OVF_vect) {
    timer_count++;
    // Reset counter after 1000ms to prevent overflow
    if (timer_count >= 1000) {
        timer_count = 0;
    }
}

ISR(INT0_vect) {
    uint8_t current_time = timer_count & 0xFF; // Take lower 8 bits for comparison
    uint8_t time_diff;
    
    // Handle timer wraparound
    if (current_time < last_interrupt_time) {
        time_diff = (255 - last_interrupt_time) + current_time;
    } else {
        time_diff = current_time - last_interrupt_time;
    }
    
    if (time_diff > DEBOUNCE_TIME) {
        evt = b1_evt;
        last_interrupt_time = current_time;
    }
}

void init_timer0() {
    // Configure Timer0 for 1ms intervals
    // Enable PWM on OC0A (PD6) and OC0B (PD5)
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64
    TIMSK0 = (1 << TOIE0);  // Enable overflow interrupt
}

void delay_ms(uint16_t ms) {
    uint16_t start = timer_count;
    while ((timer_count - start) < ms) {
        // Handle timer wraparound
        if (timer_count < start) {
            if ((1000 - start + timer_count) >= ms) break;
        }
    }
}

void set_rgb_led1(uint8_t r, uint8_t g, uint8_t b) {
    OCR1A = r;  // LED1_RED (PB1)
    OCR1B = g;  // LED1_GREEN (PB2)
    OCR2A = b;  // LED1_BLUE (PB3)
}

/* LED2 is driven by Timer2 and Timer0: 
   - Red on PD3 (OC2B), Green on PD5 (OC0B), Blue on PD6 (OC0A) */
void set_rgb_led2(uint8_t r, uint8_t g, uint8_t b) {
    OCR2B = r;  // LED2_RED (PD3)
    OCR0B = g;  // LED2_GREEN (PD5)
    OCR0A = b;  // LED2_BLUE (PD6)
}

/* For simplicity, we can create a function that updates both LEDs to the same color */
void set_rgb_leds(uint8_t r, uint8_t g, uint8_t b) {
    set_rgb_led1(r, g, b);
    set_rgb_led2(r, g, b);
}

void init_adc() {
    // Set POT (PC3) as input (it's already input by default)
    DDRC &= ~(1 << POT);

    ADMUX |= (1 << REFS0) | (1 << MUX1) | (1 << MUX0);
    
    ADCSRA |= ((1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1));
}

uint16_t read_adc() {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/* GPIO & Interrupt Setup */
void init() {
    init_timer0();

    // Set LED1 pins (PB1, PB2, PB3) as outputs
    DDRB |= (1 << LED1_RED) | (1 << LED1_GREEN) | (1 << LED1_BLUE);
    // Set SENSOR_LED and LED2 pins (PD0, PD3, PD5, PD6) as outputs
    DDRD |= (1 << SENSOR_LED) | (1 << LED2_RED) | (1 << LED2_GREEN) | (1 << LED2_BLUE);

    // Configure Timer1 for Fast PWM on OC1A and OC1B (LED1)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler = 8
    ICR1 = 0xFF; // 8-bit resolution

    // Configure Timer2 for Fast PWM on OC2A (LED1_BLUE) and OC2B (LED2_RED)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1) | (1 << COM2B1); // Fast PWM, clear on compare match
    TCCR2B = (1 << CS21); // No prescaling

    // Configure button interrupt
    EICRA = (1 << ISC01) | (1 << ISC00);  // Falling edge of INT0 generates interrupt request
    EIMSK = (1 << INT0);  // Enable INT0

    DDRD &= ~(1 << BTN);  // Set BTN as input
    PORTD |= (1 << BTN);  // Enable pull-up resistor

    lcd_init(LCD_DISP_ON);
    lcd_clrscr();

    i2c_init();
    _delay_ms(10);
    sei();  // Enable global interrupts
    init_adc();
}

uint8_t check_sensor() {
    i2c_start(0x52);
    if (i2c_write(0x80) == 0) {
        i2c_stop();
        return 1;
    }
    i2c_stop();
    return 0;
}

void enable_TCS34725() {
    i2c_start(0x52);
    i2c_write(0x80 | 0x00);
    i2c_write(0x03);
    i2c_stop();
    _delay_ms(10);

    // Set integration time to ~154 ms (0xC0)
    i2c_start(0x52);
    i2c_write(0x80 | 0x01); // ATIME register
    i2c_write(0xC0);        // 0xC0 for ~154 ms integration time
    i2c_stop();
    _delay_ms(10);
}

uint16_t read_register(uint8_t reg) {
    i2c_start(0x52);
    i2c_write(0x80 | reg);
    i2c_rep_start(0x53);
    uint16_t low = i2c_readAck();
    uint16_t high = i2c_readNak();
    i2c_stop();
    return (high << 8) | low;
}

/* State Machine Functions */
void do_nothing(void) { return; }

void reset() {
    set_rgb_leds(0, 0, 0);
    lcd_clrscr();
}

void enter_state_0(void) {
    reset();
    lcd_gotoxy(0, 0);
    lcd_puts("Ready to scan?");
    lcd_gotoxy(0, 1);
    lcd_puts("Press btn (S1)");
    PORTD &= ~(1 << SENSOR_LED);
}

void do_state_0(void) {
    set_rgb_leds(255, 0, 0); 
}

void enter_state_1(void) {
    reset();
    enable_TCS34725();
    // For a visual cue, set a fixed color on LED1 (or both)
    set_rgb_leds(255, 0, 0);
    PORTD |= (1 << SENSOR_LED);
}

void do_state_1(void) {
    // Read sensor values
    uint16_t c = read_register(0x14);
    uint16_t r = read_register(0x16);
    uint16_t g = read_register(0x18);
    uint16_t b = read_register(0x1A);

    // Calculate 8-bit color values; prevent division by zero if c is 0
    if (c == 0) c = 1;
    uint8_t red   = (uint8_t)((float)(r * 255) / (float)c);
    uint8_t green = (uint8_t)((float)(g * 255) / (float)c);
    uint8_t blue  = (uint8_t)((float)(b * 255) / (float)c);

    char buffer_1[16];
    char buffer_2[16];

    lcd_gotoxy(0, 0);
    sprintf(buffer_1, "%3d %3d %3d", red, green, blue);
    lcd_puts(buffer_1);

    uint16_t pot_value = read_adc() >> 2;  // 8-bit value from ADC
    lcd_gotoxy(0, 1);
    sprintf(buffer_2, "P:%3d L:%6d", pot_value, c);
    lcd_puts(buffer_2);

    float pot_value_f = (float)pot_value / 255.0;
    uint8_t out_r = red   * pot_value_f;
    uint8_t out_g = green * pot_value_f;
    uint8_t out_b = blue  * pot_value_f;

    // Update both LED1 and LED2 with the computed color
    set_rgb_leds(out_r, out_g, out_b);
}

const state_t state0 = {0, enter_state_0, do_state_0, do_nothing, 10};
const state_t state1 = {1, enter_state_1, do_state_1, do_nothing, 10};

const state_t* state_table[2][2] = {
    { &state1, &state0 }, // From state0: b1_evt -> state1, no_evt -> state0
    { &state0, &state1 }  // From state1: b1_evt -> state0, no_evt -> state1
};

int main() {
    init();
    const state_t* current_state = state_table[0][no_evt];
    evt = no_evt;
    current_state->Enter();
    while (1) {
        current_state->Do();
        delay_ms(current_state->delay_ms1);
        const state_t* next_state = state_table[current_state->id][evt];
        if (current_state->id != next_state->id) {
            current_state->Exit();
            current_state = next_state;
            current_state->Enter();
            evt = no_evt;
        }
    }
}