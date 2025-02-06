#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <lcd.h>
#include <util/atomic.h>
#include <i2cmaster.h>


/*            PD0 - PD7           */
#define SENSOR_LED  PD0  // Pin 2
#define BTN         PD3  // Pin 5

/*            PB0 - PB7           */
#define LED_GREEN   PB1  // Pin 15
#define LED_RED     PB2  // Pin 16
#define LED_BLUE    PB3  // Pin 17
/*   PB4 - PB7 are used for LCD   */

#define DEBOUNCE_TIME 50
#define FREQ_TIMEOUT 1000
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
    TCCR0A = (1 << WGM01) | (1 << WGM00);
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


void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    // Handle Red LED (PB2, OC1B)
    OCR1B = r; // Set duty cycle
    OCR1A = g; // Set duty cycle
    OCR2A = b; // Set duty cycle
}

/* GPIO & Interrupt Setup */
void init() {
    init_timer0();

    // Set LEDs as outputs
    DDRB |= (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);
    DDRD |= (1 << SENSOR_LED);

    // Configure Timer1 for Fast PWM on OC1A (PB1) and OC1B (PB2)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << CS11);                       // Prescaler = 8
    
    ICR1 = 0xFF; // Set top value for 8-bit resolution

    // Configure Timer2 for Fast PWM on OC2A (PB3)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM, clear on compare match
    TCCR2B = (1 << CS21);                                 // No prescaling

    // Configure button interrupt
    EICRA = (1 << ISC01) | (1 << ISC00);  // Falling edge of INT0 generates interrupt request
    EIMSK = (1 << INT0);  // Enable INT0

    DDRD &= ~(1 << BTN);  // Set BTN as input

    PORTD |= (1 << BTN);  // Enable pull-up resistor

    lcd_init(LCD_DISP_ON);  // Initialize LCD
    lcd_clrscr();

    i2c_init();
    _delay_ms(10);
    sei();  // Enable global interrupts
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
    _delay_ms(3);
}

uint16_t read_register(uint8_t reg) {
    uint16_t data;

    i2c_start(0x52);       // TCS34725 write address
    i2c_write(0x80 | reg); // Command + register
    i2c_stop();

    i2c_start(0x53);       // TCS34725 read address
    data = i2c_readAck();  // Read LSB
    data |= (i2c_readNak() << 8); // Read MSB
    i2c_stop();
    
    return data;
}

void read_TCS34725(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    *c = read_register(0x14);
    *r = read_register(0x16);
    *g = read_register(0x18);
    *b = read_register(0x1A);
}

/* State Machine Functions */

void do_nothing(void) { return; }

void reset() {
    set_rgb_led(0, 0, 0); // Turn RGB LED to rgb(0, 0, 0)
    lcd_clrscr(); // Clear LCD
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
    set_rgb_led(128, 0, 0); 
}

void enter_state_1(void) {
    reset();
    set_rgb_led(0, 128, 0);

    PORTD |= (1 << SENSOR_LED);
}

void do_state_1(void) {
    uint16_t raw_r, raw_g, raw_b, raw_c;
    
    // Ensure the sensor is enabled and ready
    enable_TCS34725();  // Make sure this step is done properly

    // Read sensor values
    read_TCS34725(&raw_r, &raw_g, &raw_b, &raw_c);

    // Debug: print raw values to LCD for inspection
    char buffer[16];
    lcd_gotoxy(0, 0);
    sprintf(buffer, "R:%u G:%u", raw_r, raw_g);
    lcd_puts(buffer);

    lcd_gotoxy(0, 1);
    sprintf(buffer, "B:%u C:%u", raw_b, raw_c);
    lcd_puts(buffer);

    // Use raw values directly
    uint16_t r = (raw_r > 255) ? 255 : raw_r;
    uint16_t g = (raw_g > 255) ? 255 : raw_g;
    uint16_t b = (raw_b > 255) ? 255 : raw_b;

    // Set the LED color
    set_rgb_led(r, g, b);
}

const state_t state0 = {0, enter_state_0, do_state_0, do_nothing, 10};
const state_t state1 = {1, enter_state_1, do_state_1, do_nothing, 10};

const state_t* state_table[2][2] = {
    /* STATE  PD0      NO_EVT */
    {/* S0 */ &state1, &state0},
    {/* S1 */ &state0, &state1}
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
