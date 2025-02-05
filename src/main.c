#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <lcd.h>

// Define GPIO Pins (ATmega328)
#define LED_GREEN   PB1  // Pin 9
#define LED_RED     PB2  // Pin 10
#define LED_BLUE    PB3  // Pin 11

// Buttons
#define BTN         PD3  // Pin 0

// Inputs
#define FREQ        PD0

/* Function pointer primitive */ 
typedef void (*state_func_t)(void);

typedef struct {
    uint8_t id;
    state_func_t Enter;
    state_func_t Do;
    state_func_t Exit;
    uint16_t delay_ms;
} state_t;

typedef enum {
    b1_evt = 0,
    no_evt = 1
} event_t;

volatile event_t evt = no_evt;

void delay_ms(uint16_t ms) {
    while (ms--) {
        _delay_ms(1); // Calls _delay_ms with a constant value
    }
}

/* Button Interrupts */
ISR(INT0_vect) {
    evt = b1_evt;
}

void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    // PWM range (0 -> 255)
    OCR1B = r; // PB2 (LED_RED)
    OCR1A = g; // PB1 (LED_GREEN)
    OCR2A = b; // PB3 (LED_BLUE)
}

/* GPIO & Interrupt Setup */
void init() {
    // Set up ADC for frequency input
    ADMUX  = ( 0 << REFS1 ) | ( 1 << REFS0 )
         | ( 0 << ADLAR )
         | ( 0 << MUX3 )  | ( 1 << MUX2 )
         | ( 0 << MUX1 )  | ( 1 << MUX0 );
    ADCSRA = ( 1 << ADEN )  | ( 1 << ADSC )
         | ( 0 << ADATE ) | ( 0 << ADIF )  | ( 0 << ADIE )
         | ( 0 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );

    // Set LEDs as outputs
    DDRB |= (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);

    // setting up TIMER0
    TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0)
            | (0 << WGM01)  | (0 << WGM00);
    TCCR0B = (0 << WGM02)  | (0 << CS02)   | (1 << CS01)   | (0 << CS00);
    TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0);
    
    // Configure Timer1 for Fast PWM on OC1A (PB1) and OC1B (PB2)
    TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);    // No prescaling
    ICR1 = 0xFF; // Set top value for 8-bit resolution

    // Configure Timer2 for Fast PWM on OC2A (PB3)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM, clear on compare match
    TCCR2B = (1 << CS20);                                 // No prescaling

    // Initialize duty cycles to 0
    OCR1A = 0; // PB1 (LED_GREEN)
    OCR1B = 0; // PB2 (LED_RED)
    OCR2A = 0; // PB3 (LED_BLUE)

    // Configure button interrupt
    EICRA = (1 << ISC01) | (1 << ISC00);  // Falling edge of INT0 generates interrupt request
    EIMSK = (1 << INT0);  // Enable INT0

    DDRD &= ~(1 << BTN);  // Set BTN as input
    PORTD |= (1 << BTN);  // Enable pull-up resistor

    lcd_init(LCD_DISP_ON);  // Initialize LCD
    lcd_clrscr();
    sei();  // Enable global interrupts
}

/* State Machine Functions */
void reset() {
    OCR1A = 0; // PB1 (LED_GREEN)
    OCR1B = 0; // PB2 (LED_RED)
    OCR2A = 0; // PB3 (LED_BLUE)
    lcd_clrscr(); // Clear LCD
}

void do_nothing(void) { return; }

void enter_state_0(void) {
    reset();

    lcd_gotoxy(0, 0);
    lcd_puts("Ready to scan?");

    lcd_gotoxy(0, 1);
    lcd_puts("Press btn (S2)");

    set_rgb_led(64, 224, 208); // Turquoise
}

void do_state_0(void) {
    return;
}

void enter_state_1(void) {
    reset();
}

void do_state_1(void) {
    static uint8_t dot_count = 0; // Track the number of dots
    
    // Update the LCD
    lcd_clrscr();
    lcd_gotoxy(0, 0);
    lcd_puts("Scanning"); // Base text

    for (uint8_t i = 0; i < dot_count; i++) {
        lcd_putc('.'); // Add dots dynamically
    }

    dot_count = (dot_count + 1) % 4; // Cycle through 0, 1, 2, 3
}


void enter_state_2(void) {
    reset();
    lcd_gotoxy(0, 0);
    lcd_puts("Hello world (B)");
}

void do_state_2(void) {
    return;
}

const state_t state0 = {0, enter_state_0, do_state_0, do_nothing, 100};
const state_t state1 = {1, enter_state_1, do_state_1, do_nothing, 500};
const state_t state2 = {2, enter_state_2, do_state_2, do_nothing, 100};

const state_t* state_table[3][2] = {
    /* STATE  PD0      NO_EVT */
    {/* S0 */ &state1, &state0},
    {/* S1 */ &state2, &state1},
    {/* S2 */ &state0, &state2}
};

int main() {
    init();

    const state_t* current_state = state_table[0][no_evt];
    evt = no_evt;

    while (1) {
        current_state->Enter();
        while (current_state->id == state_table[current_state->id][evt]->id) 
        {
            current_state->Do();
            delay_ms(current_state->delay_ms);
        }
        current_state->Exit();
        current_state = state_table[current_state->id][evt];
        evt = no_evt;
    }
}
