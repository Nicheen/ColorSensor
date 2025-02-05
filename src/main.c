#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <lcd.h>


/*            PD0 - PD7           */
#define CS_FREQ     PD0  // Pin 2
#define BTN         PD3  // Pin 5

/*            PB0 - PB7           */
#define LED_GREEN   PB1  // Pin 15
#define LED_RED     PB2  // Pin 16
#define LED_BLUE    PB3  // Pin 17
/*   PB4 - PB7 are used for LCD   */

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

/* Button Interrupts */
ISR(INT0_vect) {
    evt = b1_evt;  
}

void delay_ms(uint8_t ms) {
    while (ms > 0) {
        _delay_ms(1); // Calls _delay_ms with a constant value
        ms--;
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
    // Set LEDs as outputs
    DDRB |= (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);

    // Configure Timer1 for Fast PWM on OC1A (PB1) and OC1B (PB2)
    TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, clear on compare match
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);    // No prescaling
    ICR1 = 0xFF; // Set top value for 8-bit resolution

    // Configure Timer2 for Fast PWM on OC2A (PB3)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM, clear on compare match
    TCCR2B = (1 << CS20);                                 // No prescaling

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
    lcd_puts("Press btn (S2)");
}

void do_state_0(void) {
    set_rgb_led(255, 255, 255); 
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

    set_rgb_led(255, 128, 20); 
}

const state_t state0 = {0, enter_state_0, do_state_0, do_nothing, 10};
const state_t state1 = {1, enter_state_1, do_state_1, do_nothing, 500};

const state_t* state_table[3][2] = {
    /* STATE  PD0      NO_EVT */
    {/* S0 */ &state1, &state0},
    {/* S1 */ &state0, &state1}
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
