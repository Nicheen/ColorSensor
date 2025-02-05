#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define BUTTON_DEBOUNCE_DELAY_MS 50
#define EVENT_QUEUE_LENGTH 10

// Define GPIO Pins (ATmega328)
#define LED_RED   PB1  // Pin 9
#define LED_GREEN PB2  // Pin 10
#define LED_BLUE  PB3  // Pin 11
#define BTN1      PD2  // Pin 2
#define BTN2      PD3  // Pin 3
#define BTN3      PD4  // Pin 4

volatile uint8_t bit_mask = 0b0001;
volatile uint8_t bit_mask_on = 0b1111;
volatile uint8_t bit_mask_off = 0b0000;
volatile uint8_t button_time = 0;
volatile uint8_t event_flag = 0;

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
    b2_evt = 1,
    b3_evt = 2,
    no_evt = 3
} event_t;

volatile event_t evt = no_evt;

/* PWM LED Fade Effect */
void setup_pwm() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Fast PWM, non-inverting
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler 8
    ICR1 = 255;  // Max count for 8-bit resolution
    OCR1A = 0;   // Initial duty cycle
}

/* Button Interrupts */
ISR(INT0_vect) { evt = b1_evt; }
ISR(INT1_vect) { evt = b2_evt; }
ISR(PCINT2_vect) { evt = b3_evt; }

/* GPIO & Interrupt Setup */
void setup() {
    // Set LEDs as outputs
    DDRB |= (1 << LED_RED) | (1 << LED_GREEN) | (1 << LED_BLUE);

    // Set buttons as inputs with pull-ups
    DDRD &= ~((1 << BTN1) | (1 << BTN2) | (1 << BTN3));
    PORTD |= (1 << BTN1) | (1 << BTN2) | (1 << BTN3);

    // Enable external interrupts for buttons
    EIMSK |= (1 << INT0) | (1 << INT1);  // Enable INT0, INT1
    EICRA |= (1 << ISC01) | (1 << ISC11);  // Falling edge

    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT20);

    setup_pwm();
    sei();  // Enable global interrupts
}

/* State Machine Functions */
void leds_off() {
    PORTB &= ~(1 << LED_RED | 1 << LED_GREEN | 1 << LED_BLUE);
}

void leds_on() {
    PORTB |= (1 << LED_RED | 1 << LED_GREEN | 1 << LED_BLUE);
}

void do_nothing(void) { return; }

void do_state_0(void) {
    bit_mask = (bit_mask << 1) | (bit_mask >> 3);
    PORTB = (PORTB & 0b11110000) | bit_mask;
}

void do_state_1(void) {
    PORTB ^= (1 << LED_RED | 1 << LED_GREEN | 1 << LED_BLUE);
} 

void do_state_2(void) {
    bit_mask = (bit_mask >> 1) | (bit_mask << 3);
    PORTB = (PORTB & 0b11110000) | bit_mask;
}

void enter_state_3(void) {
    leds_off();
    DDRB |= (1 << LED_RED);
    TCCR1A |= (1 << COM1A1);
}

void exit_state_3(void) {
    TCCR1A &= ~(1 << COM1A1);
    leds_off();
}

const state_t state0 = {0, do_nothing, do_state_0, do_nothing, 200};
const state_t state1 = {1, do_nothing, do_state_1, do_nothing, 100};
const state_t state2 = {2, do_nothing, do_state_2, do_nothing, 100};
const state_t state3 = {3, enter_state_3, do_nothing, exit_state_3, 10};

const state_t* state_table[4][4] = {
    /* STATE  GPIO20   GPIO21   GPIO22   NO_EVT */
    {/* S0 */ &state2, &state1, &state3, &state0},
    {/* S1 */ &state0, &state2, &state3, &state1},
    {/* S2 */ &state1, &state0, &state3, &state2},
    {/* S3 */ &state0, &state0, &state0, &state3}
};

int main() {
    setup();

    const state_t* current_state = state_table[0][no_evt];

    while (1) {
        current_state->Enter();
        while (current_state->id == state_table[current_state->id][evt]->id) {
            current_state->Do();
            _delay_ms(current_state->delay_ms);
        }
        current_state->Exit();
        current_state = state_table[current_state->id][evt];
        evt = no_evt;
    }
}
