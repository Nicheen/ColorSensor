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
    // Enable PWM on OC0A (PD6) and OC0B (PD5)
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
    OCR1A = r; // PB1 (OC1A) --> LED1_RED
    OCR1B = g; // PB2 (OC1B) --> LED1_GREEN
    OCR2A = b; // PB3 (OC2A) --> LED1_BLUE
}

void set_led_from_color(uint8_t red, uint8_t green, uint8_t blue) {
    // Threshold to detect when two colors are "close"
    const uint8_t THRESHOLD = 30; 

    if (red > green && red > blue) {
        if (green >= red - THRESHOLD) {
            set_rgb_led(255, 128, 0);  // **Orange**
        } else if (blue >= red - THRESHOLD) {
            set_rgb_led(255, 0, 255);  // **Purple**
        } else {
            set_rgb_led(255, 0, 0);  // **Red**
        }
    } 
    else if (green > red && green > blue) {
        if (red >= green - THRESHOLD) {
            set_rgb_led(255, 255, 0);  // **Yellow**
        } else if (blue >= green - THRESHOLD) {
            set_rgb_led(0, 255, 255);  // **Cyan**
        } else {
            set_rgb_led(0, 255, 0);  // **Green**
        }
    } 
    else if (blue > red && blue > green) {
        if (red >= blue - THRESHOLD) {
            set_rgb_led(255, 0, 255);  // **Purple**
        } else if (green >= blue - THRESHOLD) {
            set_rgb_led(0, 255, 255);  // **Cyan**
        } else {
            set_rgb_led(0, 0, 255);  // **Blue**
        }
    } 
    else {
        set_rgb_led(255, 255, 255);  // **White (All values similar)**
    }
}

void init_adc() {
    // Set POT (PD1) as input (it's already input by default, just ensuring no conflicts)
    DDRC &= ~(1 << POT);

    ADMUX |= (1 << REFS0) | (1 << MUX1) | (1 << MUX0);
    
    ADCSRA |= ((1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0));
}

uint16_t read_adc() {
    // Start conversion by setting the appropriate bit in the ADCSRA register
    ADCSRA |= (1 << ADSC);

    // Wait for the conversion to complete (ADS = 0 when the conversion is done)
    while (ADCSRA & (1 << ADSC));

    return ADC;
}

/* GPIO & Interrupt Setup */
void init() {
    init_timer0();

    // Set LEDs as outputs
    DDRB |= (1 << LED1_RED) | (1 << LED1_GREEN) | (1 << LED1_BLUE);
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

    // Set integration time to 154 ms (0xC0)
    i2c_start(0x52);
    i2c_write(0x80 | 0x01); // ATIME register
    i2c_write(0xC0);        // 0xC0 gives ~154 ms integration time
    i2c_stop();
    _delay_ms(10);

    i2c_start(0x52);
    i2c_write(0x80 | 0x0F); // Control register
    i2c_write(0x01);        // Set gain to 4x
    i2c_stop();
    _delay_ms(10);
}

uint16_t read_register(uint8_t reg) {
    i2c_start(0x52);       // TCS34725 write address
    i2c_write(0x80 | reg); // Command + register
    i2c_rep_start(0x53);

    uint16_t low = i2c_readAck();  // Read LSB
    uint16_t high = i2c_readNak(); // Read MSB
    
    i2c_stop();
    
    return (high << 8) | low;
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
    set_rgb_led(255, 0, 0); 
}

void enter_state_1(void) {
    reset();
    enable_TCS34725();
    // For a visual cue, set a fixed color on LED1 (or both)
    set_rgb_led(0, 255, 0);
    PORTD |= (1 << SENSOR_LED);
}

void do_state_1(void) {

    // Read sensor values
    uint16_t c = read_register(0x14);
    uint16_t r = read_register(0x16);
    uint16_t g = read_register(0x18);
    uint16_t b = read_register(0x1A);
    
    // Color spectrum for the photodiode shows that
    const float GREEN_COMP = 1.36;
    const float BLUE_COMP = 1.58;

    const uint8_t THRESHOLD = 50;

    uint8_t red, green, blue;
    if (c == 0) {
        red = 0;
        green = 0;
        blue = 0;
    } else {
        red   = (uint8_t)(pow(((float)(r) / (float)c), 2.5) * 2.5 * 255.0); // 2
        green = (uint8_t)(pow(((float)(g) / (float)c), 2.5) * 3 * GREEN_COMP * 255.0); // 5
        blue  = (uint8_t)(pow(((float)(b) / (float)c), 2.5) * 6 * BLUE_COMP * 255.0); // 4
        
    }

    
    char buffer_1[16];
    char buffer_2[16];

    lcd_gotoxy(0, 0);
    sprintf(buffer_1, "%3d %3d %3d", red, green, blue);
    lcd_puts(buffer_1);

    // Set the LED color
    uint16_t pot_value = read_adc() >> 2;  // Read ADC value from the potentiometer

    lcd_gotoxy(0, 1);
    sprintf(buffer_2, "P:%3d L:%6d", pot_value, c);  // Display potentiometer value on LCD
    lcd_puts(buffer_2);

    float pot_value_f = (float)pot_value / 255.0;
    set_rgb_led(red * pot_value_f, green * pot_value_f, blue * pot_value_f);
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
