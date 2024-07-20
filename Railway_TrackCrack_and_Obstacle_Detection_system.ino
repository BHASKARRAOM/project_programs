#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd.h"

#define IR_SENSOR_PIN  PD2
#define ULTRASONIC_SENSOR_PIN PD3
#define MOTOR_PIN  PB0
#define BUZZER_PIN  PB1

void init_ports() {
    DDRD &= ~(1 << IR_SENSOR_PIN); // Set IR_SENSOR_PIN as input
    DDRD &= ~(1 << ULTRASONIC_SENSOR_PIN); // Set ULTRASONIC_SENSOR_PIN as input
    DDRB |= (1 << MOTOR_PIN); // Set MOTOR_PIN as output
    DDRB |= (1 << BUZZER_PIN); // Set BUZZER_PIN as output
}

void init_lcd() {
    lcd_init(LCD_DISP_ON_CURSOR_BLINK); // Initialize LCD
    lcd_clrscr(); // Clear screen
}

void display_message(char* msg) {
    lcd_clrscr();
    lcd_puts(msg);
}

void stop_motor() {
    PORTB &= ~(1 << MOTOR_PIN); // Turn off the motor
}

void activate_buzzer() {
    PORTB |= (1 << BUZZER_PIN); // Turn on the buzzer
}

void deactivate_buzzer() {
    PORTB &= ~(1 << BUZZER_PIN); // Turn off the buzzer
}

int main(void) {
    init_ports();
    init_lcd();
    uint8_t ir_sensor_status = 0;
    uint8_t ultrasonic_sensor_status = 0;

    while (1) {
        ir_sensor_status = PIND & (1 << IR_SENSOR_PIN); // Read IR sensor status
        ultrasonic_sensor_status = PIND & (1 << ULTRASONIC_SENSOR_PIN); // Read ultrasonic sensor status

        if (ir_sensor_status == 0) { // Check if IR sensor detects crack
            display_message("Crack Detected");
            stop_motor();
            activate_buzzer();
        } else if (ultrasonic_sensor_status == 0) { // Check if ultrasonic sensor detects obstacle
            display_message("Obstacle Detected");
            stop_motor();
            activate_buzzer();
        } else {
            display_message("System OK");
            deactivate_buzzer();
        }

        _delay_ms(500); // Delay for debounce
    }
}
