#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#define LCD_RS PE0
#define LCD_RW PE1
#define LCD_E PE2

// Define LCD data port
#define LCD_DATA_PORT PORTA
#define LCD_DATA_DDR DDRA
#define LCD_CTRL_PORT PORTE
#define LCD_CTRL_DDR DDRE

#define ADC0808_A PD0
#define ADC0808_B PD1
#define ADC0808_C PD2
#define ADC0808_ALE PD3
#define ADC0808_START PD4
#define ADC0808_EOC PD6
#define ADC0808_OE PD7

#define ADC0808_DATA_PORT PORTC
#define ADC0808_DATA_PIN PINC
#define ADC0808_DATA_DDR DDRC

void LCD_command(unsigned char cmd) {
	LCD_DATA_PORT = cmd;
	LCD_CTRL_PORT &= ~(1 << LCD_RS); // RS = 0 for command
	LCD_CTRL_PORT &= ~(1 << LCD_RW); // RW = 0 for write
	LCD_CTRL_PORT |= (1 << LCD_E);   // Enable pulse
	_delay_us(1);
	LCD_CTRL_PORT &= ~(1 << LCD_E);
	_delay_ms(2);
}

void LCD_data(unsigned char data) {
	LCD_DATA_PORT = data;
	LCD_CTRL_PORT |= (1 << LCD_RS);  // RS = 1 for data
	LCD_CTRL_PORT &= ~(1 << LCD_RW); // RW = 0 for write
	LCD_CTRL_PORT |= (1 << LCD_E);   // Enable pulse
	_delay_us(1);
	LCD_CTRL_PORT &= ~(1 << LCD_E);
	_delay_ms(2);
}

void LCD_init() {
	LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E);
	LCD_DATA_DDR = 0xFF; // Set data port to output

	_delay_ms(20); // Wait for LCD to power up
	LCD_command(0x38); // 8-bit mode, 2 lines, 5x7 format
	LCD_command(0x0C); // Display ON, Cursor OFF
	LCD_command(0x01); // Clear display
	_delay_ms(2);
	LCD_command(0x06); // Increment cursor
}

void LCD_string(const char* str) {
	while (*str) {
		LCD_data(*str++);
	}
}

void LCD_integer(int num) {
	char buffer[16];
	sprintf(buffer, "%d", num); // Convert integer to string
	LCD_string(buffer); // Display the string on LCD
}

void LCD_set_cursor(unsigned char row, unsigned char col) {
	unsigned char pos;
	if (row == 1) pos = 0x80 + col;
	else pos = 0xC0 + col;
	LCD_command(pos);
}

int GetKeyPressed(void) {
	// Define key mapping (adjust as per actual keypad configuration)
	int keymap[4][4] = {
		{0, 1, 2, 15},
		{4, 5, 6, 10},
		{8, 9, 3, 11},
		{12, 13, 14, 15}
	};

	// Scan rows
	for (int row = 0; row < 4; row++) {
		// Set one row low
		PORTB = ~(1 << row);
		_delay_ms(1); // Allow signals to settle

		// Check columns
		for (int col = 0; col < 4; col++) {
			if (!(PINB & (1 << (col + 4)))) {
				return keymap[row][col]; // Return corresponding key index
			}
		}
	}

	return -1; // No key pressed
}

void clearBuffer(char* buffer, int* index) {
	for (int i = 0; i < 16; i++) {
		buffer[i] = '\0';
	}
	*index = 0;
}

void PWM_init() {
	// Set Fast PWM mode, non-inverting
	TCCR1A = (1 << WGM10) | (1 << COM1A1); // Fast PWM 8-bit, non-inverting mode
	TCCR1B = (1 << WGM12) | (1 << CS11); // Fast PWM, prescaler 8
	DDRD |= (1 << PD5); // Set PD5 (OC1A) as output for PWM signal
}

void set_PWM(uint8_t duty) {
	OCR1A = duty; // Set duty cycle
}

void ADC0808_init() {
	DDRD |= (1 << ADC0808_A) | (1 << ADC0808_B) | (1 << ADC0808_C) | (1 << ADC0808_ALE) | (1 << ADC0808_START) | (1 << ADC0808_OE);
	DDRD &= ~(1 << ADC0808_EOC); // EOC as input
	ADC0808_DATA_DDR = 0x00; // Data port as input
}

uint8_t ADC0808_read(uint8_t channel) {
	PORTD = (PORTD & 0xF8) | (channel & 0x07); // Set channel on A, B, C
	PORTD |= (1 << ADC0808_ALE); // Generate ALE pulse
	_delay_us(1);
	PORTD &= ~(1 << ADC0808_ALE);
	PORTD |= (1 << ADC0808_START); // Start conversion
	_delay_us(1);
	PORTD &= ~(1 << ADC0808_START);

	// Wait for end of conversion
	while (!(PIND & (1 << ADC0808_EOC)));

	PORTD |= (1 << ADC0808_OE); // Enable output
	_delay_us(1);
	uint8_t adc_value = ADC0808_DATA_PIN;
	PORTD &= ~(1 << ADC0808_OE);

	return adc_value;
}

int main(void) {
	LCD_init();
	PWM_init();
	ADC0808_init();

	char digit[20] = {'7','8','9','3','4','5','6','*','1','2','3','-','o','0','=','+','\0'};

	DDRB = 0x0F; // Set PB0-PB3 as output (rows) and PB4-PB7 as input (columns) with pull-up resistors
	PORTB = 0xF0; // Enable pull-up resistors on PB4-PB7

	int key;
	char inputBuffer[16] = "";
	int bufferIndex = 0;
	int speed = 0;
	uint8_t adc_value;
	uint8_t mode = 0; // 0 for keypad mode, 1 for ADC mode

	LCD_command(0x01); // Clear display
	LCD_string("Enter Speed:");

	while (1) {
		if (mode == 0) { // Keypad mode
			key = GetKeyPressed();
			if (key != -1) {
				if (key >= 0 && key <= 9) {
					inputBuffer[bufferIndex++] = digit[key];
					inputBuffer[bufferIndex] = '\0';
					LCD_set_cursor(2, 0); // Set cursor to 2nd row, 1st column
					LCD_string(inputBuffer);
				}
				else if (key == 13) {
					inputBuffer[bufferIndex++] = '0';
					inputBuffer[bufferIndex] = '\0';
					LCD_set_cursor(2, 0); // Set cursor to 2nd row, 1st column
					LCD_string(inputBuffer);
				}
				else if (digit[key] == '=') {
					speed = atoi(inputBuffer); // Save the entered speed value
					if (speed > 255) speed = 255; // Limit speed to 255
					set_PWM(speed); // Set PWM duty cycle based on speed
					LCD_command(0x01); // Clear display
					LCD_string("Speed: ");
					LCD_integer(speed);
				}
				else if (digit[key] == 'o') {
					clearBuffer(inputBuffer, &bufferIndex);
					speed = 0; // Reset speed
					set_PWM(speed); // Set PWM duty cycle to 0
					LCD_command(0x01); // Clear display
					LCD_string("Speed: ");
					LCD_integer(speed);
				}
				else if (digit[key] == '+') {
					mode = 1; // Switch to ADC mode
					LCD_command(0x01); // Clear display
					LCD_string("ADC Mode:");
				}
				_delay_ms(500); // Debounce delay
			}
			} else if (mode == 1) { // ADC mode
				// Check for switch back to keypad mode
			key = GetKeyPressed();
			if (key != -1 && digit[key] == '+') {
				mode = 0; // Switch to keypad mode
				LCD_command(0x01); // Clear display
				LCD_string("Enter Speed:");
			}
			adc_value = ADC0808_read(0); // Read ADC value from channel 0
			speed = adc_value; // Use ADC value as speed
			if (speed > 255) speed = 255;
			set_PWM(speed); // Set PWM duty cycle based on ADC value
			LCD_set_cursor(2, 0); // Set cursor to 2nd row, 1st column
			LCD_string("Speed: ");
			LCD_integer(speed);
			_delay_ms(10); // Update interval

		}
	}
}
