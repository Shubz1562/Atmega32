#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

// LCD and I2C macros
#define LCD_ADDR 0x4E
#define EN 0b00000100
#define RS 0b00000001
#define BL 0b00001000  // Backlight bit

// ULN2003 Stepper Motor Driver pins
#define STEPPER_DDR DDRB
#define STEPPER_PORT PORTB
#define STEPPER_IN1 PB0
#define STEPPER_IN2 PB1
#define STEPPER_IN3 PB2
#define STEPPER_IN4 PB3

// Buzzer pin (optional - for audio feedback)
#define BUZZER_DDR DDRC
#define BUZZER_PORT PORTC
#define BUZZER_PIN PC0

// Function declarations
void i2c_init();
void i2c_start(uint8_t addr);
void i2c_write(uint8_t data);
void i2c_stop();
void lcd_send(uint8_t data, uint8_t mode);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_clear();
void lcd_gotoxy(uint8_t x, uint8_t y);
void lcd_init();
void lcd_backlight_on();
void lcd_backlight_off();
void keypad_init();
char keypad_getkey();
void stepper_init();
void stepper_step(uint8_t step);
void stepper_rotate_cw(uint16_t steps);
void stepper_rotate_ccw(uint16_t steps);
void stepper_stop();
void unlock_door();
void lock_door();
void buzzer_init();
void buzzer_beep(uint8_t count);
void clear_input();

// Global variables
char password[5] = "1234";
char input[5];
uint8_t index = 0;
uint8_t failed_attempts = 0;
uint8_t backlight_state = 1; // Backlight on by default

// Stepper motor step sequence (full step mode)
uint8_t step_sequence[4] = {
	(1 << STEPPER_IN1),                           // Step 1: 1000
	(1 << STEPPER_IN2),                           // Step 2: 0100
	(1 << STEPPER_IN3),                           // Step 3: 0010
	(1 << STEPPER_IN4)                            // Step 4: 0001
};

uint8_t current_step = 0;

// === I2C ===
void i2c_init() {
	TWSR = 0x00;
	TWBR = 0x47; // 100kHz
	TWCR = (1 << TWEN);
}

void i2c_start(uint8_t addr) {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	TWDR = addr;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void i2c_stop() {
	TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
}

// === LCD ===
void lcd_send(uint8_t data, uint8_t mode) {
	uint8_t upper = data & 0xF0;
	uint8_t lower = (data << 4) & 0xF0;
	
	// Add backlight bit if enabled
	uint8_t backlight_bit = backlight_state ? BL : 0;

	uint8_t data_arr[4] = {
		upper | mode | EN | backlight_bit,
		upper | mode | backlight_bit,
		lower | mode | EN | backlight_bit,
		lower | mode | backlight_bit
	};

	for (int i = 0; i < 4; i++) {
		i2c_start(LCD_ADDR);
		i2c_write(data_arr[i]);
		i2c_stop();
		// _delay_ms(1); // Removed for faster LCD communication
	}
}

void lcd_send_cmd(uint8_t cmd) {
	lcd_send(cmd, 0);
}

void lcd_send_data(uint8_t data) {
	lcd_send(data, RS);
}

void lcd_send_string(char *str) {
	while (*str) lcd_send_data(*str++);
}

void lcd_clear() {
	lcd_send_cmd(0x01);
	_delay_ms(1); // Reduced clear delay
}

void lcd_gotoxy(uint8_t x, uint8_t y) {
	lcd_send_cmd(0x80 + (y ? 0x40 : 0x00) + x);
}

void lcd_init() {
	i2c_init();
	_delay_ms(50);
	lcd_send_cmd(0x33);
	lcd_send_cmd(0x32);
	lcd_send_cmd(0x28);
	lcd_send_cmd(0x0C);
	lcd_send_cmd(0x06);
	lcd_send_cmd(0x01);
	_delay_ms(2);
	lcd_backlight_on(); // Turn on backlight after init
}

void lcd_backlight_on() {
	backlight_state = 1;
	// Send a dummy command to update backlight
	i2c_start(LCD_ADDR);
	i2c_write(BL);
	i2c_stop();
}

void lcd_backlight_off() {
	backlight_state = 0;
	// Send a dummy command to update backlight
	i2c_start(LCD_ADDR);
	i2c_write(0x00);
	i2c_stop();
}

// === Keypad ===
char keys[4][4] = {
	{'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'}
};

void keypad_init() {
	DDRD = 0x0F;   // Rows PD0–PD3 output, columns PD4–PD7 input
	PORTD = 0xFF;  // Enable pull-ups
}

char keypad_getkey() {
	for (uint8_t row = 0; row < 4; row++) {
		PORTD = ~(1 << row);  // Activate one row at a time
		// No delay for maximum speed keypad scanning
		for (uint8_t col = 0; col < 4; col++) {
			if (!(PIND & (1 << (col + 4)))) {
				while (!(PIND & (1 << (col + 4))));  // Wait for key release
				_delay_ms(10); // Minimal debounce delay
				return keys[row][col];
			}
		}
	}
	return 0;
}

// === Stepper Motor (ULN2003) ===
void stepper_init() {
	STEPPER_DDR |= (1 << STEPPER_IN1) | (1 << STEPPER_IN2) |
	(1 << STEPPER_IN3) | (1 << STEPPER_IN4);
	stepper_stop(); // All pins low initially
}

void stepper_step(uint8_t step) {
	STEPPER_PORT = (STEPPER_PORT & 0xF0) | step_sequence[step & 0x03];
	_delay_ms(2); // Fast step delay for smooth operation
}

void stepper_rotate_cw(uint16_t steps) {
	for (uint16_t i = 0; i < steps; i++) {
		stepper_step(current_step);
		current_step = (current_step + 1) % 4;
	}
}

void stepper_rotate_ccw(uint16_t steps) {
	for (uint16_t i = 0; i < steps; i++) {
		current_step = (current_step - 1) % 4;
		if (current_step > 3) current_step = 3; // Handle underflow
		stepper_step(current_step);
	}
}

void stepper_stop() {
	STEPPER_PORT &= 0xF0; // Turn off all stepper pins
}

void unlock_door() {
	stepper_rotate_cw(512); // Rotate ~90 degrees (2048 steps = 360°)
	stepper_stop(); // Save power
}

void lock_door() {
	stepper_rotate_ccw(512); // Rotate back ~90 degrees
	stepper_stop(); // Save power
}

// === Buzzer (Optional) ===
void buzzer_init() {
	BUZZER_DDR |= (1 << BUZZER_PIN);
	BUZZER_PORT &= ~(1 << BUZZER_PIN);
}

void buzzer_beep(uint8_t count) {
	for (uint8_t i = 0; i < count; i++) {
		BUZZER_PORT |= (1 << BUZZER_PIN);
		_delay_ms(30);
		BUZZER_PORT &= ~(1 << BUZZER_PIN);
		_delay_ms(30);
	}
}

// === Utility Functions ===
void clear_input() {
	memset(input, 0, sizeof(input));
	index = 0;
}

// === MAIN ===
int main(void) {
	keypad_init();
	lcd_init();
	stepper_init();
	buzzer_init(); // Optional
	
	lcd_clear();
	lcd_send_string("Door Lock System");
	lcd_gotoxy(0, 1);
	lcd_send_string("Ready...");
	_delay_ms(100);
	
	lcd_clear();
	lcd_send_string("Enter Password:");
	lcd_gotoxy(0, 1);

	while (1) {
		char key = keypad_getkey();
		if (key) {
			// Handle special keys
			if (key == '*') {
				// Clear current input
				clear_input();
				lcd_gotoxy(0, 1);
				lcd_send_string("                "); // Clear line
				lcd_gotoxy(0, 1);
				continue;
			}
			else if (key == '#') {
				// Early submit (for passwords shorter than 4 digits)
				if (index > 0) {
					input[index] = '\0';
					goto validate_password;
				}
				continue;
			}
			else if (key >= 'A' && key <= 'D') {
				// Special function keys
				if (key == 'A') {
					// Toggle backlight
					if (backlight_state) {
						lcd_backlight_off();
						} else {
						lcd_backlight_on();
					}
				}
				continue;
			}
			
			// Regular digit input
			if (index < 4) {
				lcd_send_data('*');
				input[index++] = key;
				buzzer_beep(1); // Single beep for keypress
			}

			if (index == 4) {
				validate_password:
				input[index] = '\0';
				_delay_ms(50);
				lcd_clear();

				if (strcmp(input, password) == 0) {
					lcd_send_string("Access Granted");
					lcd_gotoxy(0, 1);
					lcd_send_string("Door Unlocked");
					buzzer_beep(2); // Two beeps for success
					unlock_door();
					_delay_ms(2000); // Keep door unlocked
					lock_door();
					failed_attempts = 0; // Reset failed attempts
					} else {
					failed_attempts++;
					lcd_send_string("Access Denied");
					lcd_gotoxy(0, 1);
					if (failed_attempts >= 3) {
						lcd_send_string("System Locked");
						buzzer_beep(5); // Five beeps for lockout
						_delay_ms(2000); // 2 second lockout
						failed_attempts = 0;
						} else {
						lcd_send_string("Try Again");
						buzzer_beep(3); // Three beeps for failure
						_delay_ms(500);
					}
				}

				clear_input();
				lcd_clear();
				lcd_send_string("Enter Password:");
				lcd_gotoxy(0, 1);
			}
		}
	}
}