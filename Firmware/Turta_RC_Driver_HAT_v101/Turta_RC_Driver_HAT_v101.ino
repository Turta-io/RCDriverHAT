/*  Turta RC Driver HAT
 *  Microcontroller Firmware
 *  Version 1.01
 *  Released: July 16th, 2019
 *  Distributed under the terms of the MIT license.
 *  Visit https://docs.turta.io for documentation.
 */

/*  Microcontroller Information
 *  Model:          Atmega328PB
 *  VCC:            5V
 *  IO:             5V
 *  Crystal:        16 MHz
 *  Programming:    ICSP
 *  Communication:  I2C Slave
 *  I2C Address:    0x42
 *  Functions:      Decoding PWM
 *                  Reading analog inputs
 *                  Calculating RPM
 */

#include <Wire.h>                   // I2C Communication libraries
#include <avr/wdt.h>                // Watchdog timer libraries

// Macros
#define BETWEEN(x, a, b) (((a) <= (x)) && ((x) <= (b))) // Checks if x is between a and b
#define OUTSIDE(x, a, b) (((x) < (a)) || ((b) < (x)))   // Checks if c is outside of a and b

// Configuration
#define PWM_MIN_MS     500          // Minimum PWM pulse length in milliseconds
#define PWM_MAX_MS     2500         // Maximum PWM pulse length in milliseconds
#define PWM_TIMEOUT_US 2000000      // Maximum timeframe for last PWM pulse can be read
#define TACH_MIN_MS    5            // Minimum tach pulse length in milliseconds

// Pin definitions
#define TACH_L         8             // B0 PCINT0
#define TACH_R         9             // B1 PCINT1
#define AIN_1          14            // C0 ADC0
#define AIN_2          15            // C1 ADC1
#define AIN_3          16            // C2 ADC2
#define AIN_4          17            // C3 ADC3
#define PWM_S          1             // D1 PCINT17
#define PWM_T          2             // D2 PCINT18
#define PWM_3          3             // D3 PCINT19
#define PWM_4          4             // D4 PCINT20
#define PE0            23            // Atmega328PB Series new pin, do not write to this pin
#define PE1            24            // Atmega328PB Series new pin, do not write to this pin

// Masks
#define PWM_MASK       0x1E          // All PWM pins mask
#define TACH_MASK      0x03          // All tach pins mask
#define PWM_S_MASK     0x02          // PWM S pin mask
#define PWM_T_MASK     0x04          // PWM T pin mask
#define PWM_3_MASK     0x08          // PWM 3 pin mask
#define PWM_4_MASK     0x10          // PWM 4 pin mask
#define TACH_L_MASK    0x01          // Tach L pin mask
#define TACH_R_MASK    0x02          // Tach R pin mask

// I2C Registers
#define I2C_ADDRESS    0x42          // I2C device address
#define MCU_WHO_AM_I   0x00          // Read ID data and a counter for development purposes
#define MCU_PWM_S      0x01          // Read Steering PWM channel
#define MCU_PWM_T      0x02          // Read Rhrottle PWM channel
#define MCU_PWM_3      0x03          // Read Aux 3 PWM channel
#define MCU_PWM_4      0x04          // Read Aux 4 PWM channel
#define MCU_PWM_ALL    0x05          // Read all PWM channel
#define MCU_AN_1       0x11          // Read analog input channel 1
#define MCU_AN_2       0x12          // Read analog input channel 2
#define MCU_AN_3       0x13          // Read analog input channel 3
#define MCU_AN_4       0x14          // Read analog input channel 4
#define MCU_AN_ALL     0x15          // Read all analog input channels
#define MCU_TACH_L     0x21          // Read tachometer L RPM
#define MCU_TACH_R     0x22          // Read tachometer R RPM
#define MCU_TACH_ALL   0x23          // Read both tachometer RPMs
#define I_AM_MCU       0xBD          // Who am I response

// PWM variables
volatile uint32_t pwm_s_in;          // PWM S pulse rising edge event time
volatile uint32_t pwm_t_in;          // PWM T pulse rising edge event time
volatile uint32_t pwm_3_in;          // PWM 3 pulse rising edge event time
volatile uint32_t pwm_4_in;          // PWM 4 pulse rising edge event time

// Tach variables
volatile uint32_t prev_tach_l = 0;   // Previous tach L pulse event time for timespan calculating
volatile uint32_t prev_tach_r = 0;   // Previous tach R pulse event time for timespan calculating
volatile uint8_t tach_l_rev = 0;     // Left tach pulse counter
volatile uint8_t tach_r_rev = 0;     // Right tach pulse counter

// Port change interrupt variables
uint8_t prev_pwm =         0x1E;     // Previous PWM port state
uint8_t prev_tach =        0x03;     // Previous tach port state
uint8_t changed_pwm =      0x00;     // Changed PWM pins
uint8_t changed_tach =     0x00;     // Changed tach pins

// Communication variables
uint8_t reg_addr =             0;    // I2C Register address
uint8_t who_am_i_cnt =         0;    // Who am I counter
uint8_t payload[8];                  // I2C Transmit payload

// Measurement results
volatile uint16_t pwm_s_res =  0;    // PWM Steering
volatile uint16_t pwm_t_res =  0;    // PWM Throttle
volatile uint16_t pwm_3_res =  0;    // PWM Aux 3
volatile uint16_t pwm_4_res =  0;    // PWM Aux 4
volatile uint8_t tach_l_res =  0;    // Tach L RPM
volatile uint8_t tach_r_res =  0;    // Tach R RPM
volatile uint16_t an_1_res =   0;    // A1 Val
volatile uint16_t an_2_res =   0;    // A2 Val
volatile uint16_t an_3_res =   0;    // A3 Val
volatile uint16_t an_4_res =   0;    // A4 Val

// Port Change Interrupt for PWM pins
ISR(PCINT2_vect) {                                   // Port D Change Interrupt
  changed_pwm = prev_pwm ^ (PIND & PWM_MASK);        // Check changed pins
  prev_pwm = PIND & PWM_MASK;                        // Save PWM value to history

  // PWM Steering Channel
  if ((changed_pwm & PWM_S_MASK) == PWM_S_MASK) {    // If steering channel is changed
    if ((PIND & PWM_S_MASK) == PWM_S_MASK)           // If rising edge
      pwm_s_in = micros();                           // Save rising edge event time
    else                                             // If falling edge
      pwm_s_res = (uint16_t)(micros() - pwm_s_in);   // Calculate timespan
  }

  // PWM Throttle Channel
  if ((changed_pwm & PWM_T_MASK) == PWM_T_MASK) {    // Id throttle channel is changed
    if ((PIND & PWM_T_MASK) == PWM_T_MASK)           // If rising edge
      pwm_t_in = micros();                           // Save rising edge event time
    else                                             // If falling edge
      pwm_t_res = (uint16_t)(micros() - pwm_t_in);   // Calculate timespan
  }

  // PWM Aux 3 Channel
  if ((changed_pwm & PWM_3_MASK) == PWM_3_MASK) {    // If aux 3 channel is changed
    if ((PIND & PWM_3_MASK) == PWM_3_MASK)           // If rising edge
      pwm_3_in = micros();                           // Save rising edge event time
    else                                             // If falling edge
      pwm_3_res = (uint16_t)(micros() - pwm_3_in);   // Calculate timespan
  }

  // PWM Aux 4 Channel
  if ((changed_pwm & PWM_4_MASK) == PWM_4_MASK) {    // If aux 4 channel is changed
    if ((PIND & PWM_4_MASK) == PWM_4_MASK)           // If rising edge
      pwm_4_in = micros();                           // Save rising edge event time
    else                                             // If falling edge
      pwm_4_res = (uint16_t)(micros() - pwm_4_in);   // Calculate timespan
  }
}

// Port Change Interrupt for tach pins
ISR(PCINT0_vect) {                                   // Port B Change Interrupt
  changed_tach = prev_tach ^ (PINB & TACH_MASK);     // Check changed pins
  prev_tach = PINB & TACH_MASK;                      // Save tach value to history

  // Tachometer Left Channel
  if ((changed_tach & TACH_L_MASK) == TACH_L_MASK) { // Tach A channel is changed
    if ((PINB & TACH_L_MASK) != TACH_L_MASK) {       // If falling edge
      if ((millis() - prev_tach_l) > TACH_MIN_MS) {  // Filter spikes
        tach_l_rev++;                                // Increment tach counter
      }
      prev_tach_l = millis();                        // Save event time for the next spike filter
    }
  }

  // Tachometer Right Channel
  if ((changed_tach & TACH_R_MASK) == TACH_R_MASK) { // Tach B channel is changed
    if ((PINB & TACH_R_MASK) != TACH_R_MASK) {       // If falling edge
      if ((millis() - prev_tach_r) > TACH_MIN_MS) {  // Filter spikes
        tach_r_rev++;                                // Increment tach counter
      }
      prev_tach_r = millis();                        // Save event time for the next spike filter
    }
  }
}

// I2C Event: Master writes data
void serial_receive_event(int msg_length) {
  PCICR = 0x00;                             // Disable PC interrupts
  (void)msg_length;                         // Eliminate compiler warning for unused parameter
  while(Wire.available())                   // While data is available to read
    reg_addr = Wire.read();                 // Read the address data 
  PCICR = 0x05;                             // Enable PC interrupts
}

// I2C Event: Master requests data
void serial_request_event() {
  PCICR = 0x00;                             // Disable PC interrupts

  switch (reg_addr) {                       // Switch to the register
    case MCU_WHO_AM_I:                      // Who am I register
      payload[0] = I_AM_MCU;                // Who am I response
      payload[1] = who_am_i_cnt;            // Test counter
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      if (who_am_i_cnt < 0XFF)              // If test counter is less than 255
        who_am_i_cnt++;                     // Increment test counter
      else                                  // If test counter is 255
        who_am_i_cnt = 0x00;                // Reset test counter to 0
      break;

    case MCU_PWM_S:                         // Read Steering PWM channel
      if ((OUTSIDE(pwm_s_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_s_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_s_res = 0;                      // Return out of range value

      payload[0] = pwm_s_res >> 8;          // PWM Steering MSB
      payload[1] = pwm_s_res & 0xFF;        // PWM Steering LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_PWM_T:                         // Read Throttle PWM channel
      if ((OUTSIDE(pwm_t_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_t_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_t_res = 0;                      // Return out of range value

      payload[0] = pwm_t_res >> 8;          // PWM Throttle MSB
      payload[1] = pwm_t_res & 0xFF;        // PWM Throttle LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_PWM_3:                         // Read Aux 3 PWM channel
      if ((OUTSIDE(pwm_3_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_3_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_3_res = 0;                      // Return out of range value

      payload[0] = pwm_3_res >> 8;          // PWM Aux 3 MSB
      payload[1] = pwm_3_res & 0xFF;        // PWM Aux 3 LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_PWM_4:                         // Read Aux 4 PWM channel
      if ((OUTSIDE(pwm_4_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_4_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_4_res = 0;                      // Return out of range value

      payload[0] = pwm_4_res >> 8;          // PWM Aux 4 MSB
      payload[1] = pwm_4_res & 0xFF;        // PWM Aux 4 LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_PWM_ALL:                       // Read all PWM channel
      if ((OUTSIDE(pwm_s_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_s_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_s_res = 0;                      // Return out of range value

      if ((OUTSIDE(pwm_t_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_t_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_t_res = 0;                      // Return out of range value

      if ((OUTSIDE(pwm_3_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_3_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_3_res = 0;                      // Return out of range value

      if ((OUTSIDE(pwm_4_res, PWM_MIN_MS, PWM_MAX_MS)) || ((micros() - pwm_4_in) > PWM_TIMEOUT_US)) // Validate the timespan
        pwm_4_res = 0;                      // Return out of range value

      payload[0] = pwm_s_res >> 8;          // PWM Steering MSB
      payload[1] = pwm_s_res & 0xFF;        // PWM Steering LSB
      payload[2] = pwm_t_res >> 8;          // PWM Throttle MSB
      payload[3] = pwm_t_res & 0xFF;        // PWM Throttle LSB
      payload[4] = pwm_3_res >> 8;          // PWM Aux 3 MSB
      payload[5] = pwm_3_res & 0xFF;        // PWM Aux 3 LSB
      payload[6] = pwm_4_res >> 8;          // PWM Aux 4 MSB
      payload[7] = pwm_4_res & 0xFF;        // PWM Aux 4 LSB
      Wire.write((uint8_t *)&payload, 8);   // Transfer the payload
      break;

    case MCU_AN_1:                          // Read analog input channel 1
      payload[0] = an_1_res >> 8;           // A1 MSB
      payload[1] = an_1_res & 0xFF;         // A1 LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_AN_2:                          // Read analog input channel 2
      payload[0] = an_2_res >> 8;           // A2 MSB
      payload[1] = an_2_res & 0xFF;         // A2 LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_AN_3:                          // Read analog input channel 3
      payload[0] = an_3_res >> 8;           // A3 MSB
      payload[1] = an_3_res & 0xFF;         // A3 LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_AN_4:                          // Read analog input channel 4
      payload[0] = an_4_res >> 8;           // A4 MSB
      payload[1] = an_4_res & 0xFF;         // A4 LSB
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    case MCU_AN_ALL:                        // Read all analog input channels
      payload[0] = an_1_res >> 8;           // A1 MSB
      payload[1] = an_1_res & 0xFF;         // A1 LSB
      payload[2] = an_2_res >> 8;           // A2 MSB
      payload[3] = an_2_res & 0xFF;         // A2 LSB
      payload[4] = an_3_res >> 8;           // A3 MSB
      payload[5] = an_3_res & 0xFF;         // A3 LSB
      payload[6] = an_4_res >> 8;           // A4 MSB
      payload[7] = an_4_res & 0xFF;         // A4 LSB
      Wire.write((uint8_t *)&payload, 8);   // Transfer the payload
      break;

    case MCU_TACH_L:                        // Read tachometer L revolutions
      payload[0] = tach_l_res & 0xFF;       // Tach L revolutions
      Wire.write((uint8_t *)&payload, 1);   // Transfer the payload
      break;

    case MCU_TACH_R:                        // Read tachometer R revolutions
      payload[0] = tach_r_res & 0xFF;       // Tach R revolutions
      Wire.write((uint8_t *)&payload, 1);   // Transfer the payload
      break;

    case MCU_TACH_ALL:                      // Read both tachometer revolutions
      payload[0] = tach_l_res & 0xFF;       // Tach L revolutions
      payload[1] = tach_r_res & 0xFF;       // Tach R revolutions
      Wire.write((uint8_t *)&payload, 2);   // Transfer the payload
      break;

    default:
      break;
  }

  PCICR = 0x05;                             // Enable PC interrupts
}

// Timer 1 Interrupt at 1Hz
ISR(TIMER1_COMPA_vect) {                    // Timer1 Compare Match interrupt
  // Left revolutions
  tach_l_res = (uint16_t)tach_l_rev;        // Copy revolutions to the result register
  tach_l_rev = 0;                           // Reset the revolutions counter

  // Right revolutions
  tach_r_res = (uint16_t)tach_r_rev;        // Copy revolutions to the result register
  tach_r_rev = 0;                           // Reset the revolutions counter
}

// Sets up pin interrupts
void setup_pin_interrupts() {
  // Pin change interrupt control register
  PCICR = 0x05;                             // PCIE2 for PWM pins and PCIE0 for tach pins

  // Pin change mask register
  PCMSK2 = 0x1E;                            // Enable PWM S, T, 2, 3 interrupts
  PCMSK0 = 0x03;                            // Enable Tach A, B interrupts
}

void setup_timers() {
  noInterrupts();                           // Stop the interrupts

  // Set Timer 1 interrupt
  // Interrupt frequency: 1Hz
  // Prescaler:           1024
  // Compare match:       15624
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;                               // Counter
  OCR1A = 15624;                            // Compare match
  TCCR1B |= (1 << WGM12);                   // Turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);      // Prescaler
  TIMSK1 |= (1 << OCIE1A);                  // Enable timer compare interrupt

  interrupts();                             // Continue the interrupts
}

// Sets up I2C bus
void setup_i2c_slave() {
  Wire.begin(I2C_ADDRESS);                 // Initialize I2C as slace device
  Wire.onRequest(serial_request_event);    // Subscribe to I2C request event to send data
  Wire.onReceive(serial_receive_event);    // Subscribe to I2C receive event to receive data
}

// Reads analog inputs
void read_analog_inputs() {
  an_1_res = analogRead(AIN_1);            // Read analog input 1
  an_2_res = analogRead(AIN_2);            // Read analog input 2
  an_3_res = analogRead(AIN_3);            // Read analog input 3
  an_4_res = analogRead(AIN_4);            // Read analog input 4
}

// Initialization
void setup() {
  // Set-up pin modes
  pinMode(PE0,    INPUT);                  // Set to input for 328PB compatibility, do not set this pin as output
  pinMode(PE1,    INPUT);                  // Set to input for 328PB compatibility, do not set this pin as output
  pinMode(TACH_L, INPUT_PULLUP);           // Set PWM S pin to input with pull-up resistor enabled
  pinMode(TACH_R, INPUT_PULLUP);           // Set PWM S pin to input with pull-up resistor enabled
  pinMode(AIN_1,  INPUT);                  // Set analog channel 1 to input 
  pinMode(AIN_2,  INPUT);                  // Set analog channel 2 to input 
  pinMode(AIN_3,  INPUT);                  // Set analog channel 3 to input 
  pinMode(AIN_4,  INPUT);                  // Set analog channel 4 to input 
  pinMode(PWM_S,  INPUT_PULLUP);           // Set PWM S pin to input with pull-up resistor enabled
  pinMode(PWM_T,  INPUT_PULLUP);           // Set PWM T pin to input with pull-up resistor enabled
  pinMode(PWM_3,  INPUT_PULLUP);           // Set PWM 3 pin to input with pull-up resistor enabled
  pinMode(PWM_4,  INPUT_PULLUP);           // Set PWM 4 pin to input with pull-up resistor enabled

  // Set-up system clock prescaler
  CLKPR = bit(CLKPCE);                     // Enable clock prescaler bit change before modification
  CLKPR = 0x00;                            // Clock prescaler: 1

  // Set-up system
  wdt_enable(WDTO_1S);                     // Set watchdog timer to 1 seconds
  setup_i2c_slave();                       // Set-up I2C bus
  setup_timers();                          // Set-up timers
  setup_pin_interrupts();                  // Set-up pin interrupts
}

// Main program loop
void loop() {
  wdt_reset();                             // Reset watchdog timer
  read_analog_inputs();                    // Read analog inputs
  delay(1);                                // Wait 1 miliseconds
}
