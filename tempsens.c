/* Read a temperature sensor
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2008, 2010 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* Original software modified by Steve Roggenkamp to read a temperature sensor
   and change the color of a tricolor LED depending on the temperature.
   It also prints out the temperature if read by the USB 
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usb_debug_only.h"
#include "print.h"

// Green LED on PIN_B6
// Red LED   on PIN_B5
// Blue LED  on PIN_B4

#define GREEN_LED   (PIN_B6)
#define RED_LED     (PIN_B5)
#define BLUE_LED    (PIN_B4)

#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))

#define ADC_PRESCALER   (1<<2)

// Teensy 2.0: LED is active high
#if defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB1286__)
#define GREEN_ON	(PORTB |= (1<<6))
#define GREEN_OFF	(PORTB &= ~(1<<6))
#define RED_ON		(PORTB |= (1<<5))
#define RED_OFF		(PORTB &= ~(1<<5))
#define BLUE_ON		(PORTB |= (1<<4))
#define BLUE_OFF	(PORTB &= ~(1<<4))

// Teensy 1.0: LED is active low
#else
#define LED_ON	(PORTD &= ~(1<<6))
#define LED_OFF	(PORTD |= (1<<6))
#endif

#define LED_CONFIG	(DDRD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define DIT 80		/* unit time for morse code */

static uint8_t aref = (1<<REFS0); // default to AREF = Vcc

/* The following arrays assume a 10,000 ohm resistor in parallel
   with the thermistor.
*/
typedef struct {
  int8_t tempC;
  uint16_t code;
} codeTemp;


/* code2temp provides a table lookup to convert from
 * the analog code read from the pin to a
 * temperature.
 */
static codeTemp code2temp[] = {
  { -40,   30 },
  { -35,   41 },
  { -30,   55 },
  { -25,   73 },
  { -20,   96 },
  { -15,  124 },
  { -10,  157 },
  {  -5,  196 },
  {   0,  240 },
  {   5,  289 },
  {  10,  342 },
  {  15,  398 },
  {  20,  455 },
  {  25,  512 },
  {  30,  566 },
  {  35,  619 },
  {  40,  667 },
  {  45,  712 },
  {  50,  752 },
  {  55,  788 },
  {  60,  819 },
  {  65,  847 },
  {  70,  870 },
  {  75,  891 },
  {  80,  909 },
  {  85,  924 },
  {  90,  937 },
  {  95,  948 },
  { 100,  958 },
  { 105,  966 },
  { 110,  974 },
  { 115,  980 },
  { 120,  985 },
  { 125,  989 },
  { 130,  993 },
  { 135,  997 },
  { 140, 1000 },
  { 145, 1002 },
  { 150, 1005 }
};

// Mux input
int16_t adc_read(uint8_t mux)
{
#if defined(__AVR_AT90USB162__)
	return 0;
#else
	uint8_t low;

	ADCSRA = (1<<ADEN)  | ADC_PRESCALER;		// enable ADC
	ADCSRB =  (mux & 0x20);		// high speed mode
	ADMUX  = aref | (mux & 0x1F);			// configure mux input
	ADMUX  &= ~(1<<ADLAR);                          // right adjust register
	ADCSRA = (1<<ADEN)  | ADC_PRESCALER | (1<<ADSC);// start the conversion
	while (ADCSRA & (1<<ADSC)) ;			// wait for result
	low = ADCL;					// must read LSB first
	return (ADCH << 8) | low;			// must read MSB only once!
#endif
}


// Arduino compatible pin input
int16_t analogRead(uint8_t pin)
{
#if defined(__AVR_ATmega32U4__)
	static const uint8_t PROGMEM pin_to_mux[] = {
		0x00, 0x01, 0x04, 0x05, 0x06, 0x07,
		0x25, 0x24, 0x23, 0x22, 0x21, 0x20};
	if (pin >= 12) return 0;
	return adc_read(pgm_read_byte(pin_to_mux + pin));
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
	if (pin >= 8) return 0;
	return adc_read(pin);
#else
	return 0;
#endif
}


/* codetemp computes the temperature in C based on a lookup of the
 * code2temp table.  It then interpolates between the values in the
 * table to compute the temperature to the closest degree.
 *
 * returns temperature * 10, so it return 10 degrees as 100
 * this provides a fixed point value
 */
int16_t codetemp( int16_t code )
{
  unsigned char i;
  int16_t rc;

  // just do a linear lookup
  for ( i = 0; i < sizeof(code2temp) / sizeof( code2temp[0]) - 1; i++ ) {
    if (  code2temp[i].code   <  code
       && code2temp[i+1].code >= code ) {
      break;
    }
  }
  // interpolate to the nearest degree
  rc = code2temp[i].tempC * 10 + 50 * (code - code2temp[i].code)/(code2temp[i+1].code - code2temp[i].code);

  return rc;
}

// print out a digit.  nothing like low-level code here.
void printDigit( int16_t val )
{
  switch ( val ) {
  case 0: print( "0" ); break;
  case 1: print( "1" ); break;
  case 2: print( "2" ); break;
  case 3: print( "3" ); break;
  case 4: print( "4" ); break;
  case 5: print( "5" ); break;
  case 6: print( "6" ); break;
  case 7: print( "7" ); break;
  case 8: print( "8" ); break;
  case 9: print( "9" ); break;
  default: print( "0" ); break;
  }
}


// set everything up, then loop forever...
int main(void)
{
  int16_t       code;
  int16_t       celsius;
  static char          cout[2];

  cout[0] = 0;
  cout[1] = 0;

  // set for 16 MHz clock, and make sure the LED is off
  CPU_PRESCALE(0);
  LED_CONFIG;
  LED_OFF;
  
  GREEN_OFF;
  RED_OFF;
  BLUE_OFF;

  usb_init();

  // initialize the USB, but don't want for the host to
  // configure.  The first several messages sent will be
  // lost because the PC hasn't configured the USB yet,
  // but we care more about blinking than debug messages!
  //  usb_init();

  while (1) {
    
    code = analogRead(1);
      
    celsius = codetemp( code );

    print( "Temp: " );
    code = celsius;
    if ( code <    0 ) { print( "-" ); code *= -1; }
    if ( code >= 1000 ) { print( "1" ); code -= 100; }
    printDigit( code/100 );
    printDigit( (code%100)/10 );
    print(".");
    printDigit( code % 10 );
    print( "\n" );

    // now turn on the correct LED color, first
    // making sure all of them have been turned off
    BLUE_OFF;
    RED_OFF;
    GREEN_OFF;
    if ( celsius < 250 ) {  // 25.0 C
      BLUE_ON;
    } else if ( celsius < 300 ) {  // 30.0 C
      GREEN_ON;
    } else {
      RED_ON;
    }
    _delay_ms(5000);
  }
}

