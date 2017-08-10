/*
 * DigitalLowPassFilter.c
 * This code is not optimized because I used float numbers in sprintf functions
 * And I used this function in ADC interrupt for exaple
 * Created: 10.08.2017 16:38:56
 * Author : Mazinov
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "lcd_lib.h"

#define SPS 9600UL // SPS = Fadc/n = 125000Hz/13tic = 9600 Hz
#define T 0.001f
#define K (SPS*T)  // K = 9600 * 0.001c = 10
#define BEGIN_CONVERT ADCSRA|=(1<<ADSC)

typedef unsigned char byte;

uint16_t LowPassFilter(void);
void print_result(void);
void print_voltage(void);
void mcu_init(void);

volatile uint16_t adc_data;

ISR( TIMER0_OVF_vect )
{
	// 100 us oveflow
	TCNT0=0x9C;
	BEGIN_CONVERT;
}


ISR (ADC_vect)
{
	// Read the AD conversion result
	adc_data = ADC;
}

int main(void)
{
	mcu_init();
	lcd_init();
    sei();

    while (1) 
    {
		print_result();
		print_voltage();
    }
	return 0;
}

uint16_t LowPassFilter(void)
{
	static uint16_t temp = 0;
	static uint16_t adc_out = 0;
	uint16_t input_data = adc_data;

	temp = temp + input_data - adc_out;
	adc_out = temp / (uint16_t)K;

	return adc_out;
}

void print_result(void)
{
	lcd_gotoxy(0,0);
	lcd_num_to_str(adc_data, 4);
	lcd_gotoxy(0,1);
	lcd_num_to_str(LowPassFilter(), 4);
}

void print_voltage(void)
{
	float voltage;
	byte temp[4];

	voltage = (float)adc_data * 0.0048828; // (float)* data*5.00/1024.00;
	lcd_gotoxy(0,0);
	lcd_num_to_str(voltage, 4);
	lcd_gotoxy(0,1);
	lcd_num_to_str(LowPassFilter(), 4);
	lcd_gotoxy(7,1);
	sprintf(temp, "%f", voltage);
	lcd_string(temp, 4);
}

void mcu_init(void)
{
	// Oscillator devider /1
	CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);

	// Timer/Counter 0 initialization
	// Clock source: System Clock 8MHz
	// Mode: Normal top=0xFF
	// Timer Period: 0,1 ms
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
	TCNT0=0x9C;

	// Timer/Counter 0 Interrupt initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);

	
	// ADC initialization
	// ADC Clock frequency: 125,000 kHz
	// ADC Voltage Reference: AREF pin
	// ADC Auto Trigger Source: Timer0 Overflow
	// ADC4: On, ADC5: On
	DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
	ADMUX=(0<<REFS1) | (0<<REFS0) | (0<<ADLAR) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) |(0<<MUX0);
	ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
	ADCSRB=(1<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
}