#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

const float dt = 0.75; // miliseconds
const float Kp = 25.0;
const float Ki = 0.6;
const float Kd = 0.3;

float setpoint = 215;

void wait(int ms) {
  _delay_ms(ms);
}

/* 1Mohm pulldown
 *
 * 976 == 46k2 ohm
 * 938 == 92k2 ohm
 *
 * 100kOhm pulldown
 * 1014 == 1k
 * 700  == 46k2
 * 531  == 92k2
 * 972  == 5k36
 * 1012 == 1k12
 *
 * 10k pulldown
 *
 * 977 == 0k480
 * 912 == 1k12
 * 667 == 5k36
 * 182 == 46k2
 * 99  == 92k2
*/

#define T1 (273.15+25.0)
#define R1 100000.0
#define Beta 3974.0

#define BAUD 38400

#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   //clever runde
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     //reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD-1000)   //Fehler in Promille

#if ((BAUD_ERROR>10)||(BAUD_ERROR<-10))
  #error Systematischer Fehler in der Baudrate größer 1% und damit zu hoch!
#endif


int init_usart(void) {
  UBRRH = UBRR_VAL >> 8;
  UBRRL = UBRR_VAL & 0xFF;

  UCSRB = (1<<TXEN);          //UART TX einschalten
  UCSRC = (1<<URSEL)|(3<<UCSZ0);    //Asynchron 8N1
  UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}

void write(char *d) {
  while (*d != 0) {
    while (!(UCSRA & (1<<UDRE))) {};
    UDR = *d;
    d++;
  }
}

int adc_get(char channel) {
  while (ADCSRA &  (1<<ADSC));

  ADMUX &= 0xf0;
  ADMUX |= channel;

  ADCSRA  |= (1<<ADSC); // Start conversion
  while (ADCSRA &  (1<<ADSC));

  ADCSRA  |= (1<<ADSC); // Start conversion
  while (ADCSRA &  (1<<ADSC));

  // wait until conversion  completes; ADSC=0 means Complete
  return ADCW; //return ADC result
}

float measure(void) {
  float Vout = adc_get(0);
  const float Vin = 1023; // "Volts"
  const float pulldown = 10000.0; // ohms

  float R2 = pulldown * Vin / Vout - pulldown;

  float T2 = T1 * Beta / logf(R1 / R2) / (Beta / logf(R1 / R2) - T1);
  return T2 - 273.15;
}

void init_fastpwm() {
  DDRB |= (1 << DDB3);
  // PB3 is now an output
  OCR2 = 0xff;
  // set PWM 100% duty cycle
  TCCR2 |= (1 << COM21); //| (1 << COM20);
  // set noninverting mode
  TCCR2 |= (1 << WGM21) | (1 << WGM20);
  // set fast PWM Mode
  TCCR2 |= (1 << CS20);
  // set prescaler to 8 and starts PWM
}

void init_adc() {
  ADCSRA  = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
  // Set ADCSRA Register with division factor 32
}

void pwm_set(float x) {
  OCR2 = (int)x;
}

int main(void) {
  init_fastpwm();
  init_adc();
  init_usart();

  float previous_error = 0;
  float integral = 0;
  float time = 0;

  while(1) {
    float measured_value = measure();
//    setpoint = 240.0 * adc_get(1)/1023.0;
    setpoint = 160 + 80.0 * adc_get(1)/1023.0;
    char msg[64];

    float error = setpoint - measured_value;
    integral = integral + error*dt;
    float derivative = (error - previous_error)/dt;
    float output = Kp*error + Ki*integral + Kd*derivative;
    previous_error = error;
//    if (output > 250) output = 255;
//    if (output <= 5) output = 0;
    int oout = output;
    if (output > 127) output = output * 1.5;
    if (output > 255) output = 255;
    if (output < 127) output *= 2;
    if (output <= 0) output = 0;

    sprintf(msg, "t=%3d/%3d - o=%3d->%3d e=%3d d=%3d i=%3d | %d.%ds\n\r",
        (int)measured_value, (int)setpoint, oout, (int)output, (int)(error*Kp),
        (int)(derivative*Kd), (int)(integral*Ki),
        (int)time, (int)(1000.0*time-(float)(1000*floor(time))));
    write(msg);

    pwm_set(output);
    wait(dt*1000);
    time += dt;
  };

  return 0;
}
