#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

const float dt = 0.75; // miliseconds
const float Kp = 10.0; //25.0;
const float Ki = 0.3;
const float Kd = 0.7;

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

//int _old_adc_value;

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
/*
float fxlog(float x_) {
  int x;

  // convert to fixed point, x \in (-32768, 32767)
  x  = ((int)x_) << 16;
  x += (x_ - (float)x) * (1 << 16);

  int t,y;

  y=0xa65af;
  if(x<0x00008000) x<<=16,              y-=0xb1721;
  if(x<0x00800000) x<<= 8,              y-=0x58b91;
  if(x<0x08000000) x<<= 4,              y-=0x2c5c8;
  if(x<0x20000000) x<<= 2,              y-=0x162e4;
  if(x<0x40000000) x<<= 1,              y-=0x0b172;
  t=x+(x>>1); if((t&0x80000000)==0) x=t,y-=0x067cd;
  t=x+(x>>2); if((t&0x80000000)==0) x=t,y-=0x03920;
  t=x+(x>>3); if((t&0x80000000)==0) x=t,y-=0x01e27;
  t=x+(x>>4); if((t&0x80000000)==0) x=t,y-=0x00f85;
  t=x+(x>>5); if((t&0x80000000)==0) x=t,y-=0x007e1;
  t=x+(x>>6); if((t&0x80000000)==0) x=t,y-=0x003f8;
  t=x+(x>>7); if((t&0x80000000)==0) x=t,y-=0x001fe;
  x=0x80000000-x;
  y-=x>>15;
  return y;
}*/

int measure(void) {
  static int bf[8], bfpos = 0;
  bf[bfpos++] = adc_get(0);
  if (bfpos>=8) bfpos = 0;
  int i;
  float Vout = 0;
  for (i = 0; i < 8; i++)
    Vout += bf[i];
  Vout /= 8;
  char x[8];
  int kokot = ADCW;
  sprintf(x, "[%d]\n\r", (int)Vout);
  write(x);
//  float Vout = adc_get(0);
//  Vout = Vout - (Vout - _old_adc_value)/2;
//  _old_adc_value = Vout;
  const float Vin = 1023; // "Volts"
  const float pulldown = 10000; // ohms

  float R2 = pulldown * Vin / Vout - pulldown;

  float T2 = T1 * Beta / logf((float)R1 / R2) / (Beta / logf((float)R1 / R2) - T1);
  return T2 - 273;
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

void pwm_set(int x) {
  OCR2 = (int)x;
}

int main(void) {
  init_fastpwm();
  init_adc();
  init_usart();

  int previous_error = 0;
  float integral = 0;
  float time = 0;

  while(1) {
    int measured_value = measure();
//    setpoint = 240.0 * adc_get(1)/1023.0;
    setpoint = 160 + 80.0 * adc_get(1)/1023.0;
    char msg[64];

    int error = setpoint - measured_value;
    integral = integral + error*dt;
    int derivative = (float)(error - previous_error)/dt;
    int output = Kp*error + Ki*integral + Kd*derivative;
    previous_error = error;
//    if (output > 250) output = 255;
//    if (output <= 5) output = 0;
    int oout = output;
    if (output > 127) output = 39 + output * 0.7;
    if (output > 255) output = 255;
    if (output < 127) output *= 1.5;
    if (output <= 0) output = 0;

    sprintf(msg, "t=%3d/%3d - o=%3d->%3d e=%3d d=%3d i=%3d | %d.%ds\n\r",
        (int)measured_value, (int)setpoint, /*_old_adc_value,*/ oout, (int)output, (int)(error*Kp),
        (int)(derivative*Kd), (int)(integral*Ki),
        (int)time, (int)(1000.0*time-(float)(1000*floor(time))));
    write(msg);

    pwm_set(output);
    wait(dt*1000);
    time += dt;
  };

  return 0;
}
