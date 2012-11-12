#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

const int dt = 25; // miliseconds
const float Kp = 0.3;
const float Ki = 0.3;
const float Kd = 0.3;

const int setpoint = 160;

void wait(int ms) {
  _delay_ms(ms);
}

#define T1 25.0
#define R1 100000.0
#define Beta 3974.0

int adc_get() {
  ADCSRA  |= (1<<ADSC); // Start conversion
  while (ADCSRA &  (1<<ADSC));
  // wait until conversion  completes; ADSC=0 means Complete
  return ADCW; //return ADC result
}

float measure(void) {
  float R2 = 1600; // to be measured
  float T2 = T1 * Beta / log(R1 / R2) / (Beta / log(R1 / R2) - T1);
  return T2;
}

void init_fastpwm() {
  DDRB |= (1 << DDB3);
  // PB3 is now an output
  OCR2 = 0xff;
  // set PWM 100% duty cycle
  TCCR2 |= (1 << COM21) | (1 << COM20);
  // set inverting mode
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

  float previous_error = 0;
  float integral = 0;

  while(1) {
    float measured_value = measure();

    float error = setpoint - measured_value;
    integral = integral + error*dt;
    float derivative = (error - previous_error)/dt;
    float output = Kp*error + Ki*integral + Kd*derivative;
    previous_error = error;
    pwm_set(output);
    wait(dt);
  };

  return 0;
}
