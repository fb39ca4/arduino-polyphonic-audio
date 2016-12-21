#include "avr/io.h"
#include <stdint.h>

#include "oscillator.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#if defined(ARDUINO_AVR_MEGA2560)
#define OUTPUT_SPEAKER_A 9
#define OUTPUT_SPEAKER_B 10
#elif defined(ARDUINO_AVR_UNO)
#define OUTPUT_SPEAKER_A 5
#define OUTPUT_SPEAKER_B 6
#endif

#define OUTPUT_LIGHT 12

void setup() {
  // put your setup code here, to run once:
  noInterrupts();
  
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
  
  pinMode(OUTPUT_SPEAKER_A, OUTPUT);
  pinMode(OUTPUT_SPEAKER_B, OUTPUT);
  pinMode(OUTPUT_LIGHT, OUTPUT);
  
/*
#if defined(ARDUINO_AVR_MEGA2560)
  TCCR3A = _BV(COM3A1)|_BV(COM3B0)|_BV(COM3B1)|_BV(COM3C1)|_BV(WGM30);
  TCCR3B = _BV(WGM32)|_BV(CS30)|_BV(CS30);
  TIMSK3 = _BV(TOIE3);
  OCR3A = 127;
  OCR3B = 127;
  OCR3C = 127;
#elif defined(ARDUINO_AVR_UNO)
  TCCR1A = _BV(COM1A1)|_BV(COM1B0)|_BV(COM1B1)|_BV(WGM10);
  TCCR1B = _BV(WGM12)|_BV(CS10)|_BV(CS10);
  TIMSK1 = _BV(TOIE1);
  OCR1A = 127;
  OCR1B = 127;
#endif

  TCCR2A = 0;// _BV(WGM20) | _BV(WGM21);
  TCCR2B = _BV(CS20);//|_BV(WGM22);
  TIMSK2 = _BV(TOIE2);*/


  TIMSK0 = 0;
  TIMSK2 = 0;

#if defined(ARDUINO_AVR_MEGA2560)  
  TCCR2A = _BV(COM2A1)|_BV(COM2B0)|_BV(COM2B1)|_BV(WGM20)|_BV(WGM21);
  TCCR2B = _BV(CS20)|_BV(CS20);
  OCR2A = 127;
  OCR2B = 127;
#elif defined(ARDUINO_AVR_UNO)
  TCCR0A = _BV(COM0A1)|_BV(COM0B0)|_BV(COM0B1)|_BV(WGM00)|_BV(WGM01);
  TCCR0B = _BV(CS00)|_BV(CS00);
  OCR0A = 127;
  OCR0B = 127;
#endif
  
  TCCR1A = 0;
  TCCR1B = _BV(WGM12)|_BV(CS10)|_BV(CS10);
  TIMSK1 = _BV(OCIE1A);
  OCR1A = 511;
  TCNT1 = TCNT2;
  
  Serial.begin(115200);
  interrupts();
}


#define fr(freq) ((int16_t)(65536.*(freq)/31250))
oscillator osc0 = {0, fr(440), 31};
oscillator osc1 = {0, fr(659.25*2), 31};
oscillator osc2 = {0, fr(554.37*2), 31};
oscillator osc3 = {0, 1, 31};


inline void setOutput(unsigned char level) {
#if defined(ARDUINO_AVR_MEGA2560)
  OCR2A = level;
  OCR2B = level;
#elif defined(ARDUINO_AVR_UNO)
  OCR0A = level;
  OCR0B = level;
#endif
}

ISR(TIMER1_COMPA_vect)
{
  noInterrupts();
  static int val;
  setOutput(val);
  /*osc1 += F(440*2);
  osc2 += F(659.25*2);
  osc3 += F(554.37*2);*/
  val = 127;
  /*val += squareWaveMacro(osc0);
  val += squareWaveMacro(osc1);
  val += squareWaveMacro(osc2);*/
  val += squareWave(&osc0);
  val += squareWave(&osc1);
  val += squareWave(&osc2);
  val += squareNoise(&osc3);
  setOutput(val);
  interrupts();

}
int ledState = 0;
void readInput() {
  if (Serial.available() <= 0) return;
  int input = Serial.read();
  //Serial.write('i');
  //Serial.write(input);
  if (input == 0x61) {
    digitalWrite(OUTPUT_LIGHT, ledState ^= 1);
    return;
  }
  uint8_t channel = input & 0xF;
  oscillator *osc = NULL;
  switch (channel) {
    case 0:
      osc = &osc0;
      break;
    case 1:
      osc = &osc1;
      break;
    case 2:
      osc = &osc2;
      break;
    case 3:
      osc = &osc3;
      break;
    default:
      return;
      break;
  }
  uint8_t command = (input >> 4) & 0xF;
  uint8_t highByte;
  uint8_t lowByte;
  switch (command) {
    case 0:
      while (Serial.available() <= 0) continue;
      highByte = Serial.read();
      //Serial.write('h');
      //Serial.write(highByte);
      while (Serial.available() <= 0) continue;
      lowByte = Serial.read();
      //Serial.write('l');
      //Serial.write(lowByte);
      osc->stride = (highByte << 8) | lowByte;
      break;
    default:
      return;
      break;
  }
}

void loop() {
  readInput();
  digitalWrite(OUTPUT_LIGHT, osc3.stride);
}
