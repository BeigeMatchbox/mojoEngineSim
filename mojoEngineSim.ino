/*
      This code was quick and dirty, based on a PCM audio example in the
      arduino playground: http://playground.arduino.cc/Code/PCMAudio
      
      It's been heavely modified for use with RC to generate something that's
      a bit like an engine sound. With some rejigging this program could
      be far nicer to read, but in it's current state it is functional...
*/




#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "pins_arduino.h"
#include "idle.h"

#define BASE_RATE 16000 // The base sample rate of the audio
volatile uint16_t currentSmpleRate = BASE_RATE;

// Pins
#define POT_PIN A1        // Pot wiper when using pot mode
const int speakerPin = 3; // This is kept as 3, original code had 11 as option, but this conflicts with SPI
#define POT_CS  4         // If using a digi pot to control volume these are the pins
#define POT_SCK 5
#define POT_SDO 6

// Stuff not to play with!, so many globals...
boolean audioRunning = false;
int curVolume = 0;
volatile uint16_t curEngineSample;
volatile uint16_t curBrakeSample;
volatile uint8_t soundDivider = 1;
uint8_t lastSample;
int16_t  currentThrottle = 0;
uint8_t  throttleByte = 0;
uint8_t  spiReturnByte = 0;
volatile int pulseWidth = 0;



// Stuf to play with
#define DEFAULT_VOLUME 127 // Volume when in non managed mode

#define VOL_MIN 20         // Min volume in managed mode 0 - 127
#define VOL_MAX 127        // Max volume in managed mode 0 - 127

#define TOP_SPEED_MULTIPLIER 15 // RPM multiplier in managed mode, bigger the number the larger the rev range, 10 - 15 is a good place to start

// Mode settings - These could easily be 4 jumpers connected to spare pins, checked at startup to determin mode

boolean managedThrottle = true;     // Managed mode looks after the digipot if fitted for volume, and adds some mass to the engine
boolean potThrottle = true;         // A pot connected to A1, 0-1023 sets speed
boolean pwmThrottle = false;        // Takes a standard servo signal on pin 2 (UNO)
boolean spiThrottle = false;        // SPI mode, is an SPI slave, expects 1-255 for throttle position, with 0 being engine off





void setup()
{
  // Serial
  Serial.begin(57600);
  
  // SPI slave mode
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);// turn on SPI in slave mode
  SPCR |= _BV(SPIE); // turn on interrupts
  
  pinMode(POT_CS, OUTPUT);
  pinMode(POT_SCK, OUTPUT);
  pinMode(POT_SDO, OUTPUT);
  digitalWrite(POT_CS, HIGH);
  digitalWrite(POT_SCK, HIGH);
  digitalWrite(POT_SDO, HIGH);
  
  if(managedThrottle) writePot(0);
  else writePot(DEFAULT_VOLUME);
  
  // Analog input, we set these pins so a pot with 0.1in pin spacing can
  // directly into the arduino header, if you change POT_PIN you may want
  // to comment them out
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A2, LOW);
  
  
  // pwm in setup, for a standard servo pulse
  pinMode(2, INPUT); // We don't want INPUT_PULLUP as the 5v may damage some recevers!
  if(pwmThrottle){
    attachInterrupt(0, getPulsewidth, CHANGE);
  }
  
  // setup complete, so start making sounds
  startPlayback();
}




void loop()
{
      
  if(managedThrottle){
    if     (potThrottle) currentThrottle = analogRead(POT_PIN);
    else if(spiThrottle) {
      if(throttleByte > 0){
        if(!audioRunning) startPlayback();
        currentThrottle = throttleByte << 2;
      }
      else if(audioRunning) stopPlayback();
    }
    else if(pwmThrottle){
      if(pulseWidth > 800 && pulseWidth < 2200){ // check if the pulsewidth looks like a servo pulse
        if(pulseWidth < 1000) pulseWidth = 1000; // Constrain the value
        if(pulseWidth > 2000) pulseWidth = 2000;
        
        if(pulseWidth > 1520) currentThrottle = (pulseWidth - 1500) *2;  // make a throttle value from the pulsewidth 0 - 1000
        else if(pulseWidth < 1470) currentThrottle = abs( (pulseWidth - 1500) *2);
        else currentThrottle = 0;
      }
    }
    manageSpeed();
  }
  
  else {
    if     (potThrottle) currentSmpleRate = F_CPU / (BASE_RATE + long(analogRead(POT_PIN) * TOP_SPEED_MULTIPLIER));    
    else if(spiThrottle) {
      if(throttleByte > 0){
        if(!audioRunning) startPlayback();
        currentSmpleRate = F_CPU / (BASE_RATE + long((throttleByte << 2) * TOP_SPEED_MULTIPLIER));
      }
      else if(audioRunning) stopPlayback();
    }
    else if(pwmThrottle){
      if(pulseWidth > 800 && pulseWidth < 2200){ // check if the pulsewidth looks like a servo pulse
        if(pulseWidth < 1000) pulseWidth = 1000; // Constrain the value
        if(pulseWidth > 2000) pulseWidth = 2000;
        
        if(pulseWidth > 1520) currentThrottle = (pulseWidth - 1500) *2;  // make a throttle value from the pulsewidth 0 - 1000
        else if(pulseWidth < 1470) currentThrottle = abs( (pulseWidth - 1500) *2);
        else currentThrottle = 0;
        currentSmpleRate = F_CPU / (BASE_RATE + long(currentThrottle * TOP_SPEED_MULTIPLIER));
      }
    }
  }
  
}




void manageSpeed(){
  
  //int currentThrottle = 0;
  static int prevThrottle = 0xFFFF;
  static int currentRpm = 0;
  const int maxRpm = 8184;
  const int minRpm = 0;

  
  static unsigned long throtMillis;
  
  if(millis() - throtMillis > 5) {
    throtMillis = millis();  
    
    if(currentThrottle +12 > currentRpm){
      currentRpm += 6;
      if(currentRpm > maxRpm) currentRpm = maxRpm;
      prevThrottle = currentThrottle;
      
    }
    else if(currentThrottle -15 < currentRpm){
      currentRpm -= 12;
      if(currentRpm < minRpm) currentRpm = minRpm;
      prevThrottle = currentThrottle;
    }
    
    if(currentRpm >> 2 < 255) spiReturnByte = currentRpm >> 2;
    else spiReturnByte = 255;
    if(currentRpm >> 2 < 0) spiReturnByte = 0;
    //Serial.println(spiReturnByte, DEC);
    
    currentSmpleRate = F_CPU / (BASE_RATE + long(currentRpm * TOP_SPEED_MULTIPLIER) );
  }
  
  
  static unsigned long volMillis;
  if(millis() - volMillis > 50) {
    volMillis = millis();  
    
    int vol = map(currentThrottle, 0, 1023, VOL_MIN, VOL_MAX);
    
    if(vol > curVolume) curVolume = vol;
    else {
      curVolume -= (curVolume/10);  //7;
      if(curVolume < VOL_MIN) curVolume = VOL_MIN;
    }
    
    int lastVolume = 0xFFFF;
    if(curVolume != lastVolume){
      lastVolume = curVolume;
      writePot(curVolume);
    }
    
    
  }
  
}




void writePot(byte data){
  
  if(data > VOL_MAX) data = VOL_MAX; // cap it just in case
  
  digitalWrite(POT_CS, LOW);
  shiftOut(POT_SDO, POT_SCK, MSBFIRST, 0x00);
  shiftOut(POT_SDO, POT_SCK, MSBFIRST, data);
  digitalWrite(POT_CS, HIGH);
  
}









// Uses a pin change interrupt and micros() to get the pulsewidth at pin 2
void getPulsewidth(){
  unsigned long currentMicros = micros();
  boolean currentState = digitalRead(2);
  
  static unsigned long prevMicros = 0;
  static boolean lastState = LOW;
  
  if(lastState == LOW && currentState == HIGH){ // Rising edge
    prevMicros = currentMicros;
    lastState = currentState;
  }
  else if(lastState == HIGH && currentState == LOW){ // Falling edge
    pulseWidth = currentMicros - prevMicros;
    lastState = currentState;
  }
  
}

// SPI slave interrupt, just stores the last byte and sends
// current throttle when in managed mode
// If we change to a multibyte system this will get expanded
ISR (SPI_STC_vect){
  throttleByte = SPDR;
  SPDR = spiReturnByte;
}








// PCM stuff
void startPlayback()
{
  pinMode(speakerPin, OUTPUT);
  audioRunning = true;
  // Set up Timer 2 to do pulse width modulation on the speaker
  // pin.

  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);

  // Do non-inverting PWM on pin OC2B (p.155)
  // On the Arduino this is pin 3.
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set initial pulse width to the first sample.
  OCR2B = pgm_read_byte(&idle_data[0]);



  // Set up Timer 1 to send a sample every interrupt.

  cli();

  // Set CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / BASE_RATE;    // 16e6 / 8000 = 2000

  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);

  lastSample = pgm_read_byte(&idle_data[idle_len-1]);
  curEngineSample = 0;
  sei();
  
  // Fadein the pot
  byte target = map(currentThrottle, 0, 1023, VOL_MIN, VOL_MAX);
  for(byte i = 0; i < target; i ++){
    curVolume = i;
    writePot(curVolume);
    delay(1);
  }
}


void stopPlayback()
{
  // Fadeout the pot
  for(byte i = curVolume; i > 0; i--){
    curVolume = i;
    writePot(i);
    delay(1);
  }
  
  audioRunning = false;
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);

  // Disable the per-sample timer completely.
  TCCR1B &= ~_BV(CS10);

  // Disable the PWM timer.
  TCCR2B &= ~_BV(CS10);

  digitalWrite(speakerPin, LOW);
}


// This is the main playback interupt, keep this nice and tight!!
ISR(TIMER1_COMPA_vect) {  
  OCR1A = currentSmpleRate;
  
  if (curEngineSample >= idle_len) {
    curEngineSample = 0;
  }
  
  OCR2B = pgm_read_byte(&idle_data[curEngineSample]);           

  ++curEngineSample;
  
}
