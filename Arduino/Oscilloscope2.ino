
int baudDivider = 1;
int baudCode = 0;
byte buf[3];
unsigned long us;
byte analogPin = 3;

void setup() {
  // put your setup code here, to run once:
  //Determine what the buad rate divider will be
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  if (digitalRead(8) == LOW) baudCode += 1;
  if (digitalRead(9) == LOW) baudCode += 2;
  if (digitalRead(10) == LOW) baudCode += 4;
  for (int i = 0; i < baudCode; i++) baudDivider *= 2;
  
  Serial.begin(460800 / baudDivider);
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  ADCSRA |= bit (ADPS2);                               //  set adc clock prescale to 16
  //Set ADMUX to point at analogPin
  ADMUX &= ~(bit(MUX0) | bit(MUX1) | bit(MUX2)); //clear the MUX bits
  ADMUX |= analogPin;  //Set MUX bits
  //Use Vcc as ref voltage
  ADMUX |= ( ADMUX & ~(bit(REFS1) | bit(REFS0)) ) | (1<<6);
  //Left shift ADC results
  ADMUX |= bit(ADLAR);

  
  //Set up Timer Counter 1 as test input

  TCCR1A = bit(COM1A0) | bit(WGM11) | bit(WGM10);
  TCCR1B = bit(WGM13) | bit(WGM12) | bit(CS11) | bit(CS10);
  OCR1A = 24;
  pinMode(9, OUTPUT);

  //Start obtaining data; don't use the arduino loop function.
  while( true )
  {
    //Start conversion
    ADCSRA |= bit(ADSC);

    //In the meantime, get the time and start filling buffer
    us = micros();

    //Place current microseconds into buffer
    buf[0] = us & 0xFF;
    buf[1] = (us >> 8) & 0x3F;

    //Wait for conversion to complete
    while ( ADCSRA & bit(ADSC) );

    //Get the data
    buf[1] |= ( ADCL & ~(0x3F) );
    buf[2] = ADCH;

    //Send the data
    Serial.print('b');
    Serial.write(buf, 3);
  
  }
}

void loop() {
}
