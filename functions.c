/*
 * Program for the real-time part of the raspibot which is intended to run on
 * the ATtiny2313. The datasheet is available under
 * http://www.atmel.com/Images/doc2543.pdf
 *
 * @author Leander Herr <leander.herr@2011.tu-chemnitz.de>
 * @author Marcus Schlutter <marcus.homunculus@gmail.com>
 */
/******************************************************************************
 * Functions
 */

/**
 * Allows to set the speed of the motors which will result in a robot movement
 * which is either as long till the speed is set to zero again or if target
 * count (via setTargetCount()) is set till this value is reached.
 * Hint: the received value is multiplied with 2 in order to reach almost
 * full speed.
 *
 * @param speedRight the intended speed in the range from -127 to 127;
 *                   a negative value will result in a movement backwards
 * @param speedLeft the intended speed in the range from -127 to 127;
 *                  a negative value will result in a movement backwards
 */
void setMotorSpeed_r(int8_t speedRight){
#ifndef Attiny1
  // set the phase pin according to the intended direction of turn
  if (speedRight < 0) {
    // unset the Pin for the right wheel
    PORT_DIR_R &= ~(PIN_DIR_R);
    speedRight=-speedRight;
  }
  else {
    // set the Pin for the right wheel
    PORT_DIR_R |= PIN_DIR_R;
  }
  // and create the amount of the speed value -> direction is given by Phase
  // Pin
  OCR_R = ((uint8_t)(speedRight))<<1;
  // now wait for the robot to reach its target with one wheel if target is set 
#endif
}
void setMotorSpeed_l(int8_t speedLeft){
#ifndef Attiny1
  // now check for the left wheel
  if (speedLeft < 0) {
    PORT_DIR_L &= ~(PIN_DIR_L);
    speedLeft=-speedLeft;
  }
  else {
    PORT_DIR_L |= PIN_DIR_L;
  }
  // and create the amount of the speed value -> direction is given by Phase
  // Pin
  OCR_L = ((uint8_t)(speedLeft))<<1;
  // now wait for the robot to reach its target with one wheel if target is set
#endif
}

void setPIvalues(int16_t P, int16_t I, uint8_t encoderMulti)
{
	eeprom_update_word(&PID_P,P);
	eeprom_update_word(&PID_I,I);
	eeprom_update_byte(&EncoderMulti,encoderMulti);
}

void Send_Byte(uint8_t send){
 
	#ifndef Attiny2
    /* Wait for empty transmit buffer */
 	  while ( !( UCSRA & (1<<UDRE)) );
 	
 	  /* Put data into buffer, sends the data */
 	  UDR = send;
	#endif
}
void Send_Int(int16_t send){
#ifndef Attiny2
 	Send_Byte((send&0xFF00)>>8);
 	Send_Byte((send&0x00FF)   );
#endif
}
/*Send Attiny 2: für belibige Mengen Attinys erweiterbar*/
void Send_Byte_Attiny2(uint8_t send){

	#ifdef Attiny1
	  //DDR_Attiny1_to_2|= PIN_NR_Attiny1_to_2;
	  //DDR_Attiny2_to_1&=~PIN_NR_Attiny2_to_1;
    /* Wait for empty transmit buffer => Transmit of Attiny1 terminated befor Attiny2 start*/
 	  while ( !( UCSRA & (1<<UDRE)) );
	  PORT_Attiny1_to_2|=PIN_NR_Attiny1_to_2;

    /* Wait for Attiny2 transmit end*/
 	  while ( !(PIN_Attiny2_to_1&PIN_NR_Attiny2_to_1));
	  PORT_Attiny1_to_2&=~PIN_NR_Attiny1_to_2;
	#endif
	#ifdef Attiny2
	  //DDR_Attiny2_to_1|= PIN_NR_Attiny2_to_1;
	  //DDR_Attiny1_to_2&=~PIN_NR_Attiny1_to_2;
    /* Wait for Attiny1 transmit end*/
 	  while ( !(PIN_Attiny1_to_2&PIN_NR_Attiny1_to_2));

    /* Wait for empty transmit buffer */
 	  while ( !( UCSRA & (1<<UDRE)) );
 	
 	  /* Put data into buffer, sends the data */
 	  UDR = send;

    /* Wait for empty transmit buffer => Transmit of Attiny2 terminated*/
 	  while ( !( UCSRA & (1<<UDRE)) );
	  PORT_Attiny2_to_1|=PIN_NR_Attiny2_to_1;

    /* Wait for Attiny1*/
 	  while ( (PIN_Attiny1_to_2&PIN_NR_Attiny1_to_2));
	  PORT_Attiny2_to_1&=~PIN_NR_Attiny2_to_1;
	#endif
 	
}
void Send_Int_Attiny2(int16_t send){
 	Send_Byte_Attiny2((send&0xFF00)>>8);
 	Send_Byte_Attiny2((send&0x00FF)   );
}
 
unsigned char USART_Receive( void )
{
 	/* Wait for data to be received */
 	while ( !(UCSRA & (1<<RXC)) );
 	
 	/* Get and return received data from buffer */
 	return UDR;
 	
}

void setoutput(int8_t free_pin_nr,int8_t level)
{
	#ifndef Attiny2
	switch(free_pin_nr){
		case COM_PIN_1:
			DDR_FREE_PIN_1|=NR_FREE_PIN_1;
			PORT_FREE_PIN_1|=(NR_FREE_PIN_1)&(level);
			break;
		case COM_PIN_2:
			DDR_FREE_PIN_2|=NR_FREE_PIN_2;
			PORT_FREE_PIN_2|=(NR_FREE_PIN_2)&(level);
			break;
		case COM_PIN_3:
			DDR_FREE_PIN_3|=NR_FREE_PIN_3;
			PORT_FREE_PIN_3|=(NR_FREE_PIN_3)&(level);
			break;
		case COM_PIN_4:	
			DDR_FREE_PIN_4|=NR_FREE_PIN_4;
			PORT_FREE_PIN_4|=(NR_FREE_PIN_4)&(level);
			break;
	}
	#endif
}
int8_t checkpin(int8_t free_pin_nr)
{
	#ifndef Attiny2
	int8_t pinlevel=0;
	switch(free_pin_nr){
		case 1:
			DDR_FREE_PIN_1&=~NR_FREE_PIN_1;
			pinlevel=PIN_FREE_PIN_1 & NR_FREE_PIN_1;
			break;
		case 2:
			DDR_FREE_PIN_2&=~NR_FREE_PIN_2;
			pinlevel=PIN_FREE_PIN_2 & NR_FREE_PIN_2;
			break;
		case 3:
			DDR_FREE_PIN_3&=~NR_FREE_PIN_3;
			pinlevel=PIN_FREE_PIN_3 & NR_FREE_PIN_3;
			break;
		case 4:	
			DDR_FREE_PIN_4&=~NR_FREE_PIN_4;
			pinlevel=PIN_FREE_PIN_4 & NR_FREE_PIN_4;
			break;
	}
	return (pinlevel?HIGHT:LO);
	#endif
}
void setbuzzer(uint16_t ICR,uint8_t prescaler,uint16_t OCR,uint8_t pwm_pin)
{
	if (prescaler>0x07)prescaler=BUZZER_DEFAULT_PRESCALER;
#ifndef Version2
	#ifndef Attiny2
	DDR_BUZZER|=PIN_NR_BUZZER;
  	TCCR0A |= 0b10000000; // for OC0A
  	// prescaler
  	TCCR0B &= ~0x07;
	TCCR0B |= (prescaler & 0x07);

  	OCR_BUZZER = ICR;
	#endif
#else
	if (pwm_pin){
	#ifdef Attiny1
		//Attiny 1 have nothing to do
		return;
	#endif
	#ifndef Attiny1
		DDR_PWM_PIN|=PIN_NR_PWM_PIN;
  		TCCR1A |= 0b10000000; // for OC1A
  		OCR_PWM_PIN = OCR;
	#endif
	}
	else{
	#ifdef Attiny2
		//Attiny 2 have nothing to do
		return;
	#endif
	#ifndef Attiny2
		DDR_BUZZER|=PIN_NR_BUZZER;
  		TCCR1A |= 0b00100000; // for OC1B
  		OCR_BUZZER = OCR;
	#endif
	}
  	// prescaler
  	TCCR1B &= ~0x07;
  	TCCR1B |= (prescaler & 0x07);
  	ICR1  = ICR;//frequency
#endif
 
}
void play_frequency(uint16_t freq,uint8_t level, uint16_t dur,uint8_t pwm_pin)
{
	uint16_t ICR=(uint16_t)((F_CPU+(freq>>1))/freq);
	if (level>15)level=15;
	setbuzzer(ICR,0x01,ICR>>(16-(level)),pwm_pin);
	Buzzer_timeout=(uint16_t)(((uint32_t)dur*freq)/1000);
	PWM_Pin_or_Buzzer_freq=pwm_pin;
}
void disablebuzzer(void)
{
#ifndef Version2
  	TCCR0A &= ~0b11000000; // deaktivate Output
#else
  	TCCR1A &= ~0b00110000; // deaktivate Output
#endif
}
void disablePWMpin(void)
{
#ifdef Version2
	#ifdef Attiny2
		//deactivate Output=> make it to input
		DDR_PWM_PIN&=~PIN_NR_PWM_PIN;
	#endif
  	TCCR1A &= ~0b11000000; // deaktivate Output
#endif
}
