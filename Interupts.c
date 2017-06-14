/*
 * Program for the real-time part of the raspibot which is intended to run on
 * the ATtiny2313. The datasheet is available under
 * http://www.atmel.com/Images/doc2543.pdf
 *
 * @author Leander Herr <leander.herr@2011.tu-chemnitz.de>
 * @author Marcus Schlutter <marcus.homunculus@gmail.com>
 */
/******************************************************************************
 * Interrupts
 */

// for interrupt vector naming refer to:
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/group__avr__interrupts.html
/**
 * Interrupt service routine for the right wheel encoder on net OUTA2. Due the
 * to the offset of the second sensor the turning direction can be measured.
 * If the second sensor on Pin 7 signals a bright underground the wheel is
 * turning to move the robot forwards else the robot is moving backwards.
 */
ISR( PCINT2_vect)
{
  // check if the 2nd Encoder value is high, too
  // Pin 8 is the PD4, means the 5th Pin in Port D
  if ((PIN_ENC_R1 & PIN_NR_ENC_R1)==((PIN_ENC_R2 & PIN_NR_ENC_R2)>>ENC_DIFF_R1_R2))
    rightEncoderCount++;
  else
    // means there's a black space -> wheel turns backwards
    rightEncoderCount--;
}

/**
 * Interrupt service routine for the left wheel encoder on net OUTA1. Due the
 * to the offset of the second sensor the turning direction can be measured.
 * If the second sensor on Pin 13 signals a bright underground the wheel is
 * turning to move the robot forwards else the robot is moving backwards.
 */
ISR( PCINT1_vect)
{

  // check if what the second sensor signals; Pin 4 is the 2nd Pin on Port B
  if ((PIN_ENC_L1 & PIN_NR_ENC_L1)==((PIN_ENC_L2 & PIN_NR_ENC_L2)>>ENC_DIFF_L1_L2))
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

//PWM-Motor Interupt
#ifndef Version2
 ISR( TIMER1_OVF_vect)
#else
 ISR( TIMER0_OVF_vect)
#endif
{
#ifndef Attiny1
#ifndef no_target
  if (target_l||target_r) {
    if (((target_r>0)&&(rightEncoderCount >= target_r))
        || ((target_r<0)&&(rightEncoderCount <= target_r))
        || ((target_l>0)&&(leftEncoderCount >= target_l))
        || ((target_l<0)&&(leftEncoderCount <= target_l)))
    {
      // stop the motors
      OCR_R = 0;
      OCR_L = 0;
      motor_speed_l=0;
      motor_speed_r=0;
      // and unset the phase pins
      PORT_DIR_R  &= ~(PIN_DIR_R);
      PORT_DIR_L  &= ~(PIN_DIR_L);
      //and reset target
      target_r = 0;
      target_l = 0;
    }
  }
#endif
  {
    #ifdef Motor_control
    static int16_t encoderl_old,encoderr_old;
    if ((motor_speed_l!=0)&&(motor_speed_r!=0)){
          //do some PID - Magic??
          static uint16_t counter=0;//do the Magic not at every Overflow
          if(counter>=counter_drive){
              counter=0;
              int16_t drive_l=leftEncoderCount-encoderl_old;
              int16_t drive_r=rightEncoderCount-encoderr_old;
              
              /*static*/ int16_t error;
              //int16_t error_old=error;
              error=(drive_l*eeprom_read_byte(&EncoderMulti))/motor_speed_l - (drive_r*eeprom_read_byte(&EncoderMulti))/motor_speed_r;
              
              static int16_t P,I=0/*,D=0*/;
              P=error*(int16_t)eeprom_read_word((uint16_t*)&PID_P)/1000;
              I+=error*(int16_t)eeprom_read_word((uint16_t*)&PID_I)/1000;
              //D=(error-error_old)*PID_D/1000;
    
              int16_t speedright=motor_speed_r+P+I/*+D*/;
              int16_t speedleft=motor_speed_l-P-I/*-D*/;
              speedright=(speedright>max_speed_value)?max_speed_value:(speedright<-max_speed_value)?-max_speed_value:speedright;
              speedleft=(speedleft>max_speed_value)?max_speed_value:(speedleft<-max_speed_value)?-max_speed_value:speedleft;
              
              setMotorSpeed_r((int8_t)speedright);
              setMotorSpeed_l((int8_t)speedleft);
              
              encoderr_old=rightEncoderCount;	
              encoderl_old=leftEncoderCount;
          }
          else counter++;
      }
      else{
          encoderl_old=leftEncoderCount;
          encoderr_old=rightEncoderCount;	
          setMotorSpeed_l(motor_speed_l);
          setMotorSpeed_r(motor_speed_r);
          
      }/*for not drive control*/
    #endif
  }
#endif
}

//PWM-Buzzer Interupt
#ifndef Version2
 ISR( TIMER0_OVF_vect)
#else
 ISR( TIMER1_OVF_vect)
#endif
{
 if (Buzzer_timeout)
	Buzzer_timeout--;
 else {
	PWM_Pin_or_Buzzer_freq?(OCR_PWM_PIN=0):(OCR_BUZZER=0);
#ifdef Music_control
	 if (music_que_store!=music_que_load){
	 //new frequency
	 	uint16_t freq=music_que[++music_que_load];
	 	uint16_t dur=music_que[++music_que_load];
	 	uint16_t level=music_que[++music_que_load];
	  	music_que_load%=music_que_len;
		play_frequency(freq,level&0xFF,dur,(level>>8) & COM_SET_PWM_MASK);
	 } 
#endif
 }
}
