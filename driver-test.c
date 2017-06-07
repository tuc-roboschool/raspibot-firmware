/*
 * Motor driver test for the Raspibot 2
 *
 * @author Leander Herr <leander.herr@2011.tu-chemnitz.de>
 * @author Marcus Schlutter <marcus.homunculus@gmail.com>
 * @author Markus Dittmann <markus.dittmann@s2008.tu-chemnitz.de>
 */
#define F_CPU 8000000UL 
//system clk is at 8MHz
 //TODO:PID

#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "defines.h"
#include "functions.h"
#include "Interupts.c"
#include "functions.c"
/**
 */
void main_action(void)
{
  // the communication is implemented as a state machine -> initialize the
  // state variable
  static int state = WAIT;
  static uint8_t command;   // as static to always have the last command avail
  static int16_t data1;     // a place to store & combine data packages (bytes)
  static uint16_t data2;    // yet another data buffer
  static uint8_t dataInData2;   // just a flag

  // now find out what to do
  switch (state)
  {
  // state S0 is the entering state where the command is received
  case WAIT:
    {
      command = USART_Receive(); // save the data before its gone
      if (command & 0x80){
	      if ((command & 0xF0)==SET_PIN){
	      	//set PIN
		setoutput(command & COM_PIN_MSK,command & COM_PIN_HIGHT);
            	Send_Byte(ACK);
	      }
	      else if ((command & 0xF0)==GET_PIN){
	      	//get PIN
		Send_Byte(checkpin(((command & COM_PIN_MSK)>>COM_PIN_OFFSET)+1));
	      }
	      else Send_Byte(NAK);
      }
      else
        switch (command)
        {
        case ALIVE:
          {
            // the raspi just wants to know if you're there, so confirm you're
            Send_Byte(ACK);
            // a different state then S0 is not necessary due S0 waits for next
            // command
            break;
          }
        case ENC_R:
          {
            // make a copy first
            int16_t count = getEncoderCountRight();
            Send_Int(count);
            break;
          }
        case ENC_L:
          {
            // same procedure as for the right side
            int16_t count = getEncoderCountLeft();
            Send_Int(count);
            break;
          }
        case ENC_B:
          {
            // store the data for the left wheel away, make the data for the right
            // wheel ready to send
            int16_t count = getEncoderCountLeft();
            Send_Int(count);
            count = getEncoderCountRight();
            Send_Int(count);
            break;

          }
        case RST_R:
          {
            resetEncoderCountRight();
            // and confirm that we have done so
            Send_Byte(ACK);
            // and stay with waiting for a command
            break;
          }
        case RST_L:
          {
            resetEncoderCountLeft();
            Send_Byte(ACK);
            break;
          }
        case RST_B:
          {
            resetEncoderCountRight();
            resetEncoderCountLeft();
            Send_Byte(ACK);
            break;
          }
        case SET_ML:
        case SET_MR:
        case SET_M:

        case SET_TL:
        case SET_TR:
        case SET_T:

       	case COM_BUZZER_FREQ:
       	case COM_SET_BUZZER:
        case COM_SET_PWM_PIN:
          {
            // next we've to expect 2(or more) bytes
            state = RECV_2_1;
            break;
          }
        case STO_M:
          {
            // just deactivate the motors
            motor_speed_r=0;
            motor_speed_l=0;
            setMotorSpeed(0, 0);
            // and confirm that
            Send_Byte(ACK);
            break;
          }
        case REACH:
          {
            // check if one of the encoder values is within tolerance
            int16_t difference = getEncoderCountRight() - getTargetCount_r();
            int16_t absDifference = difference < 0 ? -difference : difference;
            if(absDifference < reachToleranceInc)
              Send_Byte(ACK);
            difference = getEncoderCountLeft() - getTargetCount_l();
            absDifference = difference < 0 ? -difference : difference;
            if(absDifference < reachToleranceInc)
              Send_Byte(ACK);
            Send_Byte(NAK);
            break;
          }
        case ECHO:
          {
            uint8_t echo_byte = USART_Receive();
            Send_Byte(echo_byte);
            break;
          }
        case COM_USET_BUZZER:
          {
            disablebuzzer();
            // and confirm that we have done so
            Send_Byte(ACK);
            // and stay with waiting for a command
            break;
          }
	case COM_USET_PWM_PIN:
          {
            disablePWMpin();
            // and confirm that we have done so
            Send_Byte(ACK);
            // and stay with waiting for a command
            break;
          }
        default:
          {
            // put a NAK there, maybe the raspi is reading it
            Send_Byte(NAK);
            break;
          }
        }
      break;
    }
  case RECV_2_1:
    {
      // state for receiving the first byte of a 2 byte integer
      // the MSB always comes first ...
      int16_t firstChunk=0;
      if(command != SET_ML)
        firstChunk = (int16_t) USART_Receive();
      data1 = firstChunk << 8;
      // receiving the seconf byte of a 2 byte integer
      // this is the part with the LSB
      int16_t secondChunk=0;
      if(command != SET_MR)
        secondChunk = (int16_t) USART_Receive();
      // if we made it this far there's no reason something should fail, so we
      // can confirm now already. Don't let the raspi wait ..
      Send_Byte(ACK);
      // combine both bytes
      data1 |= secondChunk;
      // see what we have to do next with it
      if((command == SET_T)||(command == SET_TL))
        setTargetCount_l(data1);
      if((command == SET_T)||(command == SET_TR))
        setTargetCount_r(data1);
      //if((command == SET_T)||(command == SET_TL)||(command == SET_TR))//nothing
      if((command == SET_M)||(command == SET_ML)||(command == SET_MR)){
        // disassemble data again
        int8_t rightThrust = (int8_t) (firstChunk);
        int8_t leftThrust = (int8_t) (secondChunk);
/*        if (command == SET_M){
          motor_speed_r=rightThrust;
          motor_speed_l=leftThrust;
          #ifndef Motor_control
          setMotorSpeed(rightThrust, leftThrust);
          #endif
        }*/
        if ((command == SET_MR)||(command == SET_M)){
          motor_speed_r=rightThrust;
          #ifndef Motor_control
          setMotorSpeed_r(rightThrust);
          #endif
        }
        if ((command == SET_ML)||(command == SET_M)){
          motor_speed_l=leftThrust;
          #ifndef Motor_control
          setMotorSpeed_l(leftThrust);        
          #endif
        }
        data1 = 0;
        state = WAIT;
      }
      else if((command==COM_SET_BUZZER)||(command==COM_SET_PWM_PIN)||(command==COM_BUZZER_FREQ))
      {
      	uint8_t data2=USART_Receive();
      	uint16_t data3=((uint16_t) USART_Receive())<<8;
	data3|=(uint16_t) USART_Receive();
	if (command==COM_SET_BUZZER)
	  play_frequency((uint16_t)data1,data2,data3);      
	else
	  setbuzzer((uint16_t)data1,data2,data3,command & COM_SET_PWM_MASK);
      }
      // and clear the data buffer again
      data1 = 0;
      state = WAIT;
      break;
    }
  case SEND_1_2:
    {
      uint8_t toSend = (uint8_t) (data1 >> 8);
      Send_Byte(toSend);
      // and cut away what have been send
      data1 &= 0xFF;
      state = SEND_2_2;
      break;
    }
  case SEND_2_2:
    {
      // the last byte is in data so cast it down to a byte and write it to the
      // SPI-register to be send away next
      Send_Byte((uint8_t) data1);
      if(command == ENC_B && !dataInData2) {
        // pull the data out of the 2nd buffer to the sending buffer
        data1 = data2;
        // and clear it
        data2 = 0;
        dataInData2 = 0;
        state = SEND_1_2;
      }
      else {
        // and clear the data buffer
        data1 = 0;
        state = WAIT;
      }
      break;
    }
  default:
    {
      // in case we run into trouble just send a NAK
      Send_Byte(NAK);
      break;
    }
  }
}

void init()
{
  // Disable PortA (Pins 1,4 & 5)
  DDRA = 0;
  PORTA = 0;
  // Define PortB in the matter what will be input and what will be output
  //define PB3 u PB4 as Output(OCR1A/B(Motors))
  DDRB = 0x18;
  PORTB = 0;
  // Define data direction for Port D and disable Pins doing that
  DDRD = 0;
  PORTD = 0;
  
  DDR_PHASE_R|=PIN_PHASE_R;
  DDR_PHASE_L|=PIN_PHASE_L;

  UCSRA=0x00;//UART status
  UCSRB=0x18;//Receive/Transmitt enabled
  UCSRC=0x2E;//Asynchron Mode, Even Parity, 2 Stop Bits, 8 Bit data length
  UBRRH=0;
  UBRRL=12;//1MHz -> 4800 baud(+0,2%)
  
  // add the PUD-Bit from MCUCR (refer to datasheet page 30)
  // Pull ups functions disable
  MCUCR = 1 << 7;

  GIMSK = 0b00011000;
  // we have two external interrupt sources: Pin 5 (PCINT2) and Pin 4 (PCINT1)

  // enable PCINT1 and PCINT2
  PCMSK1 = PCMSK1_aktiv;//right
  PCMSK2 = PCMSK2_aktiv;//left

#ifndef Version2
  // PWM-configuration
  // Set to use a pseudo fast PWM (datasheet page 74)
  TCCR1A = 0b10010000; // for OC0A & OCRB as output
  // use PWM-mode 1 (datasheet page 75 table 40)
  // WGM2 stays zero due the counter shall go through all the way to 255
  //  TCCR1A |= 0b00000000; // choose a phase correct PWM
  TCCR1B = 0x12;
  // prescaler 8:0x02
  // (datasheet page 77)
  // set the registers to be compared against to zero to disable the motors
  ICR1  = max_speed_value;//Max counter -> Max speed value
#else
  // PWM-configuration
  // Set to use a pseudo fast PWM (datasheet page 74)
  TCCR0A = 0b10100001; // for OC0A & OCRB as output
  // use PWM-mode 1 (datasheet page 75 table 40)
  // WGM2 stays zero due the counter shall go through all the way to 255
  // TCCR0A |= 0x01;
  //  TCCR0B |= 0b00000000; // choose a phase correct PWM top at 255
  // prescaler 8
  TCCR1B = 0x02;
  // (datasheet page 77)
  // set the registers to be compared against to zero to disable the motors
  //Max speed value:fixed at 255
#endif

  setMotorSpeed(0,0);

#ifndef Version2
  // BUZZER-configuration
  //TCCR0A |= 0b10000000; // for OC0A
  // use PWM-mode 1 (datasheet page 75 table 40)
  // WGM2 stays zero due the counter shall go through all the way to 255
  //  TCCR0A |= 0b00000000; // choose a phase correct PWM no soundlevel controll
  TCCR0B = 0x0A;
  TCCR0A = 0b00000001; // choose a phase correct PWM (no) freq controll
  //TCCR0B |= 0x00;
  // prescaler 8:0x02
  // (datasheet page 77)
  // set the registers to be compared against to zero to disable the motors
#else
  // BUZZER-configuration
  TCCR1A = 0;
  //TCCR1A |= 0b10000000; // for OC0A only when needed
  //TCCR1B |= 0b00100000; // for OC0B only when needed
  // use PWM-mode 1 (datasheet page 75 table 40)
  // WGM2 stays zero due the counter shall go through all the way to 255
  //  TCCR1A |= 0b00000000; // choose a phase correct PWM
  TCCR1B = 0x12;
  // prescaler 8:0x02
  // (datasheet page 77)
  // set the registers to be compared against to zero to disable the motors
  // ICR1  = 0;//Max counter -> determinate frequency
#endif
  
  TIMSK=0x82;//both Overflow Interupts enable

  // disable the control pins for the direction of the motors
  //PORT_PHASE_R &= ~(PIN_PHASE_R);  // right motor phase
  //PORT_PHASE_L &= ~(PIN_PHASE_L);  // left motor phase

  // reset encoders
  resetEncoderCountLeft();
  resetEncoderCountRight();

  // activate the Interrupts
  // here globally
  SREG |= 1 << 7;
}

int main()
{
  init();
  while (42) {
    main_action();
    // just wait
    //_delay_ms(10);
  }
  return 0;
}
