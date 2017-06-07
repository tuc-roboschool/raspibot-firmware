/*
USART: Raspy: 1Byte was ist?
				->Encoder?->Pin4
					->get Encoder?->Pin2
					->reset Encoder?->Pin3
						->welcher Encoder?->Pin0/Pin1
	Attiny antwortet:1Byte was ist?
				-> siehe raspy, Bestaetigung/einordnung
					->wenn get encoder:
						->Hight encoder Byte
							->Low encoder Byte
*/

#define UART_get_encoder 0x04
#define UART_reset_encoder 0x08

#define UART_encoder 0x80

#define UART_encoder_l 0x02
#define UART_encoder_r 0x01

#define UART_encoder_l_reseted 0x1A
#define UART_encoder_r_reseted 0x19

#define UART_encoder_l_value 0x16
#define UART_encoder_r_value 0x15


#define UART_Motor 0x40
#define UART_Motor_control 0x20

#define UART_Motor_1_byte 0x10

#define UART_Motor_l 0x02
#define UART_Motor_r 0x01


#define F_CPU           1000000UL     // Systemtakt 1 MHz
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#define counter_drive 0
#define EncoderMulti 2000
#define PID_P 1
#define PID_I 0
#define PID_D 0

int16_t Encoderr,Encoderl;
int16_t Motorr,Motorl;
uint8_t Motor_control;

void set_motor_l(int16_t speed);
void set_motor_r(int16_t speed);
inline int16_t get_encoder_l();
inline int16_t get_encoder_r();

ISR(PCINT0_vect)
{
	if ((PINB & 0x01)==((PIND & 0x40)>>6)){
		Encoderr++;
	}
	else {
		Encoderr--;
	}
}
ISR(PCINT2_vect)
{
	if ((PINB & 0x80)==((PINB & 0x40)<<1)){
		Encoderl++;
	}
	else {
		Encoderl--;
	}
}
ISR(TIMER1_OVF_vect)//Timer Overflow -> drive Control 
{
	static int16_t encoderl_old,encoderr_old;
	if(Motor_control){
		//do some PID - Magic
		static uint8_t counter=0;//do the Magic not at every Overflow
		if(counter>=counter_drive){
			int16_t drive_l=get_encoder_l()-encoderl_old;
			int16_t drive_r=get_encoder_r()-encoderr_old;
			
			static int16_t error;
			int16_t error_old=error;
			error=(drive_l*EncoderMulti)/Motorl - (drive_r*EncoderMulti)/Motorr;
			
			static int16_t P,I=0,D=0;
			P=error*PID_P;
			I+=error*PID_I;
			D=(error-error_old)*PID_D;
			
			set_motor_l(Motorl-P-I-D);
			set_motor_r(Motorr+P+I+D);
			
			encoderl_old=get_encoder_l();
			encoderr_old=get_encoder_r();	
		}
		else counter++;
	}
	else{
		encoderl_old=get_encoder_l();
		encoderr_old=get_encoder_r();	
		
	}
}

void reset_encoder_r(){
	Encoderr=0;
}
void reset_encoder_l(){
	Encoderl=0;
}
inline int16_t get_encoder_r(){
	return(Encoderr);
}
inline int16_t get_encoder_l(){
	return(Encoderl);
}
void set_motor_r(int16_t speed){
	if (speed<0){
		speed=-speed;
		//invert
		PORTB|=0x20;
	}
	else{
		PORTB&=~0x20;
	}
	OCR1B=speed;
}
void set_motor_l(int16_t speed){
	if (speed<0){
		speed=-speed;
		//invert
		PORTB&=~0x04;
	}
	else{
		PORTB|=0x04;
	}
	OCR1A=speed;
}

void Send_Byte(uint8_t send){

	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );
	
	/* Put data into buffer, sends the data */
	UDR = send;
	
}
void Send_Int(int16_t send){
	Send_Byte((send&0xFF00)>>8);
	Send_Byte((send&0x00FF)   );
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) );
	
	/* Get and return received data from buffer */
	return UDR;
	
}

int main(){
	
	//Disable Unneeded Pins (for ADC)
	PORTA=0;
	DDRA=0;
	PORTB=0;
	DDRB=0x3C;//Motors as Output, Encoder as Input
	PORTD=0;
	DDRD=0x01;//TX as Output
	
	UCSRA=0x00;//UART status
	UCSRB=0x18;//Receive/Transmitt enabled
	UCSRC=0x2E;//Asynchron Mode, Even Parity, 2 Stop Bits, 8 Bit data length
	UBRRH=0;
	UBRRL=6;//1MHz -> 9600 baud(-7%)

	GIMSK=0x30;//PCINT7..0 & 17..11 enabled
	PCMSK2=0x40;//PCINT17 enabled only
	PCMSK=0x40;//PCINT6 enabled only
	//	-> ISR(USART_RXC_vect){}

	TCCR1A=0xA0;//OCR1A/B Clear at upcounting set at downcounting
	TCCR1B=0x10;//Phase and frequency correct PWM TOP=ICR, no prescaler
	ICR1=10000;//10000 steps for Motors
	//OCR1A=0x0000;
	TIMSK=80;//Overflow Interupt

	Motor_control=0;//disable Motor control (with encoders)
	
	reset_encoder_r();
	reset_encoder_l();
	
	
	while(1){
		uint8_t UART_BYTE=USART_Receive();
		
		if (UART_BYTE & UART_encoder){
			if(UART_BYTE & UART_get_encoder){

				int16_t encr=get_encoder_r();
				int16_t encl=get_encoder_l();

				if(UART_BYTE & UART_encoder_l){
					Send_Byte(UART_encoder_l_value);
					Send_Int(encl);
				}
				if(UART_BYTE & UART_encoder_r){
					Send_Byte(UART_encoder_r_value);
					Send_Int(encr);
				}
			}
			if(UART_BYTE & UART_reset_encoder){
				if(UART_BYTE & UART_encoder_l){
					reset_encoder_l();
					Send_Byte(UART_encoder_l_reseted);
				}
				if(UART_BYTE & UART_encoder_r){
					reset_encoder_r();
					Send_Byte(UART_encoder_r_reseted);
				}
			}
		}
		else{
			if (UART_BYTE & UART_Motor){
				if (UART_BYTE & UART_Motor_control){
					//set motor control
					if (UART_BYTE & UART_Motor_l){
						int8_t HI_BYTE=USART_Receive();
						
						if (UART_BYTE & UART_Motor_1_byte){
							Motorl=HI_BYTE*79 /*10000/127*/;
						}
						else{
							uint8_t LO_BYTE=USART_Receive();
							Motorl=(HI_BYTE<<8)|LO_BYTE;
						}
						
						set_motor_l(Motorl);
					}
					if (UART_BYTE & UART_Motor_r){
						int8_t HI_BYTE=USART_Receive();
						
						if (UART_BYTE & UART_Motor_1_byte){
							Motorr=HI_BYTE*79 /*10000/127*/;
						}
						else{
							uint8_t LO_BYTE=USART_Receive();
							Motorr=(HI_BYTE<<8)|LO_BYTE;
						}

						set_motor_r(Motorr);
					}
					Motor_control=1;//enable Motor control (with encoders)

				}
				else{
					int16_t Motorr_temp,Motorl_temp;
					if (UART_BYTE & UART_Motor_l){
						Motor_control=0;//disable Motor control (with encoders)
						int8_t HI_BYTE=USART_Receive();
						
						if (UART_BYTE & UART_Motor_1_byte){
							Motorl_temp=HI_BYTE*79 /*10000/127*/;
						}
						else{
							uint8_t LO_BYTE=USART_Receive();
							Motorl_temp=(HI_BYTE<<8)|LO_BYTE;
						}
						
						set_motor_l(Motorl_temp);
					}
					if (UART_BYTE & UART_Motor_r){
						Motor_control=0;//disable Motor control (with encoders)
						int8_t HI_BYTE=USART_Receive();
						
						if (UART_BYTE & UART_Motor_1_byte){
							Motorr_temp=HI_BYTE*79 /*10000/127*/;
						}
						else{
							uint8_t LO_BYTE=USART_Receive();
							Motorr_temp=(HI_BYTE<<8)|LO_BYTE;
						}

						set_motor_r(Motorr_temp);
					}					
				}
			}
		}
	}
	return 0;
}
