/*
 * Program for the real-time part of the raspibot which is intended to run on
 * the ATtiny2313. The datasheet is available under
 * http://www.atmel.com/Images/doc2543.pdf
 *
 * @author Leander Herr <leander.herr@2011.tu-chemnitz.de>
 * @author Marcus Schlutter <marcus.homunculus@gmail.com>
 */

#define Version2
//#define Attiny1
//#define Attiny2
#define Motor_control
//#define Music_control

#define opt_only_one_Attiny
#define no_target
#define no_extra_pins

/*
 * Communication messages:
 */
 
#define ALIVE  0x01 // the raspi checks if this program is capable of answering
#define ENC_R  0x02 // the raspi requests the encoder count for the right wheel
#define ENC_L  0x03 // the raspi requests the encoder count for the left wheel
#define ENC_B  0x04 // the raspi requests both encoder counts
#define RST_R  0x05 // reset the right encoder count
#define RST_L  0x06 // reset the left encoder count
#define RST_B  0x07 // reset both encoder counts
#define ECHO   0x0C // return the next sended Byte
#define SET_TL 0x2A // set the target count to stop at for left Motor
#define SET_TR 0x26 // set the target count to stop at for right Motor
#define SET_TB 0x2E // set the target count to stop at for both Motors
#define SET_ML 0x29 // set new thrust level for left Motor
#define SET_MR 0x25 // set new thrust level for right Motor
#define SET_MB 0x2D // set new thrust level for both motors
#define STO_M  0x21 // stop motors
#define SET_PI 0x2F // set PI-value(2xint16_t(1/100 P/I Value),uint8_t Encodermultiplikator)
#define ACK    0x10 // confirm received command
#define ACK2   0x10 // confirm received command for Attiny 2
#define NAK    0x14 // tell the raspi something went south
#define NAK2   0x14 // tell the raspi something went south for Attiny 2 for Attiny 2

//extra pins 0b11Y0.0XXZ
//Y==1 => SET PIN Y==0 => GET PIN
//XX:PIN_NR
//Z:Hight/Lo(setpin)
#define SET_PIN         0xE0
#define GET_PIN         0xC0
#define COM_PIN_MSK     0x06
#define COM_PIN_OFFSET  1
#define COM_PIN_1       0x00
#define COM_PIN_2       0x02
#define COM_PIN_3       0x04
#define COM_PIN_4       0x06
#define COM_PIN_HIGHT   0x01
#define COM_PIN_LO      0x00

//BUZZER
// FREQ: freqency(uint16_t),duration(uint16_t,ms),level(uint8_t(0-15))
// SET: ICR(uint16_t),OCR(uint16_t),Prescaler(uint8_t(0-7 see Datasheet))
#define COM_BUZZER_FREQ  0x40
#define COM_SET_BUZZER   0x42
#define COM_USET_BUZZER  0x43
#define COM_PWM_FREQ  	 0x44
#define COM_SET_PWM_PIN  0x46
#define COM_USET_PWM_PIN 0x47
//intern
#define COM_SET_PWM_MASK 0x04


/*
 * Port defines:
 */
 //Encoder
#define PIN_ENC_R1      PIND
#define DDR_ENC_R1      DDRD
#define PORT_ENC_R1     PORTD
#define PIN_NR_ENC_R1   0x10
#define PIN_ENC_R2      PIND
#define DDR_ENC_R2      DDRD
#define PORT_ENC_R2     PORTD
#ifdef Version2
 #define PIN_NR_ENC_R2   0x40
 #define ENC_DIFF_R1_R2  2
#else
 #define PIN_NR_ENC_R2   0x20
 #define ENC_DIFF_R1_R2  1
#endif

#define PIN_ENC_L1      PINA
#define DDR_ENC_L1      DDRA
#define PORT_ENC_L1     PORTA
#define PIN_NR_ENC_L1   0x01
#define PIN_ENC_L2      PINA
#define DDR_ENC_L2      DDRA
#define PORT_ENC_L2     PORTA
#define PIN_NR_ENC_L2   0x02
#define ENC_DIFF_L1_L2  1
  //INTERUPTS
//(v1.6)right B(0x10) as activ not A(0x20)
//(v2.0)right B(0x10) as activ not A(0x40)
#define PCMSK2_aktiv    0x10
//left A(0x01) as activ not B(0x02)
#define PCMSK1_aktiv    0x01

  //Motors
#ifdef Version2
 #define OCR_R          OCR0A
 #define DDR_PHASE_R	DDRB
 #define PIN_PHASE_R    0x04
#else
 #define OCR_R          OCR1A
 #define DDR_PHASE_R	DDRB
 #define PIN_PHASE_R    0x08
#endif

#define PORT_DIR_R    	PORTD
#define DDR_DIR_R     	DDRD
#define PIN_DIR_R     	0x08

#ifdef Version2
 #define OCR_L          OCR0B
 #define DDR_PHASE_L	DDRD
 #define PIN_PHASE_L	0x20
#else
 #define OCR_L		OCR1B
 #define DDR_PHASE_L	DDRB
 #define PIN_PHASE_L    0x10
#endif

#define PORT_DIR_L    	PORTD
#define DDR_DIR_L     	DDRD
#define PIN_DIR_L     	0x44

//free Pins
#ifdef Version2
 #define PORT_FREE_PIN_1 PORTB
 #define DDR_FREE_PIN_1  DDRB
 #define PIN_FREE_PIN_1  PINB
 #define NR_FREE_PIN_1   0x01
 #define PORT_FREE_PIN_2 PORTB
 #define DDR_FREE_PIN_2  DDRB
 #define PIN_FREE_PIN_2  PINB
 #define NR_FREE_PIN_2   0x02
 #define PORT_FREE_PIN_3 PORTB
 #define DDR_FREE_PIN_3  DDRB
 #define PIN_FREE_PIN_3  PINB
 #define NR_FREE_PIN_3   0x08
 #define PORT_FREE_PIN_4 PORTB
 #define DDR_FREE_PIN_4  DDRB
 #define PIN_FREE_PIN_4  PINB
 #define NR_FREE_PIN_4   0x10
 #define OCR_BUZZER      OCR1B
 #define DDR_BUZZER      DDRB
 #define PIN_NR_BUZZER   0x10
 #define OCR_PWM_PIN     OCR1A
 #define DDR_PWM_PIN     DDRB
 #define PIN_NR_PWM_PIN  0x08
#else
 #define PORT_FREE_PIN_1 PORTD
 #define DDR_FREE_PIN_1  DDRD
 #define PIN_FREE_PIN_1  PIND
 #define NR_FREE_PIN_1   0x40
 #define PORT_FREE_PIN_2 PORTB
 #define DDR_FREE_PIN_2  DDRB
 #define PIN_FREE_PIN_2  PINB
 #define NR_FREE_PIN_2   0x01
 #define PORT_FREE_PIN_3 PORTB
 #define DDR_FREE_PIN_3  DDRB
 #define PIN_FREE_PIN_3  PINB
 #define NR_FREE_PIN_3   0x02
 #define PORT_FREE_PIN_4 PORTB
 #define DDR_FREE_PIN_4  DDRB
 #define PIN_FREE_PIN_4  PINB
 #define NR_FREE_PIN_4   0x04
 #define OCR_BUZZER      OCR0A
 #define DDR_BUZZER      DDRB
 #define PIN_NR_BUZZER   0x04
 #define OCR_PWM_PIN     OCR0A
 #define DDR_PWM_PIN     DDRB
 #define PIN_NR_PWM_PIN  0x04
#endif

#define PIN_NR_Attiny1_to_2 	0x20
#define DDR_Attiny1_to_2 	DDRB
#define PORT_Attiny1_to_2 	PORTB
#define PIN_Attiny1_to_2 	PINB
#define PIN_NR_Attiny2_to_1 	0x40
#define DDR_Attiny2_to_1 	DDRB
#define PORT_Attiny2_to_1 	PORTB
#define PIN_Attiny2_to_1 	PINB

//setoutput
#define FREE_PIN_1     1
#define FREE_PIN_2     2
#define FREE_PIN_3     3
#define FREE_PIN_4     4
#define HIGHT          0xFF
#define LO             0

//buzzer default prescaler=1-> lowest frequenz=15Hz(F_CPU=1MHz)
//buzzer default prescaler=1-> lowest frequenz=122Hz(F_CPU=8MHz)
#define BUZZER_DEFAULT_PRESCALER    1
#define max_speed_value   127
//counter_drive: ~250Hz=timeroverflow =>1-> PI call every 4 ms
#define counter_drive 1
uint8_t EEMEM EncoderMulti=10;
int16_t EEMEM PID_P=0;
int16_t EEMEM PID_I=0;

#ifdef Music_control
#define music_que_len             3*10
uint16_t music_que[music_que_len];
uint8_t music_que_store=0;
uint8_t music_que_load=0;
#endif

/******************************************************************************
 * Globals
 */

volatile int16_t rightEncoderCount=0;
volatile int16_t leftEncoderCount=0;
// how far the robot shall move; if zero the value has to be ignored
#ifndef no_target
int16_t target_r;
int16_t target_l;
#endif
int16_t motor_speed_r;
int16_t motor_speed_l;
//Buzzer Timer
uint16_t Buzzer_timeout;
volatile uint8_t  PWM_Pin_or_Buzzer_freq;
// how many ms the program shall wait for the capacitor to unload
#define waitCycleCapacitorMs  2
// the tolerance in increments for what is considered standing on target
#define reachToleranceInc  10
// an enum to manage the states of the communications state machine
enum
{
  WAIT,       // wait for a command to come in
  RECV_2_1,   // receive the first byte of a 2 byte message
};
