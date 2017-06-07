/*
 * Program for the real-time part of the raspibot which is intended to run on
 * the ATtiny2313. The datasheet is available under
 * http://www.atmel.com/Images/doc2543.pdf
 *
 * @author Leander Herr <leander.herr@2011.tu-chemnitz.de>
 * @author Marcus Schlutter <marcus.homunculus@gmail.com>
 */
/******************************************************************************
 * Function declarations
 */

//send something over USART
void Send_Byte(uint8_t send);
void Send_Byte_Attiny2(uint8_t send);
void Send_Int(int16_t send);
//receive something over USART
unsigned char USART_Receive( void );

/**
 * Allows to set the target count for the encoders through a function call
 * instead of accessing the global directly.
 * If zero the value will be ignored by the other functions which is equal to
 * clear the value.
 *
 * @param newTarget the encoder increments the robot shall stop if reached with
 *                  the first wheel
 */
void setTargetCount_l(int16_t newTarget);
void setTargetCount_r(int16_t newTarget);

/**
 * Allows to get the target encoder count through a function call instead of
 * accessing the global directly
 *
 * @return the set count of increments the robot shall stop at as an 16-bit int
 */
int16_t getTargetCount_l();
int16_t getTargetCount_r();

void setPIvalues(int16_t P, int16_t I, uint8_t encoderMulti);

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
void setMotorSpeed_l(int8_t speedLeft);
void setMotorSpeed_r(int8_t speedRight);
void setoutput(int8_t free_pin_nr,int8_t level);
int8_t checkpin(int8_t free_pin_nr);
void setbuzzer(uint16_t ICR,uint8_t prescaler,uint16_t OCR,uint8_t pwm_pin);
void play_frequency(uint16_t freq,uint8_t level, uint16_t dur,uint8_t pwm_pin);
void disablebuzzer(void);
void disablePWMpin(void);
void init(void);
