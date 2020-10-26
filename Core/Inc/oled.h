/*
 * oled.h
 *      Author: Fernando Aguilar
 */

#ifndef SRC_OLED_H_
#define SRC_OLED_H_

#include "stm32l4xx_hal.h"
//#include "numbers.h"
//#include "message.h"

#ifndef HEIGHT
#define HEIGHT 128    /* Define the number of pixels in Y axis */
#endif

#ifndef WIDTH
#define WIDTH 128    /* Define the number of pixels in X axis */
#endif

#ifndef PAGES
#define PAGES 16     /* Define the number the pages OLED screen */
#endif

#ifndef OLED_ADDRESS
#define OLED_ADDRESS	0x78 /* Define the Address slave to communicate with I2C protocol. */
#endif

#ifndef REG_COMMAND
#define REG_COMMAND		0x00 /* Define the register value to indicate to the slave that the data is a command. */
#endif

#ifndef REG_DATA
#define REG_DATA		0x40 /* Define register value to indicate to the slave that the data to send is a data to send to the ram. */
#endif


/*
 * brief: initializes OLED Screen
 * parameters: None
 * retval: 0 -> The device is no ready
 * 		   1 -> Initializes OK
 *
 */
uint8_t OLED_init();


/*
 * brief: Clear OLED RAM
 * parameters: None
 * retval: None
 *
 */
void Clear_All_RAM();


/*
 * brief: Write data to config OLED.
 * parameters: uint8_t data write.
 * retval: None
 */
void writeCommand(uint8_t data);


/*
 * brief: Write data to Send to RAM OLED.
 * parameters: uint8_t data write.
 * retval: None
 */
void writeData(uint8_t data);


/*
 * brief: Function to print the predicted number
 * parameters: Vector pred[] that contains the numbers to print according with the prediction.
 * retval: None
 */
void printNumbers(int* pred, uint8_t vector_size, const int* y);

/*
 * brief: Function to print a bit map
 * parameters: Bit map to print.
 * retval: None
 */
void printBitmap(const unsigned char* Bitmap);


/*
 * brief: Print init Message
 * parameters: None.
 * retval: None
 */

#endif /* SRC_OLED_H_ */
