#ifndef LAUNCH_PAD_H
#define LAUNCH_PAD_H

//colors for LaunchPad_Output
#define RED       0x01
#define GREEN     0x02
#define BLUE      0x04
#define YELLOW    0x03
#define SKY_BLUE  0x06
#define WHITE     0x07
#define PINK      0x05

/**
 * Initialize LaunchPad I/O (switches and LEDs)
 *
 * @param  none
 * @return none
 * @brief  Initialize Switch inputs and LED outputs
 */
void LaunchPad_Init(void);


/**
 * Input from two switches on LaunchPad
 * Value returned in postive logic
 *
 * @param  none
 * @return two-bit value of switches
 *         0x00 none
 *         0x01 Button1
 *         0x02 Button2
 *         0x03 both Button1 and Button2
 * @brief  Input from Switches
 */
uint8_t LaunchPad_Input(void);

/**
 * Output to red LED on LaunchPad
 *
 * @param  data is value to send to LED,
 * 0 is off; 1 is on
 * @return none
 * @brief  Output to red LED
 */
void LaunchPad_LED(uint8_t data);

/**
 * Output to 3-color LED on LaunchPad
 *
 * @param  data is value to send to LEDs
 * 0 is off; bit0 is red; bit1 is green; bit2 is blue
 * @return none
 * @brief  Output to 3-color LED
 */
void LaunchPad_Output(uint8_t data);

#endif

#ifndef LAUNCH_PAD_H
#define LAUNCH_PAD_H

//colors for LaunchPad_Output
#define RED       0x01
#define GREEN     0x02
#define BLUE      0x04
#define YELLOW    0x03
#define SKY_BLUE  0x06
#define WHITE     0x07
#define PINK      0x05

/**
 * Initialize LaunchPad I/O (switches and LEDs)
 *
 * @param  none
 * @return none
 * @brief  Initialize Switch inputs and LED outputs
 */
void LaunchPad_Init(void);


/**
 * Input from two switches on LaunchPad
 * Value returned in postive logic
 *
 * @param  none
 * @return two-bit value of switches
 *         0x00 none
 *         0x01 Button1
 *         0x02 Button2
 *         0x03 both Button1 and Button2
 * @brief  Input from Switches
 */
uint8_t LaunchPad_Input(void);

/**
 * Output to red LED on LaunchPad
 *
 * @param  data is value to send to LED,
 * 0 is off; 1 is on
 * @return none
 * @brief  Output to red LED
 */
void LaunchPad_LED(uint8_t data);

/**
 * Output to 3-color LED on LaunchPad
 *
 * @param  data is value to send to LEDs
 * 0 is off; bit0 is red; bit1 is green; bit2 is blue
 * @return none
 * @brief  Output to 3-color LED
 */
void LaunchPad_Output(uint8_t data);

#endif
