#ifndef PORTPINS_H_
#define PORTPINS_H_

// LED8 - Front left
#define LED8_FL_PIN                  0
#define LED8_FL_BIT                 (0x01 << LED8_FL_PIN)
#define LED8_FL_PORT                 P8
#define SET_LED8_FL_AS_AN_OUTPUT     LED8_FL_PORT->DIR |= LED8_FL_BIT
#define TURN_ON_LED8_FL         LED8_FL_PORT->OUT |= LED8_FL_BIT
#define TURN_OFF_LED8_FL         LED8_FL_PORT->OUT &= ~LED8_FL_BIT
#define TOGGLE_LED8_FL             LED8_FL_PORT->OUT ^= LED8_FL_BIT

// LED8 - Front Right
#define LED8_FR_PIN                  5
#define LED8_FR_BIT                 (0x01 << LED8_FR_PIN)
#define LED8_FR_PORT                 P8
#define SET_LED8_FR_AS_AN_OUTPUT     LED8_FR_PORT->DIR |= LED8_FR_BIT
#define TURN_ON_LED8_FR         LED8_FR_PORT->OUT |= LED8_FR_BIT
#define TURN_OFF_LED8_FR         LED8_FR_PORT->OUT &= ~LED8_FR_BIT
#define TOGGLE_LED8_FR             LED8_FR_PORT->OUT ^= LED8_FR_BIT

// LED8 - Back right
#define LED8_BR_PIN                  7
#define LED8_BR_BIT                 (0x01 << LED8_BR_PIN)
#define LED8_BR_PORT                 P8
#define SET_LED8_BR_AS_AN_OUTPUT     LED8_BR_PORT->DIR |= LED8_BR_BIT
#define TURN_ON_LED8_BR         LED8_BR_PORT->OUT |= LED8_BR_BIT
#define TURN_OFF_LED8_BR         LED8_BR_PORT->OUT &= ~LED8_BR_BIT
#define TOGGLE_LED8_BR             LED8_BR_PORT->OUT ^= LED8_BR_BIT

// LED8 - Back left
#define LED8_BL_PIN                  6
#define LED8_BL_BIT                 (0x01 << LED8_BL_PIN)
#define LED8_BL_PORT                 P8
#define SET_LED8_BL_AS_AN_OUTPUT     LED8_BL_PORT->DIR |= LED8_BL_BIT
#define TURN_ON_LED8_BL         LED8_BL_PORT->OUT |= LED8_BL_BIT
#define TURN_OFF_LED8_BL        LED8_BL_PORT->OUT &= ~LED8_BL_BIT
#define TOGGLE_LED8_BL             LED8_BL_PORT->OUT ^= LED8_BL_BIT

//  LED2 - Red
#define LED2_R_PIN                  0
#define LED2_R_BIT                 (0x01 << LED2_R_PIN)
#define LED2_R_PORT                 P2
#define SET_LED2_R_AS_AN_OUTPUT     LED2_R_PORT->DIR |= LED2_R_BIT
#define TURN_ON_LED2_RED            LED2_R_PORT->OUT |= LED2_R_BIT
#define TURN_OFF_LED2_RED           LED2_R_PORT->OUT &= ~LED2_R_BIT
#define TOGGLE_LED2_RED             LED2_R_PORT->OUT ^= LED2_R_BIT

//  LED2 - Green
#define LED2_G_PIN                  1
#define LED2_G_BIT                  (0x01 << LED2_G_PIN)
#define LED2_G_PORT                 P2
#define SET_LED2_G_AS_AN_OUTPUT     LED2_G_PORT->DIR |= LED2_G_BIT
#define TURN_ON_LED2_GREEN          LED2_G_PORT->OUT |= LED2_G_BIT
#define TURN_OFF_LED2_GREEN         LED2_G_PORT->OUT &= ~LED2_G_BIT
#define TOGGLE_LED2_GREEN           LED2_G_PORT->OUT ^= LED2_G_BIT

//  LED2 - Blue
#define LED2_B_PIN                  2
#define LED2_B_BIT                  (0x01 << LED2_B_PIN)
#define LED2_B_PORT                 P2
#define SET_LED2_B_AS_AN_OUTPUT     LED2_B_PORT->DIR |= LED2_B_BIT
#define TURN_ON_LED2_BLUE           LED2_B_PORT->OUT |= LED2_B_BIT
#define TURN_OFF_LED2_BLUE          LED2_B_PORT->OUT &= ~LED2_B_BIT
#define TOGGLE_LED2_BLUE            LED2_B_PORT->OUT ^= LED2_B_BIT

// Pushbutton S1
#define PUSHBUTTON_S1_PIN                       1
#define PUSHBUTTON_S1_BIT                       (0x01 << PUSHBUTTON_S1_PIN)
#define PUSHBUTTON_S1_PORT                      P1
#define SET_PUSHBUTTON_S1_TO_AN_INPUT           PUSHBUTTON_S1_PORT->DIR &= ~PUSHBUTTON_S1_BIT
#define ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S1   PUSHBUTTON_S1_PORT->REN |= PUSHBUTTON_S1_BIT
#define SELECT_PULL_UP_RESISTORS_S1             PUSHBUTTON_S1_PORT->OUT |= PUSHBUTTON_S1_BIT

// Pushbutton S2
#define PUSHBUTTON_S2_PIN                       4
#define PUSHBUTTON_S2_BIT                       (0x01 << PUSHBUTTON_S2_PIN)
#define PUSHBUTTON_S2_PORT                      P1
#define SET_PUSHBUTTON_S2_TO_AN_INPUT           PUSHBUTTON_S2_PORT->DIR &= ~PUSHBUTTON_S2_BIT
#define ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S2   PUSHBUTTON_S2_PORT->REN |= PUSHBUTTON_S2_BIT
#define SELECT_PULL_UP_RESISTORS_S2             PUSHBUTTON_S2_PORT->OUT |= PUSHBUTTON_S2_BIT

#endif /* PORTPINS_H_ */
