#ifndef PORTPINS_H_
#define PORTPINS_H_

#define NVIC_INTERRUPT_ENABLE(REG,BIT)      NVIC->ISER[REG] = (0x00000001 << BIT)
#define NVIC_INTERRUPT_DISABLE(REG,BIT)     NVIC->ICER[REG] = (0x00000001 << BIT)
#define PRIORITY_REGISTER(REG,MASK,PR)      NVIC->IP[REG] = (NVIC->IP[REG]&MASK)|PR
#define ADC14_CONVERSION_START_ADDR(REG)    (uint32_t)((REG<<ADC14_CTL1_CSTARTADD_OFS) & ADC14_CTL1_CSTARTADD_MASK)

//  LED1
#define LED1_PIN                    0
#define LED1_BIT                    (0x01 << LED1_PIN)
#define LED1_PORT                   P1
#define SET_LED1_AS_AN_OUTPUT       LED1_PORT->DIR |= LED1_BIT
#define TURN_ON_LED1                LED1_PORT->OUT |= LED1_BIT
#define TURN_OFF_LED1               LED1_PORT->OUT &= ~LED1_BIT
#define TOGGLE_LED1                 LED1_PORT->OUT ^= LED1_BIT

//  LED2 - Red
#define LED2_R_PIN                  0
#define LED2_R_BIT                  (0x01 << LED2_R_PIN)
#define LED2_R_PORT                 P2
#define SET_LED2_R_AS_AN_OUTPUT     LED2_R_PORT->DIR |= LED2_R_BIT
#define TURN_ON_LED2_R              LED2_R_PORT->OUT |= LED2_R_BIT
#define TURN_OFF_LED2_R             LED2_R_PORT->OUT &= ~LED2_R_BIT
#define TOGGLE_LED2_R               LED2_R_PORT->OUT ^= LED2_R_BIT

//  LED2 - Green
#define LED2_G_PIN                  1
#define LED2_G_BIT                  (0x01 << LED2_G_PIN)
#define LED2_G_PORT                 P2
#define SET_LED2_G_AS_AN_OUTPUT     LED2_G_PORT->DIR |= LED2_G_BIT
#define TURN_ON_LED2_G              LED2_G_PORT->OUT |= LED2_G_BIT
#define TURN_OFF_LED2_G             LED2_G_PORT->OUT &= ~LED2_G_BIT
#define TOGGLE_LED2_G               LED2_G_PORT->OUT ^= LED2_G_BIT

//  LED2 - Blue
#define LED2_B_PIN                  2
#define LED2_B_BIT                  (0x01 << LED2_B_PIN)
#define LED2_B_PORT                 P2
#define SET_LED2_B_AS_AN_OUTPUT     LED2_B_PORT->DIR |= LED2_B_BIT
#define TURN_ON_LED2_B              LED2_B_PORT->OUT |= LED2_B_BIT
#define TURN_OFF_LED2_B             LED2_B_PORT->OUT &= ~LED2_B_BIT
#define TOGGLE_LED2_B               LED2_B_PORT->OUT ^= LED2_B_BIT

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
