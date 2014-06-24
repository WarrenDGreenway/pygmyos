/**************************************************************************
    PygmyOS ( Pygmy Operating System )
    Copyright (C) 2011-2014  Warren D Greenway

    This file is part of PygmyOS.

    PygmyOS is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PygmyOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with PygmyOS.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
/**************************************************************************
This file implements a standard/mandatory set of functions and macros to
provide a Hardware Abstraction Layer (HAL) for an MCU. 
***************************************************************************/
//#include <iostream>
//#include <vector>
#include "pygmy_type.h"
#include "pygmy_uart.h"
//#include "pygmy_com.h"
#include "stm32.h"

using namespace std;

/*************************************************************************
                          Private HAL Variables
***************************************************************************/
#define PYGMY_PWMCHANNELS 10 // replace this with a vector or collection
PYGMYPWM globalPygmyPWM[ PYGMY_PWMCHANNELS ];
PYGMYVOIDPTR globalIRQHandlers[ 16 ];
u16 globalIRQPR; 
u8 globalIRQPins[ 16 ];
const u8 PYGMY_TIMERVECTORS[] = {      
    0,          TIM1_UP_IRQ,    TIM2_IRQ,       TIM3_IRQ,       TIM4_IRQ,       TIM5_IRQ, 
    TIM6_IRQ,   TIM7_IRQ,       TIM8_UP_IRQ,    TIM9_IRQ,       TIM10_IRQ,      TIM11_IRQ, 
    TIM12_IRQ,  TIM13_IRQ,      TIM14_IRQ,      TIM15_IRQ,      TIM16_IRQ,      TIM17_IRQ 
};

const u8 PYGMY_TIMERVECTORS_L15X[] = { 
    0,          0,              TIM2_IRQ,       TIM3_IRQ,       TIM4_IRQ,       0, 
    TIM6_IRQ,   TIM7_IRQ,       0,              TIM9_L15X_IRQ,  TIM10_L15X_IRQ, TIM11_L15X_IRQ,
    0,          0,              0,              0,              0,              0
};

const TIMER *PYGMY_TIMERS[] = { (TIMER*)NULL,   (TIMER*)NULL,   (TIMER*)TIM2,   (TIMER*)TIM3,   (TIMER*)TIM4,   (TIMER*)TIM5, 
                                (TIMER*)TIM6,   (TIMER*)TIM7,   (TIMER*)TIM8,   (TIMER*)TIM9,   (TIMER*)TIM10,  (TIMER*)TIM11, 
                                (TIMER*)TIM12,  (TIMER*)TIM13,  (TIMER*)TIM14,  (TIMER*)TIM15,  (TIMER*)TIM16,  (TIMER*)TIM17 };

const u8 PYGMY_TIMERPINS[] = {  PA8,  PA9,  PA10, PA11,     // TIM1
                                PA0,  PA1,  PA2,  PA3,      // TIM2
                                PA6,  PA7,  PB0,  PB1,      // TIM3
                                PB6,  PB7,  PB8,  PB9  };   // TIM4
//vector<PWMChannel>globalPygmySoftPWM;
u32 globalPygmyPWMFreq, globalPygmyPWMCR = 0;
u8 MCU_PWMTimer, MCU_DelayTimer;

/*PYGMY_RXHANDLERPTR usart1RXHandler = 0;
PYGMY_TXHANDLERPTR usart1TXHandler = 0;
PYGMY_RXHANDLERPTR usart2RXHandler = 0;
PYGMY_TXHANDLERPTR usart2TXHandler = 0;
PYGMY_RXHANDLERPTR usart3RXHandler = 0;
PYGMY_TXHANDLERPTR usart3TXHandler = 0;
PYGMY_RXHANDLERPTR usart4RXHandler = 0;
PYGMY_TXHANDLERPTR usart4TXHandler = 0;
PYGMY_RXHANDLERPTR usart5RXHandler = 0;
PYGMY_TXHANDLERPTR usart5TXHandler = 0;
PYGMY_RXHANDLERPTR usart6RXHandler = 0;
PYGMY_TXHANDLERPTR usart6TXHandler = 0;
PYGMY_RXHANDLERPTR usart7RXHandler = 0;
PYGMY_TXHANDLERPTR usart7TXHandler = 0;
PYGMY_RXHANDLERPTR usart8RXHandler = 0;
PYGMY_TXHANDLERPTR usart8TXHandler = 0;*/
//void (*usart1RXHandler)( Uart&, char ) = 0;
//char (*usart1TXHandler)( Uart& ) = 0;
void (*usart1RXHandler)( void*, char ) = 0;
char (*usart1TXHandler)( void* ) = 0;
void (*usart2RXHandler)( void*, char ) = 0;
char (*usart2TXHandler)( void* ) = 0;
void (*usart3RXHandler)( void*, char ) = 0;
char (*usart3TXHandler)( void* ) = 0;
void (*usart4RXHandler)( void*, char ) = 0;
char (*usart4TXHandler)( void* ) = 0;
void (*usart5RXHandler)( void*, char ) = 0;
char (*usart5TXHandler)( void* ) = 0;
void (*usart6RXHandler)( void*, char ) = 0;
char (*usart6TXHandler)( void* ) = 0;
void (*usart7RXHandler)( void*, char ) = 0;
char (*usart7TXHandler)( void* ) = 0;
void (*usart8RXHandler)( void*, char ) = 0;
char (*usart8TXHandler)( void* ) = 0;
void *com1;
void *com2;
void *com3;
void *com4;
void *com5;
void *com6;
void *com7;
void *com8;
/*
Uart *com1;
Uart *com2;
Uart *com3;
Uart *com4;
Uart *com5;
Uart *com6;
Uart *com7;
Uart *com8;*/

/**************************************************************************
                            Public Functions
***************************************************************************/
u32 mainClockFrequency;

u8 interruptGetTimerVector( u8 ucTimer )
{
    //ToDo: Get the MCU ID from the MCU directly
    // return value of 0 means no timer present
    /*if( descriptorGetID() == DESC_STM32L151 ){
        return( PYGMY_TIMERVECTORS_L15X[ ucTimer ] );
    } else{*/
        return( PYGMY_TIMERVECTORS[ ucTimer ] );
    //} // else
    
}

void interruptEnable( u32 ulVector )
{
    // The vectors are arranged 1 bit per vector in 32 groups
    // The vector number must be decoded, first for which group, second the bit to set
    if( ulVector > 63 ){
        NVIC->ISER[ 2 ] = 0x00000001 << ( ulVector - 64 );
    } else if( ulVector > 31 ){
        NVIC->ISER[ 1 ] = 0x00000001 << ( ulVector - 32 );
    } else{
        NVIC->ISER[ 0 ] = 0x00000001 << ulVector;
    }
}

void interruptDisable( u32 ulVector )
{
    // The vectors are arranged 1 bit per vector in 32 groups
    // The vector number must be decoded, first for which group, second the bit to set
    // Notice! Bits set in the ICER registers actually clear the interrupt vectors
    if( ulVector > 63 ){
        NVIC->ICER[ 2 ] = 0x00000001 << ( ulVector - 64 );
    } else if( ulVector > 31 ){
        NVIC->ICER[ 1 ] = 0x00000001 << ( ulVector - 32 );
    } else{
        NVIC->ICER[ 0 ] = 0x00000001 << ulVector;
    }
}

void interruptSetPriority( u32 ulVector, u8 ucPriority )
{
    // IPR is a register array of up to 20 32 bit d-words
    // Each register contains 4 8 bit priorities which are linearly mapped
    // To a corresponding Vector. Only the High nibble of each 8 bit group
    // Is used to represent the priority from 0 - 15
    // Note that some of the Vectors are dealt with in System Registers
    // Those Vectors are all start at 0xF0 according to the system to allow
    // Decoding, they are separated and handled individually below
    if( ulVector > 0xEF ){
        switch( ulVector ){
            case MEMFAULT_IRQ:
                SCB->SHPR[0] &= 0xFFFFFF0F;               // Clear nibble
                SCB->SHPR[0] |=  ( u32 ) ucPriority ;     // Write Priority
            break;
            case BUSFAULT_IRQ:      
                SCB->SHPR[0] &= 0xFFFF0FFF;               // Clear nibble
                SCB->SHPR[0] |= ( u32 ) ucPriority << 8;  // Write Priority
            break;
            case USAGEFAULT_IRQ:
                SCB->SHPR[0] &= 0xFF0FFFFF;               // Clear nibble
                SCB->SHPR[0] |= ( u32 ) ucPriority << 16; // Write Priority
            break;
            case SVCALL_IRQ:
                SCB->SHPR[1] &= 0x0FFFFFFF;               // Clear nibble
                SCB->SHPR[1] |= ( u32 ) ucPriority << 24; // Write Priority
            break;
            case PENDSVCALL_IRQ:
                SCB->SHPR[1] &= 0xFF0FFFFF;               // Clear nibble
                SCB->SHPR[1] |= ( u32 ) ucPriority << 16; // Write Priority
            break;
            case SYSTICK_IRQ:
                SCB->SHPR[2] &= 0x0FFFFFFF;               // Clear nibble
                SCB->SHPR[2] |= ( u32 ) ucPriority << 24; // Write Priority
            break;
        }
    } else{
        NVIC->IPR[ ulVector / 4 ] &= ~( ( u32 )0xF0 ) << ( ( ulVector % 4 ) * 8 );      // Clear nibble
        NVIC->IPR[ ulVector / 4 ] |= ( ( u32 )ucPriority ) << ( ( ulVector % 4 ) * 8 ); // Write Priority
    } 
}

u32 cpuGetMainClock( void )
{
    // return current system clock
    // todo: add real clock source with clock change hook support
    return( mainClockFrequency );
}

void cpuSetMainClock( u32 freq )
{
    // set current system main clock
    // todo: add real clock source with clock change hook support
    mainClockFrequency = freq;
}

void *getTimer( u8 ucTimer )
{
    // This function converts a numeric Timer representation to a
    // pointer to Timer structure
    switch( ucTimer ){
        case PYGMY_TIMER1:
            return( TIM1 );
        case PYGMY_TIMER2:
            return( TIM2 );
        case PYGMY_TIMER3:
            return( TIM3 );
        case PYGMY_TIMER4:
            return( TIM4 );
        case PYGMY_TIMER5:
            return( TIM5 );
        case PYGMY_TIMER6:
            return( TIM6 );
        case PYGMY_TIMER7:
            return( TIM7 );
        case PYGMY_TIMER8:
            return( TIM8 );
        case PYGMY_TIMER9:
            return( TIM9 );
        case PYGMY_TIMER10:
            return( TIM10 );
        case PYGMY_TIMER11:
            return( TIM11 );
        case PYGMY_TIMER12:
            return( TIM12 );
        case PYGMY_TIMER13:
            return( TIM13 );
        case PYGMY_TIMER14:
            return( TIM14 );
        case PYGMY_TIMER15:
            return( TIM15 );
        case PYGMY_TIMER16:
            return( TIM16 );
        case PYGMY_TIMER17:
            return( TIM17 );
        default:
            return( NULL );
    } // switch
}

void enableTimerClock( u8 ucTimer )
{
    if( ucTimer == PYGMY_TIMER1 ){
        PYGMY_RCC_TIMER1_ENABLE;
    } else if( ucTimer == PYGMY_TIMER2 ){
        PYGMY_RCC_TIMER2_ENABLE;
    } else if( ucTimer == PYGMY_TIMER3 ){
        PYGMY_RCC_TIMER3_ENABLE;
    } else if( ucTimer == PYGMY_TIMER4 ){
        PYGMY_RCC_TIMER4_ENABLE;
    } else if( ucTimer == PYGMY_TIMER5 ){
        PYGMY_RCC_TIMER5_ENABLE;
    } else if( ucTimer == PYGMY_TIMER6 ){
        PYGMY_RCC_TIMER6_ENABLE;
    } else if( ucTimer == PYGMY_TIMER7 ){
        PYGMY_RCC_TIMER7_ENABLE;
    } else if( ucTimer == PYGMY_TIMER8 ){
        PYGMY_RCC_TIMER8_ENABLE;
    } else if( ucTimer == PYGMY_TIMER9 ){
        PYGMY_RCC_TIMER9_ENABLE;
    } else if( ucTimer == PYGMY_TIMER10 ){
        PYGMY_RCC_TIMER10_ENABLE;
    } else if( ucTimer == PYGMY_TIMER11 ){
        PYGMY_RCC_TIMER11_ENABLE;
    } else if( ucTimer == PYGMY_TIMER12 ){
        PYGMY_RCC_TIMER12_ENABLE;
    } else if( ucTimer == PYGMY_TIMER13 ){
        PYGMY_RCC_TIMER13_ENABLE;
    } else if( ucTimer == PYGMY_TIMER14 ){
        PYGMY_RCC_TIMER14_ENABLE;
    } else if( ucTimer == PYGMY_TIMER15 ){
        PYGMY_RCC_TIMER15_ENABLE;
    } else if( ucTimer == PYGMY_TIMER16 ){
        PYGMY_RCC_TIMER16_ENABLE;
    } else if( ucTimer == PYGMY_TIMER17 ){
        PYGMY_RCC_TIMER17_ENABLE;
    } // else if
}

void disableTimerClock( u8 ucTimer )
{
    if( ucTimer == PYGMY_TIMER1 ){
        PYGMY_RCC_TIMER1_DISABLE;
    } else if( ucTimer == PYGMY_TIMER2 ){
        PYGMY_RCC_TIMER2_DISABLE;
    } else if( ucTimer == PYGMY_TIMER3 ){
        PYGMY_RCC_TIMER3_DISABLE;
    } else if( ucTimer == PYGMY_TIMER4 ){
        PYGMY_RCC_TIMER4_DISABLE;
    } else if( ucTimer == PYGMY_TIMER5 ){
        PYGMY_RCC_TIMER5_DISABLE;
    } else if( ucTimer == PYGMY_TIMER6 ){
        PYGMY_RCC_TIMER6_DISABLE;
    } else if( ucTimer == PYGMY_TIMER7 ){
        PYGMY_RCC_TIMER7_DISABLE;
    } else if( ucTimer == PYGMY_TIMER8 ){
        PYGMY_RCC_TIMER8_DISABLE;
    } else if( ucTimer == PYGMY_TIMER9 ){
        PYGMY_RCC_TIMER9_DISABLE;
    } else if( ucTimer == PYGMY_TIMER10 ){
        PYGMY_RCC_TIMER10_DISABLE;
    } else if( ucTimer == PYGMY_TIMER11 ){
        PYGMY_RCC_TIMER11_DISABLE;
    } else if( ucTimer == PYGMY_TIMER12 ){
        PYGMY_RCC_TIMER12_DISABLE;
    } else if( ucTimer == PYGMY_TIMER13 ){
        PYGMY_RCC_TIMER13_DISABLE;
    } else if( ucTimer == PYGMY_TIMER14 ){
        PYGMY_RCC_TIMER14_DISABLE;
    } else if( ucTimer == PYGMY_TIMER15 ){
        PYGMY_RCC_TIMER15_DISABLE;
    } else if( ucTimer == PYGMY_TIMER16 ){
        PYGMY_RCC_TIMER16_DISABLE;
    } else if( ucTimer == PYGMY_TIMER17 ){
        PYGMY_RCC_TIMER17_DISABLE;
    } // else if
}

u8 enableTimerInterrupt( u8 ucTimer )
{
    u8 ucIRQ;
    
    ucIRQ = interruptGetTimerVector( ucTimer );
    if( ucIRQ ){
        interruptEnable( ucIRQ );
        return( 1 );
    } // if
    
    return( 0 ); // Invalid timer selected
}

void mcoEnable( u8 ucSource )
{ 
    const u32 PYGMY_MCO[] = { RCC_MCO_SYSCLOCK, RCC_MCO_INTERNALRC, RCC_MCO_EXTCLOCK, RCC_MCO_PLLDIV2 };

    if( ucSource > 3 ){
        return;
    } // if
    PYGMY_RCC_GPIOA_ENABLE;
    GPIOA->CRH &= ~PIN8_CLEAR;
    GPIOA->CRH |= PIN8_OUT50_ALTPUSHPULL;
    RCC->CFGR &= ~RCC_MCO_CLEAR;
    RCC->CFGR |= PYGMY_MCO[ ucSource ];                 
}

u8 pinConfig( u16 ucPin, u8 ucMode )
{
    u32 uiPortMode, uiPortClear;
    u16 uiPin;
    GPIO *pygmyPort;

    // Test pin for error value (0xFF) or invalid mode before continuing 
    if( ucPin == 0xFF || !( ucMode == OUT || ucMode == IN || ucMode == ANALOG || 
        ucMode == PULLUP || ucMode == PULLDOWN || ucMode == ALT  )){
        return( FALSE );
    } // if
    // Port registers are broken into 8 pins each in the STM32
    // CRL contains the config bits for 0-7
    // CRH contains the config bits for 8-15
    // each bit is configured with 4 bits
    pinEnablePortClock( ucPin ); // Enable clock BEFORE accessing registers
    pygmyPort = pinGetPort( ucPin );
    uiPortMode = ( PIN_CLEAR & ucMode) << ( ( ucPin % 8 ) * 4 );
    uiPortClear = ~( PIN_CLEAR << ( ( ucPin % 8 ) * 4 ) ); // Clear before setting bits
    uiPin = ucPin % 16;
    if( uiPin < 8 ){
        pygmyPort->CRL &= uiPortClear;
        pygmyPort->CRL |= uiPortMode;
    } else{
        pygmyPort->CRH &= uiPortClear;
        pygmyPort->CRH |= uiPortMode;
    } // else
    
    if( ucMode & PULLUP ){
        pinSet( ucPin, HIGH );
    } else if( ucMode & PULLDOWN ){
        pinSet( ucPin, LOW );
    } // else if
}

/*void pinConfig( u16 pin, u8 mode )
{
    u8 pullUpDown;
    u16 pinModulo;
    u32 portMode, portClear;
    GPIO *pygmyPort;

    // Test pin for error value (0xFF) or invalid mode before continuing 
    if( pin == 0xFFFF || !( mode == OUT || mode == IN || mode == ANALOG || 
        mode == PULLUP || mode == PULLDOWN || mode == ALT  )){
        return;
    } // if
    // Port registers are broken into 8 pins each in the STM32
    // CRL contains the config bits for 0-7
    // CRH contains the config bits for 8-15
    // each bit is configured with 4 bits
    pinEnablePortClock( pin ); // Enable clock BEFORE accessing registers
    pullUpDown = mode & ( PIN_PULLUP | PIN_PULLDOWN );
    mode &= ~( PIN_PULLUP | PIN_PULLDOWN );
    pygmyPort = pinGetPort( pin );
    pinModulo = pin % 8;
    portMode = mode << ( pinModulo * 4 );
    portClear = ~( PIN_CLEAR << ( pinModulo * 4 ) ); // Clear before setting bits
    if( pinModulo < 8 ){
        pygmyPort->CRL &= portClear;
        pygmyPort->CRL |= portMode;
    } else{
        pygmyPort->CRH &= portClear;
        pygmyPort->CRH |= portMode;
    } // else
    
    if( pullUpDown & PIN_PULLUP ){
        pinSet( pin, HIGH );
    } else if( mode & PIN_PULLDOWN ){
        pinSet( pin, LOW );
    } // else if
}*/

void pinSet( u16 pin, bool state )
{
    // Return value indicates TRUE for success and FALSE for failure

    u16 pinMask, pinState;

    if( pin == 0xFF ){
        // The stm32 family doesn't support pins outside the 8 bit range
        return;
    } // if
    pinMask = BIT0 << ( pin % 16 );
    pinState = 0;
    if( state ){
        pinState = pinMask;
    } // if
    if( pin < 16 ){
        GPIOA->ODR &= ~pinMask;
        GPIOA->ODR |= pinState;
    } else if( pin < 32 ){
        GPIOB->ODR &= ~pinMask;
        GPIOB->ODR |= pinState; 
    } else if( pin < 48 ){
        GPIOC->ODR &= ~pinMask;
        GPIOC->ODR |= pinState;
    } else if( pin < 64 ){
        GPIOD->ODR &= ~pinMask;
        GPIOD->ODR |= pinState;
    } else if( pin < 80 ){
        GPIOE->ODR &= ~pinMask;
        GPIOE->ODR |= pinState;
    }
    #ifdef GPIOF
        if( ucPin < 96 ){
            GPIOF->ODR &= ~pinMask;
            GPIOF->ODR |= pinState;
        } // else if
    #endif
}

bool pinGet( u16 pin )
{
    // Return the value of the specified pin in a generic way
    u16 pinMask;
    
    pinMask = BIT0 << ( pin % 16 );
    if( pin < 16 ){
        pinMask &= GPIOA->IDR;
    } else if( pin < 32 ){
        pinMask &= GPIOB->IDR;
    } else if( pin < 48 ){
        pinMask &= GPIOC->IDR;
    } else if( pin < 64 ){
        pinMask &= GPIOD->IDR;
    } else if( pin < 80 ){
        pinMask &= GPIOE->IDR;
    } // else if 
    #ifdef GPIOF
        if( pin < 96 ){
            pinMask &= GPIOF->IDR;
        } // else if
    #endif
    
    if( pinMask ){
        return( true );
    } else{
        return( false );
    } // else
}

u8 getDelayTimer( void )
{
    // private function
    return( MCU_DelayTimer );
}

u8 getPWMTimer( void )
{
    // private function
    return( MCU_PWMTimer );
}


bool pinPWM( u16 pin, u32 frequency, u8 dutyCycle )
{
    // This function will first attempt to find a timer and timer channel to match the selected pin
    // If the pin isn't attached to a hardware timer, then a software timer will be set up.
    // Since this function is part of the public HAL, it is important that the calling function needn't
    //   need to know about the hardware specifics.
    TIMER *pygmyTimer;
    TIMER1 *pygmyTimer1;
    u32 duty, mainClock;
    u8 channel;

    // ToDo: Add clock frequency change hooks 
    if( dutyCycle > 100 || getPWMTimer() == PYGMY_TIMER0 ){
        return( false );
    } // if
    pygmyTimer = (TIMER*)pinGetTimer( pin );
    pygmyTimer1 = (TIMER1*)pinGetTimer( pin );
    channel = pinGetChannel( pin );
    if( !channel || !pygmyTimer || getDelayTimer() == PYGMY_TIMER1 ){
        // Timer1 is unavailable for hardware PWM on all F103s except the F103XLD
        // pin selected isn't a timer pin, call softPWM
        return( pinAddSoftPWM( pin, frequency, dutyCycle ) );
    } // if
    if( frequency == 0 ){
        frequency = 1;
    } // if
    mainClock = cpuGetMainClock();
    if( frequency > mainClock ){
        return( false );
    } // if
    frequency = mainClock / frequency;
    duty = ( frequency / 100 ) * dutyCycle;
       
    pinConfig( pin, ALT );
    if( pygmyTimer1 == TIM1 ){
        PYGMY_RCC_TIMER1_ENABLE;
        interruptEnable( TIM1_UP_IRQ );
        pygmyTimer1->CR1 = 0;            // Disable before configuring timer
        pygmyTimer1->CR2 = 0;                        //
        pygmyTimer1->BDTR |= (TIM_MOE|TIM_AOE);
        pygmyTimer1->CNT = 0;                        // Count Register
        pygmyTimer1->ARR = frequency;                   // Auto Reload Register
        pygmyTimer1->PSC = 0;                        // Prescaler
        if( channel == 1 ){   
            pygmyTimer1->CCR1 = duty;              // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer1->CCMR1 &= ~(TIM_OC1M_CLEAR|TIM_OC1PE);
            pygmyTimer1->CCMR1 |= (TIM_OC1M_PWM1|TIM_OC1PE);          // 
            pygmyTimer1->CCER |= TIM_CC1E;
        } else if( channel == 2 ){
            pygmyTimer1->CCR2 = duty;              // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer1->CCMR1 &= ~(TIM_OC2M_CLEAR|TIM_OC2PE);
            pygmyTimer1->CCMR1 |= (TIM_OC2M_PWM1|TIM_OC2PE);          // 
            pygmyTimer1->CCER |= TIM_CC2E;
        } else if( channel == 3 ){
            pygmyTimer1->CCR3 = duty;              // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer1->CCMR2 &= ~(TIM_OC3M_CLEAR|TIM_OC3PE);
            pygmyTimer1->CCMR2 |= (TIM_OC3M_PWM1|TIM_OC3PE);          // 
            pygmyTimer1->CCER |= TIM_CC3E;
        } else if( channel == 4 ){
            pygmyTimer1->CCR4 = duty;              // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer1->CCMR2 &= ~(TIM_OC4M_CLEAR|TIM_OC4PE);
            pygmyTimer1->CCMR2 |= (TIM_OC4M_PWM1|TIM_OC4PE);          // 
            pygmyTimer1->CCER |= TIM_CC4E;
        } // else if
        pygmyTimer1->EGR |= TIM_UG;
        pygmyTimer1->CR1 = TIM_ARPE|TIM_CEN;
    } else{
        PYGMY_RCC_TIMER2_ENABLE;
        PYGMY_RCC_TIMER3_ENABLE;
        PYGMY_RCC_TIMER4_ENABLE;
        pygmyTimer->CR1 = 0;//&= ~( TIM_CEN );            // Disable before configuring timer
        pygmyTimer->CR2 = 0;                        //
        pygmyTimer->CNT = 0;                        // Count Register
        pygmyTimer->ARR = frequency;                   // Auto Reload Register
        pygmyTimer->PSC = 0;                        // Prescaler
        if( channel == 1 ){
            pygmyTimer->CCR1 = duty;              // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer->CCMR1 &= ~(TIM_OC1M_CLEAR|TIM_OC1PE);
            pygmyTimer->CCMR1 |= (TIM_OC1M_PWM1|TIM_OC1PE);          // 
            pygmyTimer->CCER |= TIM_CC1E;
        } else if( channel == 2 ){
            pygmyTimer->CCR2 = duty;              // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer->CCMR1 &= ~(TIM_OC2M_CLEAR|TIM_OC2PE);
            pygmyTimer->CCMR1 |= (TIM_OC2M_PWM1|TIM_OC2PE);          // 
            pygmyTimer->CCER |= TIM_CC2E;
        } else if( channel == 3 ){
            pygmyTimer->CCR3 = duty;      // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer->CCMR2 &= ~(TIM_OC3M_CLEAR|TIM_OC3PE);          // 
            pygmyTimer->CCMR2 |= (TIM_OC3M_PWM1|TIM_OC3PE);
            pygmyTimer->CCER |= TIM_CC3E;
        } else if( channel == 4 ){
            pygmyTimer->CCR4 = duty;      // Capture Compare for PWM in output, sets duty cycle
            pygmyTimer->CCMR2 &= ~(TIM_OC4M_CLEAR|TIM_OC4PE);          // 
            pygmyTimer->CCMR2 |= (TIM_OC4M_PWM1|TIM_OC4PE);
            pygmyTimer->CCER |= TIM_CC4E;
        } // else if
        pygmyTimer->EGR |= TIM_UG;
        pygmyTimer->CR1 = TIM_ARPE|TIM_CEN;
    } // else
    
    return( true );
}

u8 pinPulseHigh( u8 ucPin, u32 ulDeadTime, u32 ulPulse )
{
    u16 i;
    
    // This function's timing is based on SoftPWM Frequency
    // If frequency is set to 10000, then count is decremented 10000 per second
    // First find empty PWM register slot
    if( globalPygmyPWMCR == 0 ){
        pinInitSoftPWM( );
    } // if
    for( i = 0; i < PYGMY_PWMCHANNELS; i++ ){
        if( globalPygmyPWM[ i ].CR == 0 || globalPygmyPWM[ i ].Pin == ucPin ){
            globalPygmyPWM[ i ].CR = 0; // if pin already exists, CR must be cleared 
            break;
        } // if 
    } // for
    if( i == PYGMY_PWMCHANNELS ){
        // No open slots
        return( 0 );
    } // if

    pinConfig( ucPin, OUT );
    pinSet( ucPin, LOW );
    globalPygmyPWM[ i ].UpCount = ulPulse;
    globalPygmyPWM[ i ].DownCount = ulDeadTime;
    globalPygmyPWM[ i ].Count = globalPygmyPWM[ i ].DownCount; 
    globalPygmyPWM[ i ].Pin = ucPin;
    globalPygmyPWM[ i ].CR = ( PYGMY_PWM_DIR | PYGMY_PWM_EN );
    
    return( 1 );
}

u8 pinToggle( u8 ucPin, u32 ulFreq )
{
    // This function uses the PWM time base frequency
    // Max toggle speed is PWM frequency / 2
    return( pinAddSoftPWM( ucPin, ulFreq, 50 ) );
}

u8 pinPulseLow( u8 ucPin, u32 ulDeadTime, u32 ulPulse )
{
    u16 i;
    
    // This function's timing is based on SoftPWM Frequency
    // If frequency is set to 10000, then count is decremented 10000 per second
    // First find empty PWM register slot
    if( globalPygmyPWMCR == 0 ){
        pinInitSoftPWM( );
    } // if
    for( i = 0; i < PYGMY_PWMCHANNELS; i++ ){
        if( globalPygmyPWM[ i ].CR == 0 || globalPygmyPWM[ i ].Pin == ucPin ){
            globalPygmyPWM[ i ].CR = 0; // if pin already exists, CR must be cleared 
            break;
        } // if 
    } // for
    if( i == PYGMY_PWMCHANNELS ){
        // No open slots
        return( 0 );
    } // if

    pinConfig( ucPin, OUT );
    pinSet( ucPin, HIGH );
    globalPygmyPWM[ i ].UpCount = ulPulse;
    globalPygmyPWM[ i ].DownCount = ulDeadTime;
    globalPygmyPWM[ i ].Count = globalPygmyPWM[ i ].UpCount; 
    globalPygmyPWM[ i ].Pin = ucPin;
    globalPygmyPWM[ i ].CR = PYGMY_PWM_EN;
    
    return( 1 );
}




/**************************************************************************
                            Private Functions
             Do not call these functions outside this source
***************************************************************************/
GPIO *pinGetPort( u16 pin )
{
    if( pin < 16 ){
        return( GPIOA );
    } else if( pin < 32 ){
        return( GPIOB );
    } else if( pin < 48 ){
        return( GPIOC );
    } else if( pin < 64 ){
        return( GPIOD );
    } else if( pin < 80 ){
        return( GPIOE );
    } // else if
        
    #ifdef GPIOF
        return( GPIOF );
    #endif
    return( 0 );
}

void pinEnablePortClock( u16 pin )
{
    if( pin < 16 ){
        PYGMY_RCC_GPIOA_ENABLE;
    } else if( pin < 32 ){
        PYGMY_RCC_GPIOB_ENABLE;
    } else if( pin < 48 ){
        PYGMY_RCC_GPIOC_ENABLE;
    } else if( pin < 64 ){
        PYGMY_RCC_GPIOD_ENABLE;
    } else if( pin < 80 ){
        PYGMY_RCC_GPIOE_ENABLE;
    } else if( pin < 96 ){
        PYGMY_RCC_GPIOF_ENABLE;
    }
}

void *pinGetTimer( u16 pin )
{
    // This function identifies the hardware timer associated with pin
    u16 i, ii;
    u8 *timerPins;
    
    timerPins = (u8*)PYGMY_TIMERPINS;
    for( i = 0; i < 4; i++ ){
        for( ii = 0; ii < 4; ii ++ ){
            if( *(timerPins + ( i * 4 ) + ii ) == pin ){
                if( i == 0 ){
                    return( TIM1 );
                } else if( i == 1 ){
                    return( TIM2 );
                } else if( i == 2 ){
                    return( TIM3 );
                } else if( i == 3 ){
                    return( TIM4 );
                } else{
                    return( NULL );
                }
            } // if
        } // for
    } // for
}

u8 pinGetChannel( u16 pin )
{
    // This function identifies the hardware timer channel associated with pin
    u8 i, ii, *timerPins;
    
    timerPins = (u8*)PYGMY_TIMERPINS;
    for( i = 0; i < 4; i++ ){
        for( ii = 0; ii < 4; ii ++ ){
            if( *(timerPins + ( i * 4 ) + ii ) == pin ){
                return( 1 + ii );
            } // if
        } // for
    } // for
    
    return( 0 ); // 0 is an invalid channel
}

void pinSetSoftPWMFreq( u32 ulFreq )
{
    globalPygmyPWMFreq = ulFreq;
}

u32 pinGetSoftPWMFreq( void )
{
    return( globalPygmyPWMFreq );
}

void pinInitSoftPWM( void )
{
    TIMER *pygmyTimer;
    u16 i;
    u8 ucTimer;
    
    #ifdef __PYGMYSOFTPWMFREQ
        globalPygmyPWMFreq = __PYGMYSOFTPWMFREQ;
    #else
        globalPygmyPWMFreq = 8000;
    #endif
    for( i = 0; i < PYGMY_PWMCHANNELS; i++ ){
        globalPygmyPWM[ i ].CR = 0;
    } // for
    ucTimer = getPWMTimer();
    if( ucTimer < 2 ){
        return; // Timer1 cannot be used for SoftPWM
    } // if
    
    pygmyTimer = (TIMER*)getTimer( ucTimer );
    
    enableTimerClock( ucTimer );
    enableTimerInterrupt( ucTimer );
    //PYGMY_RCC_TIMER16_ENABLE;
    //interruptEnable( TIM16_IRQ );
    pygmyTimer->CR1     = 0;                            // Disable before configuring timer
    pygmyTimer->CR2     = 0;                            //
    pygmyTimer->SMCR    = 0;                            //
    pygmyTimer->PSC     = 0;                            // 
    pygmyTimer->CNT     = 0;                            // Count Register
    pygmyTimer->DIER    = TIM_UIE;                      // DMA and interrupt enable register
    pygmyTimer->ARR     = ( cpuGetMainClock() / globalPygmyPWMFreq ) & 0x0000FFFF;  // Auto Reload Register
    pygmyTimer->SR      = 0;                            //
    pygmyTimer->CR1     = ( TIM_ARPE | TIM_CEN );       // Enable Auto Reload and Counter
}

u8 pinRemoveSoftPWM( u8 ucPin )
{
    u16 i;

    for( i = 0; i < PYGMY_PWMCHANNELS; i++ ){
        if( globalPygmyPWM[ i ].Pin == ucPin ){
            globalPygmyPWM[ i ].CR = 0;
            return( 1 );
        } // if
    } // for  

    return( 0 );
}

u8 pinAddSoftPWM( u8 ucPin, u32 ulFreq, u8 ucDutyCycle )
{
    u16 i;

    // First find empty PWM register slot
    if( globalPygmyPWMCR == 0 ){
        pinInitSoftPWM( );
    } // if
    for( i = 0; i < PYGMY_PWMCHANNELS; i++ ){
        if( globalPygmyPWM[ i ].CR == 0 || globalPygmyPWM[ i ].Pin == ucPin ){
            globalPygmyPWM[ i ].CR = 0; // if pin already exists, CR must be cleared 
            break;
        } // if 
    } // for
    if( i == PYGMY_PWMCHANNELS ){
        // No open slots
        return( 0 );
    } // if
    if ( ucDutyCycle > 100 || !ulFreq ){ 
        // Out of Range
        return( 0 );
    } // if

    pinConfig( ucPin, OUT );
    globalPygmyPWM[ i ].UpCount = 0;
    globalPygmyPWM[ i ].DownCount = 0;
    if( ucDutyCycle == 0 ){
        // 0% duty cycle is always LOW, no need to use counters
        pinSet( ucPin, LOW );
    } else if( ucDutyCycle == 100 ){
        // 100% duty cycle is always HIGH, no need to use counters
        pinSet( ucPin, HIGH );
    } else{
        globalPygmyPWM[ i ].UpCount = globalPygmyPWMFreq / ( ulFreq * ( 100 / ucDutyCycle ) );
        globalPygmyPWM[ i ].DownCount = globalPygmyPWMFreq / ( ulFreq * ( 100 / ( 100 - ucDutyCycle ) ) );
        if( ( globalPygmyPWM[ i ].UpCount + globalPygmyPWM[ i ].DownCount ) > globalPygmyPWMFreq ){
            // main counter has insufficient resolution
            // print( COM3, "\rInsufficient resolution" ); // Debug Output
            return( 0 );
        } // if
        globalPygmyPWM[ i ].CR = PYGMY_PWM_EN;
        globalPygmyPWM[ i ].Pin = ucPin; 
    } // else
    
    globalPygmyPWM[ i ].Count = globalPygmyPWM[ i ].UpCount;

    return( 1 );
}

void pinProcessSoftPWM( void )
{
    u16 i;

    for( i = 0; i < PYGMY_PWMCHANNELS; i++ ){
        if( globalPygmyPWM[ i ].CR & PYGMY_PWM_EN ){
            if( globalPygmyPWM[ i ].Count ){
                --globalPygmyPWM[ i ].Count;
            } else{
                if( globalPygmyPWM[ i ].CR & PYGMY_PWM_DIR ){
                    globalPygmyPWM[ i ].Count = globalPygmyPWM[ i ].DownCount;
                    globalPygmyPWM[ i ].CR &= ~PYGMY_PWM_DIR;
                    pinSet( globalPygmyPWM[ i ].Pin, LOW );
                } else{
                    globalPygmyPWM[ i ].Count = globalPygmyPWM[ i ].UpCount;
                    globalPygmyPWM[ i ].CR |= PYGMY_PWM_DIR;
                    pinSet( globalPygmyPWM[ i ].Pin, HIGH );
                } // else
            } // else
        } // if
    } // for
}

void TIM1_UP_IRQHandler (void) // TIM16 // TIM11 // TIM9
{
    TIM16->SR = 0;
    pinProcessSoftPWM();
}

void EXTI0_IRQHandler( void ) 
{
    EXTI->PR |= BIT0;      // Clear pending bits to prevent recursive access
    globalIRQPR |= BIT0;
    ( globalIRQHandlers[ 0 ] )();
    globalIRQPR &= ~BIT0;
}

void EXTI1_IRQHandler( void ) 
{
    EXTI->PR |= BIT1;      // Clear pending bits to prevent recursive access
    globalIRQPR |= BIT1;
    ( globalIRQHandlers[ 1 ] )();
    globalIRQPR &= ~BIT1;
}

void EXTI2_IRQHandler( void ) 
{
    EXTI->PR |= BIT2;      // Clear pending bits to prevent recursive access
    globalIRQPR |= BIT2;
    ( globalIRQHandlers[ 2 ] )();
    globalIRQPR &= ~BIT2;
}

void EXTI3_IRQHandler( void ) 
{
    EXTI->PR |= BIT3;      // Clear pending bits to prevent recursive access
    globalIRQPR |= BIT3;
    ( globalIRQHandlers[ 3 ] )();
    globalIRQPR &= ~BIT3;
}

void EXTI4_IRQHandler( void ) 
{
    EXTI->PR |= BIT4;      // Clear pending bits to prevent recursive access
    globalIRQPR |= BIT4;
    ( globalIRQHandlers[ 4 ] )();
    globalIRQPR &= ~BIT4;
}

void EXTI9_5_IRQHandler( void ) 
{
    if( EXTI->PR & BIT5 ){
        EXTI->PR |= BIT5;      // Clear pending bits to prevent recursive access
        globalIRQPR |= BIT5;
        ( globalIRQHandlers[ 5 ] )();
        globalIRQPR &= ~BIT5;
    } // if
    if( EXTI->PR & BIT6 ){
        EXTI->PR |= BIT6;
        globalIRQPR |= BIT6;
        ( globalIRQHandlers[ 6 ] )();
        globalIRQPR &= ~BIT6;
    } // if 
    if( EXTI->PR & BIT7 ){
        EXTI->PR |= BIT7;
        globalIRQPR |= BIT7;
        ( globalIRQHandlers[ 7 ] )();
        globalIRQPR &= ~BIT7;
    } // if
    if( EXTI->PR & BIT8 ){
        EXTI->PR |= BIT8;
        globalIRQPR |= BIT8;
        ( globalIRQHandlers[ 8 ] )();
        globalIRQPR &= ~BIT8;
    } // if
    if( EXTI->PR & BIT9 ){
        EXTI->PR |= BIT9;
        globalIRQPR |= BIT9;
        ( globalIRQHandlers[ 9 ] )();
        globalIRQPR &= ~BIT9;
    } // if
    
}

void EXTI15_10_IRQHandler( void ) 
{
    if( EXTI->PR & BIT10 ){
        EXTI->PR |= BIT10;
        globalIRQPR |= BIT10;
        ( globalIRQHandlers[ 10 ] )();
        globalIRQPR &= ~BIT10;
    } // if
    if( EXTI->PR & BIT11 ){
        EXTI->PR |= BIT11;
        globalIRQPR |= BIT11;
        ( globalIRQHandlers[ 11 ] )();
        globalIRQPR &= ~BIT11;
    } //if 
    if( EXTI->PR & BIT12 ){
        EXTI->PR |= BIT12;
        globalIRQPR |= BIT12;
        ( globalIRQHandlers[ 12 ] )();
        globalIRQPR &= ~BIT12;
    } // if
    if( EXTI->PR & BIT13 ){
        EXTI->PR |= BIT13;
        globalIRQPR |= BIT13;
        ( globalIRQHandlers[ 13 ] )();
        globalIRQPR &= ~BIT13;
    } // if
    if( EXTI->PR & BIT14 ){
        EXTI->PR |= BIT14;
        globalIRQPR |= BIT14;
        ( globalIRQHandlers[ 14 ] )();
        globalIRQPR &= ~BIT14;
    } // if
    if( EXTI->PR & BIT15 ){
        EXTI->PR |= BIT15;
        globalIRQPR |= BIT15;
        ( globalIRQHandlers[ 15 ] )();
        globalIRQPR &= ~BIT15;
    } // else if
}

void cpuComOpen( u8 port, u16 tx, u16 rx, u16 cts, u16 rts, u32 baudRate, u8 flowControl )
{
    USART_TYPEDEF *Uart;
    
    //enableComClock( port );
    pinConfig( tx, ALT );
    pinConfig( rx, PULLUP );         
    if( flowControl ){ 
        pinConfig( rts, ALT );
        pinConfig( cts, IN );
    } // if
    PYGMY_RCC_AFIO_ENABLE;
    switch( port ){
        case 0:
        case 1:
            PYGMY_RCC_USART1_ENABLE;
            Uart = USART1;
            interruptEnable( USART1_IRQ );
            interruptSetPriority( USART1_IRQ, 1 );
            break;
        case 2:
            PYGMY_RCC_USART2_ENABLE;
            Uart = USART2;
            interruptEnable( USART2_IRQ );
            interruptSetPriority( USART2_IRQ, 1 );
            break;
        case 3:
            PYGMY_RCC_USART3_ENABLE;
            Uart = USART3;
            interruptEnable( USART3_IRQ );
            interruptSetPriority( USART3_IRQ, 1 );
            break;
        case 4:
            PYGMY_RCC_USART4_ENABLE;
            Uart = USART4;
            interruptEnable( UART4_IRQ );
            interruptSetPriority( UART4_IRQ, 1 );
            break;
        case 5:
            PYGMY_RCC_USART5_ENABLE;
            Uart = USART5;
            interruptEnable( UART5_IRQ );
            interruptSetPriority( UART5_IRQ, 1 );
            break;
        /*case 6:
            PYGMY_RCC_USART6_ENABLE;
            Uart = UART6;
            interruptEnable( USART6_IRQ );
            interruptSetPriority( UART6_IRQ, 1 );
            break;*/
            //ToDo: Add support for USART6-8
    } // switch 
    Uart->CR3 = 0;//USART_ONEBITE;
    // Warning! CTS and RTS are not supported on UART4-UART5
    if( flowControl ){
        Uart->CR3 |= (USART_RTSE|USART_CTSE);
    } // if
    //MainClock = sysGetMainClock();
    Uart->CR1 = 0;
    Uart->BRR = ( ( ( cpuGetMainClock() >> 4 ) / baudRate ) << 4 ) + ( ( ( ( cpuGetMainClock() / 2 ) / baudRate ) ) & 0x0007 );
    //Uart->BRR = ( ( ( MainClock >> 3 ) / BaudRate ) << 4 ) + ( ( ( MainClock / BaudRate ) ) & 0x0007 );
    //if( Options & TXIE ){
        //Uart->CR1 = ( USART_OVER8 | USART_UE | USART_TXEIE | USART_RXNEIE | USART_TE | USART_RE  );
    //} else{
    Uart->CR1 = ( USART_OVER8 | USART_UE | USART_RXNEIE | USART_TE | USART_RE  );
    //} // else
    
    Uart->SR = 0;
}

#pragma push_options
#pragma optimize ("O0")
#ifdef __cplusplus
extern "C" {
#endif

__attribute__ (( weak )) void USART1_IRQHandler( void )
{
    u16 i;
    char c;

    if( USART1->SR & USART_RXNE){
        if( usart1RXHandler ){
            c = USART1->DR;
            //usart1RXHandler( (Uart&)*com1, c );
            usart1RXHandler( com1, c );
        } // if
    } // if
    if( USART1->SR & USART_TXE ){
        if( usart1TXHandler ){
            //c = usart1TXHandler( (Uart&)*com1 );
            c = usart1TXHandler( com1 );
            for( i = 0; !( USART1->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART1->DR = c;
        } // if
    } // if
    USART1->SR = 0;
}
#ifdef __cplusplus
}
#endif
#pragma pop_options

void *getCom1Uart( void )
{
    return( com1 );
}

void setCom1Uart( void *com )
{
    com1 = com;
}

void setCom1TXHandler( void *handler )
{
    usart1TXHandler = (char (*)( void* ))handler;
}

void *getCom1TXHandler( void )
{
    return( ( void* )usart1TXHandler );
}

void setCom1RXHandler( void *handler)
{
    usart1RXHandler = (void (*)( void*, char ))handler;
}

void *getCom1RXHandler( void )
{
    return( ( void* )usart1RXHandler );
}

void putcUSART1( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART1->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART1->DR = Byte;
}

void putsUSART1( u8 *ucBuffer )
{
    for( ; *ucBuffer ; ){
        putcUSART1( *(ucBuffer++) );  
    } // for
}

__attribute__ (( weak )) void USART2_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART2->SR & USART_RXNE){
        if( usart2RXHandler ){
            c = USART2->DR;
            usart2RXHandler( com2, c );
        } // if
    } // if
    if( USART2->SR & USART_TXE ){
        if( usart2TXHandler ){
            c = usart2TXHandler( com2 );
            for( i = 0; !( USART2->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART2->DR = c;
        } // if
    } // if
    USART2->SR = 0;
}

void *getCom2Uart( void )
{
    return( com2 );
}

void setCom2Uart( void *com )
{
    com2 = com;
}

void setCom2TXHandler( void *handler )
{
    usart2TXHandler = (char (*)( void* ))handler;
}

void *getCom2TXHandler( void )
{
    return( ( void* )usart2TXHandler );
}

void setCom2RXHandler( void *handler )
{
    usart2RXHandler = (void (*)( void*, char ))handler;
}

void *getCom2RXHandler( void )
{
    return( ( void* )usart2RXHandler );
}

void putcUSART2( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART2->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART2->DR = Byte;
}

void putsUSART2( u8 *ucBuffer )
{ 
    for( ; *ucBuffer ; ){
        putcUSART2( *(ucBuffer++) );  
    } // for
}

__attribute__ (( weak )) void USART3_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART3->SR & USART_RXNE){
        if( usart3RXHandler ){
            c = USART3->DR;
            usart3RXHandler( com3, c );
        } // if
    } // if
    if( USART3->SR & USART_TXE ){
        if( usart3TXHandler ){
            c = usart3TXHandler( com3 );
            for( i = 0; !( USART3->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART3->DR = c;
        } // if
    } // if
    USART3->SR = 0;
}

void *getCom3Uart( void )
{
    return( com3 );
}

void setCom3Uart( void *com )
{
    com3 = com;
}

void setCom3TXHandler( void *handler )
{
    usart3TXHandler = (char (*)( void* ))handler;
}

void *getCom3TXHandler( void )
{
    return( ( void* )usart3TXHandler );
}

void setCom3RXHandler( void *handler )
{
    usart3RXHandler = (void (*)( void*, char ))handler;
}

void *getCom3RXHandler( void )
{
    return( ( void* )usart3RXHandler );
}

void putcUSART3( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART3->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART3->DR = Byte;
}

void putsUSART3( u8 *ucBuffer )
{ 
    for( ; *ucBuffer ; ){
        putcUSART3( *(ucBuffer++) );  
    } // for
}

__attribute__ (( weak )) void USART4_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART4->SR & USART_RXNE){
        if( usart4RXHandler ){
            c = USART4->DR;
            usart4RXHandler( com4, c );
        } // if
    } // if
    if( USART4->SR & USART_TXE ){
        if( usart4TXHandler ){
            c = usart4TXHandler( com4 );
            for( i = 0; !( USART4->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART4->DR = c;
        } // if
    } // if
    USART4->SR = 0;
}

void *getCom4Uart( void )
{
    return( com4 );
}

void setCom4Uart( void *com )
{
    com4 = com;
}

void setCom4TXHandler( void *handler )
{
    usart4TXHandler = (char (*)( void* ))handler;
}

void *getCom4TXHandler( void )
{
    return( ( void* )usart4TXHandler );
}

void setCom4RXHandler( void *handler )
{
    usart4RXHandler = (void (*)( void*, char ))handler;
}

void *getCom4RXHandler( void )
{
    return( ( void* )usart4RXHandler );
}

void putcUSART4( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART4->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART4->DR = Byte;
}

void putsUSART4( u8 *ucBuffer )
{ 
    u16 i;

    for( ; *ucBuffer ; ){
        putcUSART4( *(ucBuffer++) );  
    } // for
}

__attribute__ (( weak )) void USART5_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART5->SR & USART_RXNE){
        if( usart5RXHandler ){
            c = USART5->DR;
            usart5RXHandler( com5, c );
        } // if
    } // if
    if( USART5->SR & USART_TXE ){
        if( usart5TXHandler ){
            c = usart5TXHandler( com5 );
            for( i = 0; !( USART5->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART5->DR = c;
        } // if
    } // if
    USART5->SR = 0;
}

void setCom5Uart( void *com )
{
    com5 = com;
}

void setCom5TXHandler( void *handler )
{
    usart5TXHandler = (char (*)( void* ))handler;
}

void *getCom5TXHandler( void )
{
    return( ( void* )usart5TXHandler );
}

void setCom5RXHandler( void *handler )
{
    usart5RXHandler = (void (*)( void*, char ))handler;
}

void *getCom5RXHandler( void )
{
    return( ( void* )usart5RXHandler );
}

void putcUSART5( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART5->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART5->DR = Byte;
}

void putsUSART5( u8 *ucBuffer )
{ 
    u16 i;

    for( ; *ucBuffer ; ){
        putcUSART5( *(ucBuffer++) );  
    } // for
}

/*__attribute__ (( weak )) void USART6_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART6->SR & USART_RXNE){
        if( usart6RXHandler ){
            usart6RXHandler( USART6->DR );
        } // if
    } // if
    if( USART6->SR & USART_TXE ){
        if( usart6TXHandler ){
            c = usart6TXHandler( );
            for( i = 0; !( USART6->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART6->DR = c;
        } // if
    } // if
    USART6->SR = 0;
}

void setCom6Uart( void *com )
{
    com6 = com;
}

void setCom6TXHandler( PYGMY_RXHANDLERPTR handler )
{
    usart6TXHandler = handler;
}

PYGMY_TXHANDLERPTR getCom6TXHandler( void )
{
    return( usart6TXHandler );
}

void setCom6RXHandler( PYGMY_RXHANDLERPTR handler )
{
    usart6RXHandler = handler;
}

PYGMY_RXHANDLERPTR getCom6RXHandler( void )
{
    return( usart6RXHandler );
}

void putcUSART6( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART6->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART6->DR = Byte;
}

void putsUSART6( u8 *ucBuffer )
{ 
    u16 i;

    for( i = 0; !( USART6->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART6->DR = Byte;
}

__attribute__ (( weak )) void USART7_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART7->SR & USART_RXNE){
        if( usart7RXHandler ){
            usart7RXHandler( USART7->DR );
        } // if
    } // if
    if( USART7->SR & USART_TXE ){
        if( usart7TXHandler ){
            c = usart7TXHandler( );
            for( i = 0; !( USART7->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART7->DR = c;
        } // if
    } // if
    USART7->SR = 0;
}

void setCom7Uart( void *com )
{
    com7 = com;
}

void setCom7TXHandler( PYGMY_RXHANDLERPTR handler )
{
    usart7TXHandler = handler;
}

PYGMY_TXHANDLERPTR getCom7TXHandler( void )
{
    return( usart7TXHandler );
}

void setCom7RXHandler( PYGMY_RXHANDLERPTR handler )
{
    usart7RXHandler = handler;
}

PYGMY_RXHANDLERPTR getCom7RXHandler( void )
{
    return( usart7RXHandler );
}

void putcUSART7( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART7->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART7->DR = Byte;
}

void putsUSART7( u8 *ucBuffer )
{ 
    u16 i;

    for( i = 0; !( USART7->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART7->DR = Byte;
}

__attribute__ (( weak )) void USART8_IRQHandler( void )
{
    u16 i;
    u8 c;

    if( USART8->SR & USART_RXNE){
        if( usart8RXHandler ){
            usart8RXHandler( USART8->DR );
        } // if
    } // if
    if( USART8->SR & USART_TXE ){
        if( usart8TXHandler ){
            c = usart8TXHandler( );
            for( i = 0; !( USART8->SR & USART_TXE ) && i < 10000; i++ ){;}
            USART8->DR = c;
        } // if
    } // if
    USART8->SR = 0;
}

void setCom8Uart( void *com )
{
    com8 = com;
}

void setCom8TXHandler( PYGMY_RXHANDLERPTR handler )
{
    usart8TXHandler = handler;
}

PYGMY_TXHANDLERPTR getCom8TXHandler( void )
{
    return( usart8TXHandler );
}

void setCom8RXHandler( PYGMY_RXHANDLERPTR handler )
{
    usart8RXHandler = handler;
}

PYGMY_RXHANDLERPTR getCom8RXHandler( void )
{
    return( usart8RXHandler );
}

void putcUSART8( u8 Byte )
{
    u16 i;

    for( i = 0; !( USART8->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART8->DR = Byte;
}

void putsUSART8( u8 *ucBuffer )
{ 
    u16 i;

    for( i = 0; !( USART8->SR & USART_TXE ) && i < 10000; i++ ){;}
    USART8->DR = Byte;
}*/

/**************************************************************************
                            Public Analog 
***************************************************************************/
const u8 PYGMY_ADC_CHANNELS[] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5, ADCTEMP, ADCREF };
u16 uiGlobalADC1Channel, uiGlobalADC1[ 16 ], globalADCStatus = 0;

u16 pinAnalog( u8 ucPin )
{
    if( !adcGetStatus( ) ){
        adcSingleSampleInit();
    } // if

    return( adcSingleSample( ucPin ) );
}

u16 adcGetStatus( void )
{
    return( globalADCStatus );
}

void ADC1_2_IRQHandler( void )
{
    u16 i;

    if( ADC1->SR & ADC_EOC ){
      
        //uiGlobalADC1[ uiGlobalADC1Channel++ ] = ADC1->DR;
        
    } // if
    
    ADC1->SR = 0x00;
        
}

void DMAChannel1_IRQHandler( void )
{
    u16 i;

    if( DMA1->ISR & BIT3 ){
        // ToDo: Add DMA Handler code
    }
    //DMA1->IFCR = 0x0000000F;
}

void adcInit( void ) //u32 ulScan )
{
    // ADC1 uses DMA1 Channel1 to manage Data Register after conversion
    u16 i;

    for( i = 0; i< 16; i++ ){
        uiGlobalADC1[ i ] = 0;
        ADC1->SQR1 = 0;
        ADC1->SQR2 = 0;
        ADC1->SQR3 = 0;
        ADC1->SMPR1 = 0;
        ADC1->SMPR2 = 0;
    } // for
    PYGMY_RCC_ADC1_ENABLE;
    PYGMY_RCC_DMA1_ENABLE;
    PYGMY_DMA1_CH1_DISABLE;
    ADC1->CR2 = 0x00;       // Clear ADC if already in use
    //enableInterrupt( ADC_IRQ );
    //enableInterrupt( DMAChannel1_IRQ );
    //setInterruptPriority( DMAChannel1_IRQ, 1 );
    //setInterruptPriority( ADC_IRQ, 1 );

    ADC1->CR1 = ADC_SCAN;//BIT5; // Enable EOC Interrupt 
    
    //ADC1->CR2 |= (BIT20|BIT19|BIT18|BIT17); // Software trigger
    ADC1->CR2 = (ADC_DMA|ADC_EXTTRIG|ADC_EXTSEL_SWSTART|ADC_ADON); // Start ADC with Calibration
    //ADC1->CR2 = ADC_ADON;
    //while( !(ADC1->CR2 & ADC_CAL) ); // Wait for Calibration to complete
    
    DMA1_CH1->CPAR = (volatile u32)(volatile u32*)ADC1->DR; // Peripheral Register Pointer
    DMA1_CH1->CMAR = (volatile u32)(volatile u32*)uiGlobalADC1; // Destination Memory Address
    DMA1_CH1->CNDTR = 0; // Number of values to transfer
    DMA1_CH1->CCR = ( DMA_MSIZE16 | DMA_PSIZE16 | DMA_MINC | DMA_CIRC );   
}

void adcStart( void )
{
    //ADC1->CR2 |= ADC_ADON;   // Start Single Conversion
    ADC1->CR2 |= ADC_SWSTART;
}

u8 adcSetSampleTime( u8 ucChannel, u8 ucSampleTime )
{
    u8 ucInput;

    //ucInput = 
    
    //ADC1->SMPR1 = 
    //ADC1->SMPR2 = 
}

u8 adcEnableChannel( u8 ucPin ) 
{
    u16 i;
    u8 ucChannel;

    ucChannel = adcGetChannelFromPin( ucPin ) + 1;
    for( i = 1; i < 17; i++ ){
        if( !adcGetSQR( i ) ){
            break; // found empty channel
        } // if
    } // for
    if( i == 17 ) {
        return( 0 ); // No empty channels
    } // if

    if( i < 7 ){
        ADC1->SQR1 |= ( ucChannel << ( ( i - 1 ) * 5 ) );
    } else if( i < 13 ){
        ADC1->SQR2 |= ( ucChannel << ( ( i - 7 ) * 5 ) );
    } else{
        ADC1->SQR3 |= ( ucChannel << ( ( i - 13 ) * 5 ) );
    } // else
    //PYGMY_DMA1_CH1_DISABLE;
    //DMA1_CH1->CNDTR += 1;
    //DMA1_CH1->CCR |= DMA_EN;  

    return( 1 );
}

u8 adcGetSQR( u8 ucIndex )
{
    // Note!!! This function uses a 1 base index, i.e. SQ1, etc., to match the ST datasheets
    u8 ucSQR;
    
    if( ucIndex < 7 ){
        ucSQR = ADC1->SQR1 >> ( ( ucIndex - 1 ) * 5 );
    } else if( ucIndex < 13 ){
        ucSQR = ADC1->SQR2 >> ( ( ucIndex - 7 ) * 5 );
    } else{
        ucSQR = ADC1->SQR3 >> ( ( ucIndex - 13 ) * 5 );
    } // else
    
    return( ucSQR & 0x1F ); // Mask the 5 bit register value
}

u8 adcDisableChannel( u8 ucChannel )
{

}

void adcDisableAll( u8 ucChannel )
{

}

void adcSingleSampleInit( void )
{
    //u16 i;

    PYGMY_RCC_ADC1_ENABLE;
    ADC1->CR2 = 0;
    ADC1->SR = 0;
    ADC1->CR1 = 0;
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->CR2 = (ADC_CAL|ADC_EXTTRIG|ADC_EXTSEL_SWSTART|ADC_ADON);
    while( ADC1->CR2 & ADC_CAL );
    globalADCStatus = BIT0;
    //for( i = 0; i < 10; i++ ){
    //    adcSingleSample( ADCREF );
    //} // for
    
}

u16 adcSingleSample( u8 ucPin )
{
    u16 uiADC;

    ADC1->SQR3 = adcGetChannelFromPin( ucPin );
    ADC1->SR = 0;
    ADC1->CR2 |= (ADC_ADON);
    while( !( ADC1->SR & ADC_EOC ) );    
    uiADC = ADC1->DR;
    return( uiADC ); 
}

u8 adcGetChannelFromPin( u8 ucPin )
{
    u8 i;

    for( i = 0; i < 18; i++ ){
        if( PYGMY_ADC_CHANNELS[ i ] == ucPin ){
            return( i );
        } // if
    } // for

    return( 0 );
}

u16 adcGet( u8 ucChannel )
{
    
}

float adcConvertRawToFloat( u16 raw )
{
    // This function is written for 12 bit ADC with a raw range of 0-4095
    float voltage;
    
    voltage = (double)raw * ( (double)PYGMY_SUPPLYVOLTAGE / 4096.0 );

    return( voltage );
}
/**************************************************************************
                            Private Analog 
***************************************************************************/