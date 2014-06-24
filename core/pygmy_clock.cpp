/**************************************************************************
    PygmyOS ( Pygmy Operating System )
    Copyright (C) 2011-2012  Warren D Greenway

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
using namespace std;;
#include "pygmy_clock.h"
#include "stm32.h"

Clock::Clock( void )
{
    Clock( 8000000 ); // default to a safe 8MHz if no frequency is specified
}

Clock::Clock( long frequency )
{
    this->mainFrequency = frequency;
    cpuSetMainClock( frequency );
    
    RCC->APB2ENR |= (RCC_IOPBEN|RCC_IOPCEN|RCC_IOPAEN);
    PYGMY_RCC_TIMER1_ENABLE;
    FPEC->ACR = FPEC_ACR_PRFTBE | FPEC_ACR_LATENCY2;
    // HSI Must be ON for Flash Program/Erase Operations
    PYGMY_RCC_HSI_ENABLE;
    PYGMY_RCC_HSE_ENABLE;
    while( !PYGMY_RCC_HSE_READY );
    RCC->CFGR2 = 0;
    RCC->CFGR = RCC_PLL_X9|BIT16|BIT15|BIT14|BIT1;;
    PYGMY_RCC_PLL_ENABLE;
    while( !PYGMY_RCC_PLL_READY );
    // End Initialize Clocks
}

void Clock::setXtal( long frequency )
{
    this->xtalFrequency = frequency;
}

long Clock::getXtal( void )
{
    return( this->xtalFrequency );
}

void Clock::setMainClock( long frequency )
{
    this->mainFrequency = frequency;
}

long Clock::getMainClock( void )
{
    return( this->mainFrequency );
}

void Clock::enableUartClock( short port )
{
    if( port == 1 ){
        PYGMY_RCC_USART1_ENABLE;
    } else if( port == 2 ){
        PYGMY_RCC_USART2_ENABLE;
    } else if( port == 3 ){
        PYGMY_RCC_USART3_ENABLE;
    } else if( port == 4 ){
        PYGMY_RCC_USART4_ENABLE;
    } // else if
}

void Clock::disableUartClock( short port )
{
    if( port == 1 ){
        PYGMY_RCC_USART1_DISABLE;
    } else if( port == 2 ){
        PYGMY_RCC_USART2_DISABLE;
    } else if( port == 3 ){
        PYGMY_RCC_USART3_DISABLE;
    } else if( port == 4 ){
        PYGMY_RCC_USART4_DISABLE;
    } // else if
}

