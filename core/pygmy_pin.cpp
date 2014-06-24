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
#include "pygmy_type.h"
#include "pygmy_pin.h"
#include "stm32.h"

using namespace std;

Pin::Pin( void )
{

}

Pin::Pin( u16 pin )
{
    Pin_pin = pin;
    //pinConfig( Pin_pin, mode );
}

Pin::Pin( u16 pin, u8 mode )
{
    // call the HAL function for configuring a pin
    Pin_pin = pin;
    Pin_mode = mode;
    pinConfig( Pin_pin, mode );
}

void Pin::set( bool state )
{
    // call the HAL function for setting a pin state
    pinSet( Pin_pin, state );
}

bool Pin::get( void )
{
    // call the HAL function for getting a pin state
    return( pinGet( Pin_pin ) );
}

u16 Pin::analog( void )
{
    adcSingleSampleInit();

    return( adcSingleSample( Pin_pin ) );
    //return( pinAnalog( Pin_pin ) );
}

u16 Pin::interrupt( void *handler )
{
    
}

void Pin::pwm( u32 frequency, u8 dutyCyle )
{

}

PWMChannel::PWMChannel( u16 pin, u32 frequency, u8 dutyCycle ) : Pin( pin)
{
    Pin_frequency;
    Pin_dutyCycle = dutyCycle;
    pinPWM( pin, frequency, dutyCycle );
}

u32 PWMChannel::getFrequency( void )
{
    return( Pin_frequency );
}

u8 PWMChannel::getDutyCycle( void )
{
    return( Pin_dutyCycle );
}

void PWMChannel::setDutyCycle( u32 dutyCycle )
{
    Pin_dutyCycle = dutyCycle;
    pinPWM( Pin_pin, Pin_frequency, Pin_dutyCycle );
}

void PWMChannel::setFrequency( u32 frequency )
{
    Pin_frequency = frequency;
    pinPWM( Pin_pin, Pin_frequency, Pin_dutyCycle );
}
