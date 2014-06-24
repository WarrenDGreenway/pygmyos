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

#pragma once

class Pin{
    public:
        Pin( void );
        Pin( u16 );
        Pin( u16, u8 );
        void set( bool state );
        bool get( void );
        u16 analog( void );
        u16 interrupt( void *handler );
        void pwm( u32 frequency, u8 dutyCyle );
    protected:
        u16 Pin_pin;
        u8 Pin_mode;
};

class PWMChannel: public Pin{
    public:
        PWMChannel( u16, u32, u8 );
        void setDutyCycle( u32 );
        void setFrequency( u32 );
        u8 getDutyCycle( void );
        u32 getFrequency( void );
    protected:
        //Pin Pin_pin;
        u32 Pin_frequency;
        u32 upCount;
        u32 downCount;
        u32 count;
        u8 Pin_dutyCycle;
        u8 CR;
};