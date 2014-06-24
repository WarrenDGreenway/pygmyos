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
#pragma once
using namespace std;

#define PUNCT               BIT0
#define WHITESPACE          BIT1
#define NEWLINE             BIT2
#define SEPARATORS          BIT3
#define QUOTES              BIT4
#define COMMA               BIT5
#define ALPHA               BIT6
#define FILESEPARATORS      BIT7

#define DATA        1
#define COMMAND     0
#define PYGMY_STREAMS_ECHO          BIT1
#define PYGMY_STREAMS_BACKSPACE     BIT2
#define PYGMY_STREAMS_ACTIONCHARS   BIT3
#define PYGMY_STREAMS_CR            BIT4
#define PYGMY_STREAMS_USERHANDLER   BIT5

#include <string>
#include "pygmy_string.h"
#include "pygmy_type.h"
#include "pygmy_fifo.h"

class Stream{
    public:
        Stream( void );
        Stream( int, int );
        void print( char *, ... );
        void print( const char *, ... );
        void print( PygmyString& s, ... );
        void printVariadic( PygmyString &, va_list );
        void echoChar( char c );
        void disableDefaultGet( void );
        void enableDefaultGet( void );
        bool isEchoEnabled( void );
        void enableEcho( void );
        void disableEcho( void );
        void enableActionChars( void );
        void disableActionChars( void );
        void enableBackspace( void );
        void disableBackspace( void );

        Fifo *RXBuffer;
        Fifo *TXBuffer;
    protected:
        //short Pipe;
        unsigned int CR;
};

