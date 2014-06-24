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

#include <string>
#include "pygmy_type.h"
#include "pygmy_stream.h"
#include "pygmy_uart.h"

enum { COM1=1, COM2, COM3, COM4, COM5, COM6, COM7, COM8 };
enum { RS232=1, RS422, RS485  };

class Uart : public Stream{
    public:
        Uart( void );
        //Uart( u8, u16, u16 );
        void init( short, short, int, int, int, int );
        Uart( short, short, int, int, int, int );
        //Uart( u8, u16, u16, u16, u16 );
        void setPipe( short port );
        void setProtocol( u8 );
        short getProtocol( void );
        void setDataBits( u8 );
        short getDataBits( void );
        void setFlowControl( bool );
        bool getFlowControl( void );
        enum ComPort { COM1=1, COM2, COM3, COM4, COM5, COM6, COM7, COM8 };
        enum Protocol { RS232=1, RS422, RS485  };
        
    protected:
        static void com1TXChar( void *, char );
        static void com2TXChar( void *, char );
        static void com3TXChar( void *, char );
        static void com4TXChar( void *, char );
        static void com5TXChar( void *, char );
        static void rxChar( void *, char );
        void *getComUart( short port );
        short port;
        short pipe;
        bool echo;
        short protocol;
        short dataBits;
        short stopBits;
        bool flowControl;
        long baud;
        
};