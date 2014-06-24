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

using namespace std;
#include "pygmy_type.h"
#include "pygmy_uart.h"
#include "core/stm32.h"

Uart::Uart( void )
{
    this->port = 0;
}

/*Uart::Uart( short port, short protocol, u16 tx, u16 rx )
{
    
}*/

Uart::Uart( short port, short protocol, int tx, int rx, int cts, int rts )
{
    this->init( port, protocol, tx, rx, cts, rts );
}

void Uart::init( short port, short protocol, int tx, int rx, int cts, int rts )
{
    this->port = port;
    this->pipe = 0;
    this->echo = false;
    this->protocol = protocol;
    this->dataBits = 8;
    this->stopBits = 1;
    this->flowControl = false;
    this->baud = 115200;
    cpuComOpen( port, tx, rx, cts, rts, 115200, 0 );
    if( port == Uart::COM1 ){
        setCom1Uart( (void*)this );
        TXBuffer->setEventHandler( com1TXChar );
        setCom1RXHandler( (void*)rxChar );
    } else if( port == Uart::COM2 ){
        setCom2Uart( (void*)this );
        TXBuffer->setEventHandler( com2TXChar );
        setCom2RXHandler( (void*)rxChar );
    } else if( port == Uart::COM3 ){
        setCom3Uart( (void*)this );
        TXBuffer->setEventHandler( com3TXChar );
        setCom3RXHandler( (void*)rxChar );
    } else if( port == Uart::COM4 ){
        setCom4Uart( (void*)this );
        TXBuffer->setEventHandler( com4TXChar );
        setCom4RXHandler( (void*)rxChar );
    } else if( port == Uart::COM5 ){
        //setCom5Uart( (void*)this );
        //TXBuffer->setEventHandler( com5TXChar );
        //setCom5RXHandler( (void*)rxChar );
    } else if( port == Uart::COM6 ){
        // ToDo: Implement UART6, UART7, and UART8 in HAL
        //setCom6Uart( this );
        //TXBuffer->setEventHandler( txChar );
        //setCom6RXHandler( (void*)rxChar );
    } else if( port == Uart::COM7 ){
        //setCom7Uart( this );
        //TXBuffer->setEventHandler( txChar );
        //setCom7RXHandler( (void*)rxChar );
    } else if( port == Uart::COM8 ){
        //setCom8Uart( this );
        //TXBuffer->setEventHandler( txChar );
        //setCom8RXHandler( (void*)rxChar );
    } // else if
}

void Uart::setPipe( short port )
{
    this->pipe = port;
    /*if( port == Uart::COM1 ){
        TXBuffer->setEventHandler( com1TXChar );
        setCom1Uart( (void*)this );
    } else if( port == Uart::COM2 ){
        TXBuffer->setEventHandler( com2TXChar );
        setCom2Uart( (void*)this );
    } else if( port == Uart::COM3 ){
        TXBuffer->setEventHandler( com3TXChar );
        //RXBuffer->setEventHandler( com1TXChar );
        setCom1Uart( getCom3Uart() );
        //setCom3RXHandler( 
        //setCom1TXHandler( getCom3TXHandler( ) );
        //TXBuffer->setEventHandler(
    } else if( port == Uart::COM4 ){
        TXBuffer->setEventHandler( com4TXChar );
        setCom4Uart( (void*)this );
    } // else if
    */
}

void Uart::rxChar( void *src, char c )
{
    Uart *com;

    com = (Uart*)src;
    
    com->RXBuffer->putChar( c );
    if( com->echo ){
        com->TXBuffer->pushChar( c );
    } // if
    if( com->pipe ){
        com = (Uart*)com->getComUart( com->pipe );
        if( com ){
            com->TXBuffer->pushChar( c );
        } // if
    } // if
}

void *Uart::getComUart( short port )
{
    if( port == Uart::COM1 ){
        return( getCom1Uart() );
    } else if( port == Uart::COM2 ){
        return( getCom2Uart() );
    } else if( port == Uart::COM3 ){
        return( getCom3Uart() );
    } else if( port == Uart::COM4 ){
        return( getCom4Uart() );
    } // else if

    return( NULL );
}

void Uart::com1TXChar( void* src, char c )
{
    Fifo *fifo;

    fifo = (Fifo *)src;
    putcUSART1( fifo->popChar() );
}

void Uart::com2TXChar( void* src, char c )
{
    Fifo *fifo;

    fifo = (Fifo *)src;
    putcUSART2( fifo->popChar() );
}

void Uart::com3TXChar( void* src, char c )
{
    Fifo *fifo;

    fifo = (Fifo *)src;
    putcUSART3( fifo->popChar() );
}

void Uart::com4TXChar( void* src, char c )
{
    Fifo *fifo;

    fifo = (Fifo *)src;
    putcUSART4( fifo->popChar() );
}

/*void Uart::com5TXChar( void* src, char c )
{
    Fifo *fifo;

    fifo = (Fifo *)src;
    putcUSART5( fifo->popChar() );
}*/

void Uart::setProtocol( u8 protocol )
{
    this->protocol = protocol;
}

short Uart::getProtocol( void )
{
    return( protocol );
}

void Uart::setDataBits( u8 dataBits )
{
    this->dataBits = dataBits;
}

short Uart::getDataBits( void )
{
    return( dataBits );
}

void Uart::setFlowControl( bool flowControl )
{
    this->flowControl = flowControl;
}

bool Uart::getFlowControl( void )
{
    return( flowControl );
}

