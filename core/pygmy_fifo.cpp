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
#include <string>
#include "pygmy_string.h"
#include "pygmy_fifo.h"
#include "pygmy_type.h"

Fifo::Fifo( void )
{
    this->index = 0;
    this->length = 0;
    this->size = 0;
    this->action = false;
    this->buffer = (char*)NULL;
    eventHandler = NULL;
    actions = vector<Command>();
}

Fifo::Fifo( int size )
{
    this->index = 0;
    this->length = 0;
    this->size = size;
    this->action = false;
    this->buffer = new char[ size ];
    eventHandler = NULL;
    actions = vector<Command>();
}

void Fifo::setNewBuffer( int size )
{
    if( buffer ){
        delete( buffer );
    } // if

    buffer = new char[ size ];
}

void Fifo::addAction( Command& command )
{
    this->action = true;
    this->actions.push_back( command );
}

void Fifo::removeAction( PygmyString& s )
{
    
    if( this->actions.size() == 0 ){
        this->action = false;
    } // if
}

void Fifo::setEventHandler( void (*handler)( void*, char ) )//void *handler )//PYGMYHANDLER handler )
{
    eventHandler = handler;
}

/*void Fifo::setActionHandler( void (*handler)( PygmyString &s) )
{
    actionHandler = handler;
}*/

void Fifo::reset( void )
{
    index = 0;
    length = 0;
}

bool Fifo::isEmpty( void )
{
    if( this->length ){
        return( false ); // Still data in buffer
    } // if
    
    return( true ); // Buffer empty
}

bool Fifo::isFull( void )
{
    if( this->length == this->size ){
        return( true );
    } // if

    return( false );
}

char Fifo::popChar( void )
{
    // Function returns last char received and decrements length
    int i;
    
    if( length ){
        --length;
        i = ( index + ( length ) ) % size;
        return( buffer[ i ] );
    } // if
    
    return( 0 ); // there was no data remaining
}

char Fifo::peekChar( void )
{
    // Function returns value of last char without affecting FIFO
    int i;
    
    if( this->length ){
        i = ( this->index + ( this->length - 1 ) ) % this->size;
        return( this->buffer[ i ] );
    } // if
    
    return( 0 );
}

int Fifo::getSize( void )
{
    return( size );
}

char Fifo::getChar( void )
{
    int i;

    if( length ){
        i = index;
        index = ( index+1 ) % size; // modulo prevents overrun
        --length;
        return( buffer[ i ] );
    } // if

    return( 0 );
}

PygmyString Fifo::getString( void )
{
    PygmyString s;

    for( ; length; ){
        s += getChar();
    } // for
    
    return( s );
}

void Fifo::pushChar( char c )
{
    // Function prepends char to FIFO and increments length
    int i;

    if( length < size){
        i = ( index + length ) % size;
        ++length;
        buffer[ i ] = c;
    } // if
    if( eventHandler ){
        eventHandler( (void*)this, c );
    } // if
}

void Fifo::putChar( char c )
{    
    int i, ii, iii;
    PygmyString name;

    if( length < ( size - 1 ) ){
        i = ( index + length ) % size;
        buffer[ i ] = c;
        ++length;
    } // if
    if( eventHandler ){
        eventHandler( (void*)this, c );
    } // if
    if( this->action ){ 
        // actions are enabled, this carries a significant penalty
        // We must match as few characters as necessary
        // There could be significant wasted overhead involved with getting the fifo as a string to match
        // We will match starting from the end of the fifo (the char just received)
        int index = this->index;
        int length = this->length - 1;
        int size = this->size;
        for( i = 0; i < this->actions.size(); i++ ){
            // process one action string at a time for a complete match
            name = this->actions[ i ].getName();
            for( ii = 0; ii < name.size(); ii++ ){
                iii = ( index + length ) % size;
                if( this->buffer[ iii ] != name[ name.size() - ii - 1 ] ){ // remember that we are matching from last to first
                    // we have reached a mismatch, don't waste time matching further
                    break; 
                } // if
            } // for
            if( ii == name.size() ){
                // we have a match, call the registered handler
                CommandFunctionPointer handler = (CommandFunctionPointer)this->actions[ i ].getHandler();
                name = this->getString();
                handler( this->actions[ i ].getParent(), name );
            } // if
        } // for
    } // if
}


void Fifo::putString( PygmyString s )
{
    int i;

    for( i = 0; i < s.length(); i++ ){
        putChar( s[ i ] );
    } // for
}

