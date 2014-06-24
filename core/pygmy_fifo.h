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
#include <vector>
#include <string>
#include "pygmy_command.h"
#include "pygmy_type.h"

class Fifo{
    public:
        Fifo( void );
        Fifo( int );
        void reset( void );
        bool isEmpty( void );
        bool isFull( void );
        char popChar( void );
        char peekChar( void );
        void pushChar( char c );
        char getChar( void );
        int getSize( void );
        PygmyString getString( void );
        void putChar( char );
        void putBuffer( char* );
        void putString( PygmyString s );
        void addAction( Command& command );
        void removeAction( PygmyString& s );
        //void addActionString( PygmyString );
        void setEventHandler( void (*)( void*, char) );
        //void setActionHandler( void (*)( void*, PygmyString& ) );
        void setNewBuffer( int );
    protected:
        void  (*eventHandler)( void*, char );
        //void (void*)(void) eventHandler;
        PYGMYHANDLER actionHandler;
        char *buffer;
        //vector<PygmyString> actionStrings;
        vector<Command> actions;
        bool action;
        int index;
        int length;
        int size;
};
