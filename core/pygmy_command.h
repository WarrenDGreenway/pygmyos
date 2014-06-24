/**************************************************************************
    PygmyOS ( Pygmy Operating System ) - BootLoader
    Copyright (C) 2011-2012  Warren D Greenway

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
#pragma once
#include <vector>
#include "pygmy_string.h"
using namespace std;

typedef bool (*CommandFunctionPointer)( void *, PygmyString& );

class Command{
    public:
        Command( void );
        Command( void *, void *, PygmyString& );
        Command( void *, void *, const char * );
   
        void setName( PygmyString& );
        void setName( const char * );
        PygmyString& getName( void );
        void setHandler( void * );
        void *getHandler( void );
        void setParent( void *parent );
        void *getParent( void );
        bool run( PygmyString );

    protected:
        void *parent; // contains a pointer to the parent class instance
        static bool emptyHandler( void *c, PygmyString& s );
        PygmyString name;
        bool (*handler)( void *, PygmyString& );
};




