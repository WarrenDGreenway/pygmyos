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

#include "pygmy_string.h"
#include "pygmy_command.h"

using namespace std;

Command::Command( void )
{
    this->parent = NULL;
    this->name = "";
    this->handler = (CommandFunctionPointer)emptyHandler; 
}
/*
Command::Command( PygmyString name )
{
    this->name = name;
}*/

Command::Command( void *parent, void *handler, const char *name )
{
    PygmyString s( name );

    this->parent = parent;
    this->name = s;
    this->handler = (CommandFunctionPointer)handler;
}

Command::Command( void *parent, void *handler, PygmyString& name )
{
    this->parent = parent;
    this->name = name;
    this->handler = (CommandFunctionPointer)handler;
}

bool Command::emptyHandler( void *c, PygmyString& s )
{
    // This is the default handler
    // If an instance of Command is used before it is initialized, this method will be called
    // This prevents a null pointer exception
}

void Command::setName( PygmyString& s )
{
    this->name = s;
}

void Command::setName( const char *s )
{
    PygmyString tmpName( s );
    
    setName( tmpName );
}

PygmyString& Command::getName( void )
{
    return( this->name );
}

void Command::setHandler( void *handler )
{
    this->handler = (CommandFunctionPointer)handler;
}

void *Command::getHandler( void )
{
    return( (void *)this->handler );
}

void Command::setParent( void *parent )
{
    this->parent = parent;
}

void *Command::getParent( void )
{
    return( this->parent );
}

bool Command::run( PygmyString s )
{
    return( this->handler( this, s ) );
}


