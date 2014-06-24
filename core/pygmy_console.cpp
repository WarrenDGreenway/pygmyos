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
#include <vector>
#include "pygmy_string.h"
#include "pygmy_console.h"
#include "pygmy_command.h"
using namespace std;

void Console::init( void )
{
    Command action( (void*)this, (void *)Console::run, "\r" );

    this->RXBuffer->addAction( action );
    commands = vector<Command>(); // initialize the command vector
    //this->commands.insert( new Command( (void *)Console::uname, "uname" ) );
    this->commands.push_back( Command( (void *)this, (void *)Console::uname, "uname" ) );
    //this->print( "\nsize in constructor: %d", this->commands.size() );
}

void Console::init( vector<Command>&commands )
{
    this->commands = commands;
}
/*
bool Console::run( const char *s )
{
    PygmyString tmpString( s );
    
    this->run( tmpString );
}

bool Console::run( PygmyString& s )
{
    
}*/

bool Console::run( void *c, PygmyString& s )
{
    Console *console = (Console*)c;
    bool status;

    int size = console->commands.size();
    //console->print( "\ns: %s", s.c_str() );
    for( int i = 0; i < console->commands.size(); i++ ){
        if( s.startsWith( console->commands[ i ].getName() ) ){
            CommandFunctionPointer handler = (CommandFunctionPointer)console->commands[ i ].getHandler();
            // todo: erase the command from string before passing to handler
            status = handler( console, s );
            if( status == false ){
                console->print( "\nError\n>" );
            } // if
            return( status ); 
        } // if
    } // for
    // no match was found
}

bool Console::uname( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

    console->print( "\nuname not implemented" );
}

bool Console::analog( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::pinconfig( void *c, PygmyString& s )
{
    Console *console = (Console*)c;
    PYGMYPARAMLIST parameters;

    if( s.getAllParameters( &parameters ) && parameters.ParamCount == 2  ){
        /*if( pinConfig( convertStringToPin( parameters.Params[ 0 ] ), s.convertStringToMode( Parameters.Params[ 1 ] ) ) ){
            freeParameterList( &Parameters );
            return( TRUE );
        } // if
        */
    } // if

    return( false );
}

bool Console::pinget( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::pinpwm( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::pinset( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

//bool cmd_set( PygmyString& s );
//bool cmd_erase( PygmyString& s );
bool Console::format( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::rx( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::tx( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::read( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}   

bool Console::rm( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::cd( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::append( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::open( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

/*bool Console::new( vid *, PygmyString& s )
{
    Console *console = (Console*)c;

}*/

bool Console::mkdir( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::rmdir( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

//bool cmd_echo( PygmyString& s );
bool Console::cat( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::strings( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::dump( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::ls( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::touch( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::mv( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::cp( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::reset( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::boot( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}
 
bool Console::flash( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::fdisk( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::umount( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::mount( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::verify( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::test( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::date( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::time( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::find( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::df( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::du( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::pwd( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::tail( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::cksum( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

//bool Console::if( PygmyString& s );
bool Console::sleep( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::lsof( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::gawk( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::declare( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::dc( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::kill( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

bool Console::killall( void *c, PygmyString& s )
{
    Console *console = (Console*)c;

}

//bool cmd_null( PygmyString& s );
//bool cmd_cmd( PygmyString& s );
//bool cmd_run( PygmyString& s );*/

