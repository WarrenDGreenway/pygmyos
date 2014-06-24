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
#include "pygmy_command.h"
#include "pygmy_stream.h"
#include "pygmy_string.h"
#include "pygmy_uart.h"

class Console : public Uart{
    public:
        Console( void );
        Console( short port, short protocol, int tx, int rx, int cts, int rts ) : Uart( port, protocol, tx, rx, cts, rts ){}
        void init( void );
        void init( vector<Command>&commands );
        static bool run( void *, PygmyString& s );
        static bool uname( void *, PygmyString& );
        static bool volt( void *, PygmyString& );
        static bool analog( void *, PygmyString& );
        static bool pinconfig( void *, PygmyString& );
        static bool pinget( void *, PygmyString& );
        static bool pinpwm( void *, PygmyString& );
        static bool pinset( void *, PygmyString& );
        
        static bool format( void *, PygmyString& );
        static bool rx( void *, PygmyString& );
        static bool tx( void *, PygmyString& );
        static bool read( void *, PygmyString& );
        static bool rm( void *, PygmyString& );
        static bool cd( void *, PygmyString& );
        static bool append( void *, PygmyString& );
        static bool open( void *, PygmyString& );
        //static bool new( PygmyString& );
        static bool mkdir( void *, PygmyString& );
        static bool rmdir( void *, PygmyString& );
        //static bool cmd_echo( PygmyString& );
        static bool cat( void *, PygmyString& );
        static bool strings( void *, PygmyString& );
        static bool dump( void *, PygmyString& );
        static bool ls( void *, PygmyString& );
        static bool touch( void *, PygmyString& );
        static bool mv( void *, PygmyString& );
        static bool cp( void *, PygmyString& );
        static bool reset( void *, PygmyString& );
        static bool boot( void *, PygmyString& );  
        static bool flash( void *, PygmyString& );
        static bool fdisk( void *, PygmyString& );
        static bool umount( void *, PygmyString& );
        static bool mount( void *, PygmyString& );
        static bool verify( void *, PygmyString& );
        static bool test( void *, PygmyString& );
        static bool date( void *, PygmyString& );
        static bool time( void *, PygmyString& );
        static bool find( void *, PygmyString& );
        static bool df( void *, PygmyString& );
        static bool du( void *, PygmyString& );
        static bool pwd( void *, PygmyString& );
        static bool tail( void *, PygmyString& );
        static bool cksum( void *, PygmyString& );
        //static bool cmd_if( PygmyString& );
        static bool sleep( void *, PygmyString& );
        static bool lsof( void *, PygmyString& );
        static bool gawk( void *, PygmyString& );
        static bool declare( void *, PygmyString& );
        static bool dc( void *, PygmyString& );
        static bool kill( void *, PygmyString& );
        static bool killall( void *, PygmyString& );
        //static bool cmd_null( PygmyString& );
        //static bool cmd_cmd( PygmyString& );
        //static bool cmd_run( PygmyString& );
    protected:
        
        vector <Command> commands;
};

