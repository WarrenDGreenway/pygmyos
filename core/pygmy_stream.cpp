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
#include "pygmy_string.h"
#include "pygmy_type.h"
#include "pygmy_stream.h"
#include "pygmy_fifo.h"
//#include "pygmy_rtc.h"

Stream::Stream( void )
{
    RXBuffer = new Fifo( 128 );
    TXBuffer = new Fifo( 128 );
}

Stream::Stream( int txLength, int rxLength )
{
    if( RXBuffer->getSize() != rxLength ){
        RXBuffer->setNewBuffer( rxLength );
    } // if
    if( TXBuffer->getSize() != txLength ){
        TXBuffer->setNewBuffer( txLength );
    } // if
}

/*void Stream::print( PygmyString& s )
{
    TXBuffer->putString( s.c_str() );
}*/

void Stream::print( char *s, ... )
{
    PygmyString tmpString( s );
    va_list ap;

    va_start( ap, s );
    printVariadic( tmpString, ap );
    va_end( ap );
}

void Stream::print( const char *s, ... )
{
    PygmyString tmpString( s );
    va_list ap;

    va_start( ap, s );
    printVariadic( tmpString, ap );
    va_end( ap );
}

void Stream::print( PygmyString& s, ... )
{
    va_list ap;

    va_start( ap, s );
    printVariadic( s, ap );
    va_end( ap );
}

void Stream::printVariadic( PygmyString& s, va_list ap  )
{
    PYGMYTIME pygmyTime;
    PygmyString value;
    PygmyString format;
    int i, ii, length, type;
   
    for( i = 0; i < s.length(); i++ ){
        // detect newline and generate CR LF sequence
        if( s.at( i ) == '\n' ){
            TXBuffer->putChar( 0x0D ); // CR
            TXBuffer->putChar( 0x0A ); // LF
        } else if( s.at( i ) == '%' ){ // Found format specifier
            // first collect precision, if any
            for( ii = 0, ++i, format = ""; ii < 12 && ( s.isNumeric( s.at( i ) ) || s.at( i ) == '-' 
                || s.at( i ) == '+' || s.at( i ) == '.' ); ii++, i++ ){
                format += s.at( i );
            } // for

            if( s.at( i ) == '%' ){ // Format specifier was only escaping '%' char, print
                TXBuffer->putChar( '%' );
            } else if( s.at( i ) == 'c' ){ 
                TXBuffer->putChar( (char)va_arg( ap, int ) );
            } else if( s.at( i ) == 's' ){
                value = va_arg( ap, char * );
                length = value.length( );
                // determine if padding characters are needed and print
                if( format.length() && format.at( 0 ) == '-' ){
                    format.erase( format.begin() );
                    TXBuffer->putString( value );
                    ii = format.convertStringToInt( );
                    if( ii > length ){
                        length = ii - length;
                    } else{
                        length = 0;
                    } // else
                    for( ii = 0; ii < length; ii++ ){
                         TXBuffer->putChar( ' ' );
                    } // for
                } else{
                    ii = format.convertStringToInt( );
                    if( ii > length ){
                        length = ii - length;
                    } else{
                        length = 0;
                    } // if
                    for( ii = 0; ii < length; ii++ ){
                         TXBuffer->putChar( ' ' );
                    } // for
                    TXBuffer->putString( value );
                } // else
            } else if( s.at( i ) == 'i' || s.at( i ) == 'd' ||
                s.at( i ) == 'x' || s.at( i ) == 'X' || s.at( i ) == 'o' || s.at( i ) == 'f' || s.at( i ) == 'l' ){
                type = 16;
                if( s.at( i ) == 'l' ){
                    ++i;
                    if( s.at( i ) == 'l' ){
                        ++i;
                        type = 64;
                    } // if
                } // if
                format += s.at( i ); // this creates a complete format string (the rest is captured at the beginning)    

                if( type == 64 ){ 
                    format = format.convertIntToString( va_arg( ap, unsigned long long ), format ); 
                } else if( s.at( i ) == 'f' ){
                    format = format.convertFloatToString( va_arg( ap, double ), format );
                } else{
                    format = format.convertIntToString( va_arg( ap, unsigned int ), format ); 
                } // else
                TXBuffer->putString( format );
                
            } else if( s.at( i ) == 'e' || s.at( i ) == 'E' ){
                // ToDo: Add Exponent Handler
                //print( "Exponent Not Supported" );
            } else if( s.at( i ) == 't' ){
                //convertSecondsToSystemTime( va_arg( ap, s32 ), &pygmyTime );
                //print( "%04d-%02d-%02d %02d:%02d:%02d", pygmyTime.Year, pygmyTime.Month, pygmyTime.Day,
                //    pygmyTime.Hour, pygmyTime.Minute, pygmyTime.Second );
            }
        } else{
            // this character isn't part of a format specifier, print it
            TXBuffer->putChar( s.at( i ) );
        } // else
    } // for
}

void Stream::echoChar( char c )
{
    TXBuffer->putChar( c );
}


/*
u8 Stream::setPut( u8 ucStream, void *ptrFunc )
{
    if( ucStream < MAXCOMPORTS ){
        globalStreams[ ucStream ].Put = ptrFunc;
    
        return( 1 );
    } // if

    return( 0 );
}

u8 Stream::setPutc( u8 Stream, void *Func )
{
    if( Stream < MAXCOMPORTS ){
        globalStreams[ Stream ].Putc = Func;
    
        return( 1 );
    } // if

    return( 0 );
}

u8 Stream::setGet( u8 ucStream, void *ptrFunc )
{
    if( ucStream < MAXCOMPORTS ){
        globalStreams[ ucStream ].Get = ptrFunc;
    
        return( 1 );
    } // if

    return( 0 );
}
*/

/*void Stream::setActionChars( u8 *ucString )
{
    // Warning! ucString must be NULL terminated
    globalStreams[ ucStream ].ActionChars = ucString;
}*/

void Stream::disableDefaultGet( void )
{
    CR |= PYGMY_STREAMS_USERHANDLER;
}

void Stream::enableDefaultGet( void )
{
    CR &= ~PYGMY_STREAMS_USERHANDLER;
}

bool Stream::isEchoEnabled( void )
{
    return( CR & PYGMY_STREAMS_ECHO ? true : false );
}

void Stream::enableEcho( void )
{
    CR |= PYGMY_STREAMS_ECHO; 
}

void Stream::disableEcho( void )
{
    CR &= ~PYGMY_STREAMS_ECHO;
}

void Stream::enableActionChars( void )
{
    CR |= PYGMY_STREAMS_ACTIONCHARS;
}

void Stream::disableActionChars( void )
{
    CR &= ~PYGMY_STREAMS_ACTIONCHARS;
}

void Stream::enableBackspace( void )
{
    CR |= PYGMY_STREAMS_BACKSPACE;
}

void Stream::disableBackspace( void )
{
    CR &= ~PYGMY_STREAMS_BACKSPACE;
}

//************

#define EPSILON 1.0e-7
#define floatIsEqual(a, b) (fabs((a)-(b)) < EPSILON)

//const char PYGMYHEXCHARS[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
//const char PYGMYBASE64CHARS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
const char *PYGMYMONTHABBREVIATIONS[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" }; 
#ifdef __PYGMYNEBULA
    #define PYGMYMAXPINS  31
    const PYGMYPAIR PYGMYNEBULAPINS[] = {
                                        { (u8*)"LED0",        LED0 },
                                        { (u8*)"LED1",        LED1 },
                                        { (u8*)"DAC1",        DAC1 },
                                        { (u8*)"DAC2",        DAC2 },
                                        { (u8*)"MCO",         MCO },
                                        { (u8*)"TX1",         TX1 },
                                        { (u8*)"RX1",         RX1 },
                                        { (u8*)"TX2",         TX2 },
                                        { (u8*)"RX2",         RX2 },
                                        { (u8*)"TA0",         TA0 },
                                        { (u8*)"TA1",         TA1 },
                                        { (u8*)"T0",          T0 },
                                        { (u8*)"T1",          T1 },
                                        { (u8*)"T2",          T2 },
                                        { (u8*)"T3",          T3 },
                                        { (u8*)"A0",          A0 },
                                        { (u8*)"A1",          A1 },
                                        { (u8*)"A2",          A2 },
                                        { (u8*)"A3",          A3 },
                                        { (u8*)"A4",          A4 },
                                        { (u8*)"A5",          A5 },
                                        { (u8*)"A6",          A6 },
                                        { (u8*)"A7",          A7 },
                                        { (u8*)"D0",          D0 },
                                        { (u8*)"D1",          D1 },
                                        { (u8*)"D2",          D2 },
                                        { (u8*)"D3",          D3 },
                                        { (u8*)"FLASH_CS",    FLASH_CS },
                                        { (u8*)"FLASH_SCK",   FLASH_SCK },
                                        { (u8*)"FLASH_MISO",  FLASH_MISO },
                                        { (u8*)"FLASH_MOSI",  FLASH_MOSI },
                                        { (u8*)NULL,    0 } };
#endif // __PYGMYNEBULA
#ifndef PYGMYMAXPINS
    #define PYGMYMAXPINS 0
#endif

u8 convertMonthStringToInt( char *Month )
{
    PygmyString s( Month );
    // This function matches the month as a substring, looking for the abbreviations
    u8 i;

    for( i = 0; i < 12; i++ ){
        if( s.seekStringIgnoreCase( (const char*)PYGMYMONTHABBREVIATIONS[ i ] ) ){
            // We have found the month we are looking for, return the month as an integer equivalent
            return( i + 1 ); // There is no zeroth month, 0 is reserved for an error condition
        } // if

    } // for

    return( 0 ); // There is no zeroth month 
}

string convertStateToString( bool state )
{
    if( state == false ){
        return( "0" );
    } else if( state == true ){
        return( "1" );
    } // else

    return( NULL );
}
                                    
u16 convertStringToPin( char *buffer )
{
    PygmyString s( buffer );

    // This function is constant string safe
    u16 i, pin;

    #ifdef __PYGMYNEBULA
        for( i = 0; i < PYGMYMAXPINS; i++ ){
            if( isStringSame( PYGMYNEBULAPINS[ i ].String, buffer ) ){
                return( PYGMYNEBULAPINS[ i ].Value );
            } // if     
        } // for
    #endif // __PYGMYNEBULA
    // Pin isn't an alias, check for port letter and pin number
    if( s.replaceChars( "pP", ' ' ) ){
        if( s.replaceChars( "aA", ' ' ) ){
            pin = 0;
        } else if( s.replaceChars( "bB", ' ' ) ){
            pin = 16;
        } else if( s.replaceChars( "cC", ' ' ) ){
            pin = 32;
        } else if( s.replaceChars( "dD", ' ' ) ){
            pin = 48;
        } else if( s.replaceChars( "eE", ' ' ) ){
            pin = 64;
        } else if( s.replaceChars( "fF", ' ' ) ){
            pin = 80;
        } else{
            return( 0xFF );
        } // else
        return( pin + s.convertStringToInt( ) );
    } // if

    return( 0xFF );
}


/*u32 convertDateStringToSeconds( char *buffer )
{
    // This function converts a Date/Time string to seconds
    //"11/20/2003 12:48:00"
    PYGMYTIME Time;
    u32 Seconds;
    char *TmpString, TmpBuffer[8];

    TmpString = TmpBuffer;
    TmpString = strtok ( buffer, ": /" );
    if( !TmpString ){
        return( FALSE );
    } // if
    Time.Month = convertStringToInt( TmpString );
    TmpString = strtok ( NULL, ": /" );
    if( !TmpString ){
        return( FALSE );
    } // if
    Time.Day = convertStringToInt( TmpString );
    TmpString = strtok ( NULL, ": /" );
    if( !TmpString ){
        return( FALSE );
    } // if
    Time.Year = convertStringToInt( TmpString );
    TmpString = strtok ( NULL, ": /" );
    if( !TmpString ){
        return( FALSE );
    } // if
    Time.Hour = convertStringToInt( TmpString );
    TmpString = strtok ( NULL, ": /" );
    if( !TmpString ){
        return( FALSE );
    } // if
    Time.Minute = convertStringToInt( TmpString );
    TmpString = strtok ( NULL, ": /" );
    if( !TmpString ){
        return( FALSE );
    } // if
    Time.Second = convertStringToInt( TmpString );

    Seconds = convertSystemTimeToSeconds( &Time );
    
    return( Seconds );
}*/

/*void copyBuffer( char *from, char *to, u16 length )
{
    u32 i;

    for( i = 0; i < length; i++ ){
        *(to++) = *(from++);
    } // for
}*/

/*char *stripLeadingChars( char *buffer, u8 *chars )
{    
    for( ; *buffer; buffer++){
       if(!isCharInString( *buffer, chars ) ){
            break;
        } // if
    } // for
    
    return( buffer );
}*/

/*
u8 *splitString( u8 *ucString, u8 ucChar, s16 sCount )
{
    // Returns pointer to second half of split string
    u16 i, uiLen;
    s16 sInc;
    //u8 *ucStart;

    uiLen = len( ucString );
    //ucStart = ucString;
    sInc = -1;
    if( sCount < 0 ){
        sInc = 1;
        ucString = ucString + uiLen;
    } // if
    for( i = 0; i < uiLen && sCount != 0; i++ ){
        if( *ucString == ucChar ){
            sCount += sInc;
        } // if
        if( sInc < 0 ){
            --ucString;
        } else{
            ++ucString;
        } // else
    } // for
    
    return( ucString );
}
*/

void convertU16ToBuffer( u16 data, char *buffer, u8 endian )
{
    if( endian == BIGENDIAN ){
        *(buffer++) = (char)((u16) data >> 8 );
        *(buffer++) = (char)data;
    } else {
        *(buffer++) = (char)data;
        *(buffer++) = (char)((u16) data >> 8 );
    } // else
}

void convertU32ToBuffer( u32 data, char *buffer, u8 endian )
{
    if( endian == BIGENDIAN ){
        *(buffer++) = (char)((u32)data >> 24 );
        *(buffer++) = (char)((u32)data >> 16 );
        *(buffer++) = (char)((u32)data >> 8 );
        *(buffer++) = (char)data;
    } else {
        *(buffer++) = (char)data;
        *(buffer++) = (char)((u32)data >> 8 );
        *(buffer++) = (char)((u32)data >> 16 );
        *(buffer++) = (char)((u32)data >> 24 );
    } // else
}

u16 convertBufferToU16( char *buffer, u8 endian )
{
    u16 data;

    if( endian == BIGENDIAN ){
        data = (u16)buffer[ 0 ] << 8;
        data |= (u16)buffer[ 1 ];
    } else {
        data = (u16)buffer[ 0 ];
        data |= (u16)buffer[ 1 ] << 8;
    } // else

    return( data );
}

u32 convertBufferToU32( u8 *buffer, u8 endian )
{
    u32 data;

    if( endian == BIGENDIAN ){
        data = (u32)buffer[ 0 ] << 24;
        data |= (u32)buffer[ 1 ] << 16;
        data |= (u32)buffer[ 2 ] << 8;
        data |= (u32)buffer[ 3 ]; 
    } else{
        data = (u32)buffer[ 0 ];
        data |= (u32)buffer[ 1 ] << 8;
        data |= (u32)buffer[ 2 ] << 16;
        data |= (u32)buffer[ 3 ] << 24;
    } // else

    return( data );
}

bool isCharSameIgnoreCase( char c1, char c2 )
{
    PygmyString s;

    if( s.convertCharToUpper( c1 ) == s.convertCharToUpper( c2 ) ){
       return( true );
    } // if

    return( false );
}

bool areStringsSameIgnoreCase( char *s1, char *s2 )
{
    if( strlen( s1 ) != strlen( s2 ) )
        return ( false );
    
    for( ; *s1; ){
        if( !isCharSameIgnoreCase( *(s1++), *(s2++) ) ){
            return( false );
        } // if
    } // for
    
    return( true );
}

bool areStringsSame( const char *s1, const char *s2 )
{
    
}

bool isStringSame( char *s1, char *s2 )
{
    if( strlen( s1 ) != strlen( s2 ) )
        return ( false );
        
    for( ; *s1 ; ){
        if( *(s1++) != *(s2++) ){
            return( false );
        } // if
    } // for
    
    return( true );
}

/*
u8 getAllParameters( char *Buffer, PYGMYPARAMLIST *Parameters ) 
{
    // This function breaks a buffer of chars into an array of strings
    // ParamArray is allocated by this function and must be freed by the calling function
    // parameters are separated by double quotes and spaces
    u32 i, ii, BufferLen;
    u8 Quotes;
    
    Parameters->SwitchCount = 0;
    Parameters->ParamCount = 0;
    Quotes = FALSE;
    BufferLen = strlen( Buffer );

    if( BufferLen < 1 ){
        return( FALSE );
    } // if
    Parameters->Switches = malloc( sizeof( void * ) );
    if( !Parameters->Switches ){
        return( FALSE );
    } // if
    Parameters->Params = malloc( sizeof( void * ) );
    if( !Parameters->Params ){
        free( Parameters->Switches );
        return( FALSE );
    } // if
    
    for( i = 0; i < BufferLen; i++ ){
        // Skip spaces until a parameter or name is found
        // Parameters start with a - or --
        if( Buffer[ i ] == '-' ){
            // This is a parameter
            ++i;
            if( Buffer[ i ] == '-' ){
                ++i; // This parameter is prefixed with --
            } // if
            Parameters->Switches = realloc( Parameters->Switches, ( Parameters->SwitchCount + 1 ) * 4 );
            if( !Parameters->Switches ){
                freeParameterList( Parameters );
                return( FALSE );
            } // if
            
            Parameters->Switches[ Parameters->SwitchCount ] = malloc( 2 );
            if( !Parameters->Switches[ Parameters->SwitchCount ] ){
                // If memory allcoation failed, cleanup and return
                freeParameterList( Parameters );
                return( FALSE );
            } // if
            
            // copy the parameter, break on whitespace or end of buffer
            for( ii = 0; ii < BufferLen && Buffer[ i ] != ' ' && Buffer[ i ] != ','; ii++, i++ ){
            //for( ii = 0; ii < BufferLen && Buffer[ i ] != ' '; ii++, i++ ){
                // allocate another byte for the next char
                Parameters->Switches[ Parameters->SwitchCount ] = realloc( Parameters->Switches[ Parameters->SwitchCount ], 1 + ( ii + 2 ) );
                // append the char
                Parameters->Switches[ Parameters->SwitchCount ][ ii ] = Buffer[ i ];
            } // for
            Parameters->Switches[ (Parameters->SwitchCount)++ ][ ii ] = '\0'; // Terminate string and increment parameter counter
        } else if( Buffer[ i ] != ' ' &&  Buffer[ i ] != ',' ){
            // This is a name
            if( Buffer[ i ] == '\"' ){
                Quotes = TRUE;
                ++i;
            } else{
                Quotes = FALSE;
            } // else
            Parameters->Params = realloc( Parameters->Params, ( Parameters->ParamCount + 1 ) * 4 );
            Parameters->Params[ Parameters->ParamCount ] = malloc( 2 );
            if( !Parameters->Params || !Parameters->Params[ Parameters->ParamCount ] ){
                // memory allocation failed, cleanup and return with error
                freeParameterList( Parameters );
                return( FALSE );
            } // if
            // Copy the name
            for( ii = 0; ii < BufferLen; ii++, i++ ){
                Parameters->Params[ Parameters->ParamCount ] = realloc( Parameters->Params[ Parameters->ParamCount ], ii + 3 );
                if( !Parameters->Params[ Parameters->ParamCount ] ){
                    freeParameterList( Parameters );
                    return( FALSE );
                } // if
                
                if( Quotes ){
                    if( Buffer[ i ] == '\"' ){
                        ++i;
                        Quotes = FALSE;
                        break; // The end of string has been reached, terminate the string and break
                    } // if
                    if( Buffer[ i ] == '\\' ){
                        ++i;
                        if( Buffer[ i ] == 'r'){
                            //print( COM3, "\rFound Escape Sequence: Carriage Return" );
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0D;  // carriage return escape sequence
                        } else if( Buffer[ i ] == 't' ){ 
                            //print( COM3, "\rFound Escape Sequence: Tab" );
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x09;  // horizontal tab escape sequence
                        } else if( Buffer[ i ] == 'n' ){          
                            //print( COM3, "\rFound Escape Sequence: Newline" );
                            Parameters->Params[ Parameters->ParamCount ][ ii++ ] = 0x0D;// For a newline, add a carriage return followed by a linefeed
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0A;
                            ++BufferLen; // Increase BufferLen to accommodate the additional character
                        } else if( Buffer[ i ] == 'l' ){
                            //print( COM3, "\rFound Escape Sequence: Line Feed" );
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0A;  // line feed escape sequence
                        } else if( Buffer[ i ] == 'f' ){
                            //print( COM3, "\rFound Escape Sequence: Form Feed" );
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0C;  // form feed/page break escape sequence
                        } else if( Buffer[ i ] == '\"' ){
                            //print( COM3, "\rFound Escape Sequence: Double Quote" );
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x22;  // double quotes escape sequence
                        } else if( Buffer[ i ] == '\'' ){
                            //print( COM3, "\rFound Escape Sequence: Single Quote" );
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x27;  // single quotes escape sequence
                        } // else if
                        continue;
                    } // if
                } else if( Buffer[ i ] == ' ' ){
                    break; // The end of string has been reached, terminate the string and break
                } // else
                Parameters->Params[ Parameters->ParamCount ][ ii ] = Buffer[ i ];
            } // for
            Parameters->Params[ Parameters->ParamCount++ ][ ii ] = '\0'; // Terminate string
        } // else if
    } // for

    return( TRUE );
}

void freeParameterList( PYGMYPARAMLIST *Parameters )
{
    u32 i;
  
    return;
    if( Parameters->Params ){
        for( i = 0; i < Parameters->ParamCount; i++ ){ 
            if( Parameters->Params[ i ] ){
                free( Parameters->Params[ i ] );
            } // if
        } // for
        free( Parameters->Params );
    } // if
    if( Parameters->Switches ){
        for( i = 0; i < Parameters->SwitchCount; i++ ){
            if( Parameters->Switches[ i ] ){
                free( Parameters->Switches[ i ] );
            } // if
        } // for
        free( Parameters->Switches );
    } // if
    
}

u16 getAllSubStrings( char *buffer, char *strings[], u16 length, u8 mode )
{
    u16 i;
    u8 *ucSub;

    // First clear string buffer to prevent memory errors later
    for( i = 0; i < length; i++ ){
        strings[ i ] = NULL;
    } // for
    ucSub = getNextSubString( buffer, mode );
    for( i = 0; i < length && ucSub; i++ ){
        strings[ i ] = ucSub;
        ucSub = getNextSubString( NULL, mode );
    } // for

    return( i );
}

u8 *getNextSubString( char *buffer, u8 mode )
{
    static char *index = NULL;
    static char *bufferCopy = NULL;
    char *start;

    if( buffer ){
        if( bufferCopy ){
            free( bufferCopy );
        } // if
        bufferCopy = malloc( 1 + len( buffer ) );
        if( !bufferCopy ){
            return( NULL );
        } // if
        //print( STDIO, "\nucBuffer: %s", ucBuffer );
        copyString( buffer, bufferCopy );
        ucIndex = bufferCopy;
        
    } // if
    if( index == NULL ){
        return( NULL );
    } // if
    
    for( ; *index;  ){
        if( ( isWhitespace( *index ) ) || 
            ( (mode & FILESEPARATORS) && isFileSeparator( *index ) ) ||
            ( (mode & PUNCT) && isPunctuation( *index ) ) ||
            ( (mode & SEPARATORS) && isSeparator( *index ) ) ||
            ( (mode & QUOTES) && isQuote( *index ) ) ||
            ( (mode & COMMA) && *ucIndex == ',' ) ){
            ++index;
            continue;
            } // if
        start = index;
        
        for( ; *index ; ){
            if( ( (mode & WHITESPACE) && isWhitespace( *index ) ) || 
                ( (mode & FILESEPARATORS) && *index == '/' ) ||
                ( (mode & PUNCT) && isPunctuation( *index ) ) ||
                //( (ucMode & PUNCT) && isFileSeparator( *ucIndex ) ) ||
                ( (mode & SEPARATORS) && isSeparator( *index ) ) ||
                ( (mode & QUOTES) && isQuote( *index ) ) ||
                ( (mode & NEWLINE) && isNewline( *index ) ) ||
                ( (mode & COMMA) && *index == ',' ) ||
                ( (mode & ALPHA) && isAlpha( *index ) ) ){
                break;
            } // if
            ++index;
        } // for
        if( *index ){
            *(index++) = '\0';
        } else{
            index = NULL;
        } // else

        return( start );
    } // for
    
    return( NULL );
}*/
/*
u16 replaceChars( char *buffer, const char *chars, char c )
{
    u16 i, replaced;
    
    for( replaced = 0; *buffer ; ){
        for( i = 0; *(chars+i); i++ ){
            if( *buffer == *(chars+i) ){
                ++replaced; 
                *buffer = c;
                break;
            } // if
        } // for
        ++ucBuffer;
    } // for
    
    return( replaced );
}
*/
u8 convertCharToInt( char c )
{
    if( c > 47 && c < 58 ){
        return( c - 48 );
    } else if( c > 64 && c < 71 ){
        return( c - 55 );
    } else if( c > 96 && c < 103 ){
        return( c - 87);
    }
    
    return( 0 );   
}

u8 convertIntToChar( char ucData )
{
    
}

char getLastChar( char *buffer )
{ 
    return( buffer[ strlen( buffer ) - 1 ] );
}

u8 replaceLastChar( char *buffer, char c, char newChar )
{
    // Warning! Do not use with constant strings!
    // Replaces last occurance of ucChar with ucNewChar
    // Returns location of char
    u16 i;

    i = strlen( buffer );
    for( ; i; ){
        if( buffer[ --i ] == c ){
            buffer[ i ] = newChar;
            break;
        } // if
    } // for

    return( i );
}

/*
u8 convertBase64ToCode( char c )
{
    // This function intentionally avoids the normal for loop
    // since there are two common alphabets for Base64 and a
    // for loop to handle both is significantly less efficient
    // Alphabet 1 = A-Z,a-z,0-9,+,/,=
    // Alphabet 2 = A-Z,a-z,0-9,-,_,=
    // The = char signifies padding, input len must be divisible by 3
    // and output len must be divisible by 4
    if( c > 64 && c < 91 ){ // A-Z
        return( c - 65 );
    } else if( c > 96 && c < 123 ){ // a-z
        return( c - 71 );
    } else if( c > 47 && c < 58 ){ // 0-9
        return( c + 4 );
    } else if( c == '+' || c == '-' ){
        return( 62 );
    } else if( c == '/' || c == '_' ){
        return( 63 );
    } 

    return( 64 );
}

u16 convertHexEncodedStringToBuffer( char *s, char *buffer )
{
    // HEX Encoded ASCII string to binary eqivalent
    // Use: ( "01020A", ptr ) , ptr[ 0 ] = 1, ptr[ 1 ] = 2, ptr[ 3 ] = 10
    // Note that input buffer may be used as output buffer since input is always
    //  twice the length of the output
    u16 uiLen;
    u8 ucByte;

    for( uiLen = 0; *ucString; ){
        ucByte = convertHexCharToInteger( *(s++) ) * 16;
        ucByte += convertHexCharToInteger( *(s++) );
        buffer[ uiLen++ ] = ucByte;
    }
        
    return( uiLen );
}*/

/*
u16 convertHexCharToInteger( char c )
{
    u16 i;

    for( i = 0; i < 16 && c != PYGMYHEXCHARS[ i ]; i++ )
        ;
    
    return( i );
}

u16 convertBase64ToBuffer( char *bufferIn, char *bufferOut )
{
    u16 i, uiPad, uiLen;
    u32 ulValue;
    
    for( uiLen = 0; *bufferIn ; ){
        for( i = 0, ulValue = 0, uiPad = 0; i < 4 && *bufferIn; i++ ){
            if( convertBase64ToCode( *bufferIn ) == 64 )
                ++uiPad;
            else
                ulValue +=  ( u32)convertBase64ToCode( *bufferIn ) << ( ( 3 - i ) * 6 ) ;
            ++bufferIn;
        }
        for( i = 0; i < 3-uiPad; i++, uiLen++ ){
            *(bufferOut++) = PYGMYBASE64CHARS[ (u8)(0x000000FF & ( ulValue >> ( ( 2 - i ) * 8 ) ) ) ];
        }
    }
    
    return( uiLen );
}

void convertBase64ToString( char *buffer, char *s )
{
    u16 i, uiPad;
    u32 ulValue;
    
    for( ; *buffer ; ){
        for( i = 0, ulValue = 0, uiPad = 0; i < 4 && *buffer; i++ ){
            if( convertBase64ToCode( *buffer ) == 64 )
                ++uiPad;
            else
                ulValue +=  ( u32)convertBase64ToCode( *buffer ) << ( ( 3 - i ) * 6 ) ;
            ++buffer;
        }
        for( i = 0; i < 3-uiPad; i++){
            *(s++) = PYGMYBASE64CHARS[ (u8)(0x000000FF & ( ulValue >> ( ( 2 - i ) * 8 ) ) ) ];
        }
    }
    *s = '\0';
}

void convertStringToBase64( char *s, char *ucBase64 )
{
    u16 i, uiPad, uiLen;
    u32 ulValue;
    
    uiLen = len( s );
    for( ; *s; ){
        for( i = 0, ulValue = 0, uiPad = 0; i < 3; i++){
            if( !(*s) ){
                uiPad = 1;
                break;
            }
            ulValue += ((u32)*(s++) << ( ( 2 - i ) * 8 ) );
        }
        if( uiPad )
            uiPad = 3 - ( uiLen % 3 );
        for( i = 0; i < 4 - uiPad; i++ ){
            *(ucBase64++) = PYGMYBASE64CHARS[ (u8)(0x0000003F & ( ulValue >> ( ( 3 - i ) * 6 ) ) ) ];
        }
        for( ; i < 4; i++ ){
            *(ucBase64++) = '=';
        }
    }
    *ucBase64 = '\0';
}*/