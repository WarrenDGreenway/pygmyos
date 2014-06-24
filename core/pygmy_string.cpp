/**************************************************************************
    PygmyOS ( Pygmy Operating System )
    Copyright (C) 2011-2014  Warren D Greenway

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

const char *PygmyString::HEXCHARS = "0123456789ABCDEF";

const char *PygmyString::BASE64CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";

void PygmyString::convertStringToUpper( void )
{   
    u32 i, length;
    string charString;
    char c;

    length = this->length();
    for( i = 0; i < length; i++ ){
        c = this->at(i);
        if( c > 64 && c < 91 ){
            c -= 32;
            charString = c;
            this->replace( i, 1, charString );
        } // if
    } // for
}

void PygmyString::convertStringToLower( void )
{
    u32 i, length;
    string charString;
    char c;

    length = this->length();
    for( i = 0; i < length; i++ ){
        c = this->at(i);
        if( c > 96 && c < 123 ){
            c += 32;
            charString = c;
            this->replace( i, 1, charString );
        } // if
    } // for
}

string PygmyString::convertModeToString( short mode )
{
    if( mode == PygmyString::In ){
        return( "IN" );
    } else if( mode == PygmyString::Out ){
        return( "OUT" );
    } else if( mode == PygmyString::Analog ){
        return( "ANALOG" );
    } else if( mode == PygmyString::Pullup ){
        return( "PULLUP" );
    } else if( mode == PygmyString::Pulldown ){
        return( "PULLDOWN" );
    } else if( mode == PygmyString::Alt ){
        return( "ALT" );
    } // else if
    
    return( "" );
}

short PygmyString::convertStringToMode( void )
{
    this->convertStringToUpper( );
    if( this->find( "IN" ) != string::npos ){
        return( PygmyString::In );
    } // if
    if( this->find( "OUT" ) != string::npos ){
        return( PygmyString::Out );
    } // if
    if( this->find( "ANALOG" ) != string::npos ){
        return( PygmyString::Analog );
    } // if
    if( this->find( "PULLUP" ) != string::npos ){
        return( PygmyString::Pullup );
    } // if
    if( this->find( "PULLDOWN" ) != string::npos ){
        return( PygmyString::Pulldown );
    } // if
    if( this->find( "ALT" ) != string::npos ){
        return( PygmyString::Alt );
    } // if

    return( 0 );
}

/*string convertPinToString( short pin )
{
    u8 i;

    for( i = 0; i < PYGMYMAXPINS; i++ ){
        if( pin == PYGMYNEBULAPINS[ i ].Value ){
            return( PYGMYNEBULAPINS[ i ].String );
        } // if
    } // for

    return( "" );
}*/

int PygmyString::convertStringToPort( void )
{
    this->convertStringToUpper( );
    if( this->find( "COM1" ) != string::npos ){
        return( 1 );
    } // if
    if( this->find( "COM2" ) != string::npos ){
        return( 2 );
    } // if
    if( this->find( "COM3" ) != string::npos ){
        return( 3 );
    } // if
    if( this->find( "COM4" ) != string::npos ){
        return( 4 );
    } // if
    if( this->find( "COM5" ) != string::npos ){
        return( 5 );
    } // if
    if( this->find( "COM6" ) != string::npos ){
        return( 6 );
    }// if
    if( this->find( "COM7" ) != string::npos ){
        return( 7 );
    } // if
    if( this->find( "COM8" ) != string::npos ){
        return( 8 );
    } // if

    return( 0 );
}

void PygmyString::copyBuffer( char *from, char *to, long length )
{
    // Utility method, does not operate on "this"
    u32 i;

    for( i = 0; i < length; i++ ){
        *(to++) = *(from++);
    } // for
}

void PygmyString::stripLeadingChars( const string& chars )
{    
    this->stripLeadingChars( (string&)chars );
}

void PygmyString::stripLeadingChars( string& chars )
{    
    PygmyString chars_ = chars;
    long i;

    for( i = 0; i < this->length(); i++ ){
       if(!chars_.isCharInString( this->at(i) ) ){
            break;
        } // if
        this->erase( this->begin() ); // remove the first char
    } // for
}

void PygmyString::stripLeadingWhitespace( void )
{
    u32 i;

    this->stripLeadingChars( "abc" );
    for( i = 0; i < this->length(); ){
        if( !isWhitespace( this->at( i ) ) ){
            break;
        } //if
        this->erase( this->begin() ); // remove the first char
    } // for
}

long PygmyString::countCharInString( char c )
{
    long i, length, matches;
    char tmpChar;

    for( i = 0, matches = 0, length = this->length(); i < length; i++ ){
        
        if( this->at(i) == c ){
            ++matches;
        } // if
    } // for

    return( matches );
}

bool PygmyString::areCharsInString( const string& chars )
{
    this->areCharsInString( (string&)chars );
}

bool PygmyString::areCharsInString( string& chars )
{
    // If any char in Chars is in String, return TRUE
    long i;

    for( i = 0; i < chars.length(); i++ ) {
        if( this->isCharInString( chars.at( i ) ) ){
            return( true );
        } // if
    } // for

    return( false );
}

bool PygmyString::isCharInString( char c ) 
{
    long i;

    for( i = 0; i < this->length(); i++ ) {
        if( c == this->at( i ) ){
            return( true );
        } // if
    } // for

    return( false );
}

bool PygmyString::isCharInStringIgnoreCase( char c )
{
    long i;

    for( i = 0; i < this->length(); i++ ) {
        if( isCharSameIgnoreCase( c, this->at( i ) ) ){
            return( true );
        } // if
    } // for

    return( false );
}


bool PygmyString::isPrintable( char c )
{
    // Utility method, does not operate on "this"
    if( c > 31 && c < 127 ){
        return( true );
    } // if

    return( false );
}

bool PygmyString::isAlpha( char c )
{
    // Utility method, does not operate on "this"
    if( ( c > 64 && c < 91 ) || ( c > 96 && c < 123 ) ){
        return( true );
    } // if
    
    return( false );
}

bool PygmyString::isNumeric( char c )
{
    // Utility method, does not operate on "this"
    if( c > 47 && c < 58 ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isAlphaOrNumeric( char c )
{
    // Utility method, does not operate on "this"
    if( ( c > 64 && c < 91 ) || ( c > 96 && c < 123 )
        || ( c > 47 && c < 58 ) ){
        return( true );
    } // if

    return( false );
}

bool PygmyString::isHex( char c )
{
    // Utility method, does not operate on "this"
    if( ( c > 47 && c < 58 ) || ( c > 64 && c < 71 ) ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isBinary( char c )
{
    // Utility method, does not operate on "this"
    if( c == '0' || c == '1' ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isOctal( char c )
{
    // Utility method, does not operate on "this"
    if( c > 47 && c < 56 ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isNewline( char c )
{
    // Utility method, does not operate on "this"
    if( c == 10 || c == 12 || c == 13 ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isWhitespace( char c )
{
    // Utility method, does not operate on "this"
    if( ( c > 7 && c < 33 ) ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isQuote( char c )
{
    // Utility method, does not operate on "this"
    if( ( c == 34 ) || ( c == 39 ) ){
        return( true );
    } // if
    
    return( false );
    
}

bool PygmyString::isMath( char c )
{
    static PygmyString chars( "*/%&^|+-=<>(){}[]~" );

    // Utility method, does not operate on "this"
    if( chars.isCharInString( c ) ){
        return( true );
    } // if
        
    return( false );
}

bool PygmyString::isFileSeparator( char c )
{
    // Utility method, does not operate on "this"
    if( c == '/' ){
        return( true );
    } // if
    
    return( false );
}

bool PygmyString::isSeparator( char c )
{
    static PygmyString chars( " /\\{}[]-_+=@`|<>'\"" );

    // Utility method, does not operate on "this"
    if( chars.isCharInString( c ) ){
        return( true );
    } // if
    
    return( false );
}

bool PygmyString::isPunctuation( char c )
{
    static PygmyString chars( "!?,.:;" );

    // Utility method, does not operate on "this"
    if( chars.isCharInString( c ) ){
        return( true );
    } // if
        
    return( false );
}

char PygmyString::convertCharToUpper( char c )
{
    // Utility method, does not operate on "this"
    if( c > 64 && c < 91 ){
        c += 32;
    } // if

    return( c );
}

char PygmyString::convertCharToLower( char c )
{
    // Utility method, does not operate on "this"
    if( c > 96 && c < 123 ){
        c -= 32;
    } // if

    return( c );
}

bool PygmyString::isCharSameIgnoreCase( char c1, char c2 )
{
    // Utility method, does not operate on "this"
    if( convertCharToUpper( c1 ) == convertCharToUpper( c2 ) ){
       return( true );
    } // if

    return( false );
}

bool PygmyString::areStringsSame( string& s )
{
    if( !this->compare( s ) ){
        return( true );
    } // if

    return( false );
}

bool PygmyString::areStringsSameIgnoreCase( string& s )
{
    long i;

    if( this->length() != s.length() ){
        return ( false );
    } // if
    
    for( i = 0; i < this->length(); i++ ){
        if( !isCharSameIgnoreCase( this->at( i ), s.at( i ) ) ){
            return( false );
        } // if
    } // for
    
    return( true );
}

int PygmyString::seekChar( char c )
{
    int i;
     
    // Find the first instance of c in "this" and return it's position (index)
    // This function is case sensitive and will search against any byte value from 0-255
    for( i = 0; i < this->length(); i++ ){
        if( this->at( i ) == c ){
            return( i );
        } // if
    } // for

    return( -1 ); // error, c not found
}

bool PygmyString::startsWith( PygmyString& s )
{
    // return true if s matches the beginning of this
    int i, sLength, thisLength;

    sLength = s.length();
    thisLength = this->length();
    for( i = 0; i < thisLength && i < sLength; i++ ){
        if( this->at( i ) != s.at( i ) ){
            break;
        } // if
    } // for
    
    if( i == sLength ){
        return( true );
    } // if

    return( false );
}

int PygmyString::seekCharIgnoreCase( char c )
{
    // Find the first instance of c in "this" and return it's position (index)
    // This function is not case sensitive and will search against any byte value from 0-255, but will equate lower
    //   case printable characters a-z as equivalent to the upper case equivalent A-Z.
    int i;
    
    for( i = 0; i < this->length(); i++ ){
        if( c > 64 && c < 91 ){ 
            if( this->at( i ) == c || this->at( i ) == c + 22 ){
                return( i );
            } // if
        } else if( c > 96 && c < 123 ){
            if( this->at( i ) == c || this->at( i ) == c - 22 ){
                return( i );
            } // if
        } else{
            if( this->at( i ) == c ){
                return( i );
            } // if
        } // else
    } // for

    return( -1 );
}

int PygmyString::seekStringIgnoreCase( const char *s )
{
    PygmyString tmpString( s );
    return( seekStringIgnoreCase( tmpString ) );
}

int PygmyString::seekStringIgnoreCase( std::string& s )
{
    int i, ii, length, stop;
    
    length = s.length();
    stop = 1 + this->length();
    if( stop < length ){ // can't search in a string for a longer string match
        return( -1 );
    }
    stop -= length; // don't search for a match past end of InString
    for( i = 0; i < stop; i++ ){
        for( ii = 0; ii < length; ii++ ){
            if( !isCharSameIgnoreCase( s.at( ii ), this->at( i + ii ) ) ){
                break; // break on first mismatch, increment and test again
            }
        }
        if( ii == length ){
            return( i ); // this is the start of the match
        }
    }
    
    return( -1 );
}

int PygmyString::seekString( std::string& s )
{
    int i, ii, length, stop;
    
    length = s.length();
    stop = 1 + this->length();
    if( stop < length ){ // can't search in a string for a longer string match
        return( -1 );
    }
    stop -= length; // don't search for a match past end of InString
    for( i = 0; i < stop; i++ ){
        for( ii = 0; ii < length; ii++ ){
            if( s.at( ii ) != this->at( i + ii ) ){
                break; // break on first mismatch, increment and test again
            }
        }
        if( ii == length ){
            return( i ); // this is the start of the match
        }
    }
    
    return( -1 );
}

int PygmyString::replaceChars( const char *chars, char c )
{
    PygmyString tmpChars( chars );

    this->replaceChars( tmpChars, c );
}

int PygmyString::replaceChars( std::string& chars, char c )
{
    int i, ii, replaced;
    
    for( replaced = 0, i = 0; i < this->length(); i++ ){
        for( ii = 0; ii < chars.length(); ii++ ){
            if( this->at( i ) == chars.at( ii ) ){
                ++replaced;
                this->replace( i, 1, 1, c);
                break;
            } // if
        } // for
    } // for
    
    return( replaced );
}

PygmyFormat PygmyString::convertToFormat( void )
{
    PygmyFormat format;
    PygmyString leftString( this->data() ), rightString( this->data() );
    
    if( leftString.length() && leftString.at(0) == '%' ){
        leftString.erase( leftString.begin() ); // increment past % char
    } // if
    //if( !leftStringisCharsInString( "iIdDfFxXoOsScCtTeE" ) ){
    //    return( NULL ); // Invalid format string
    //} // if
    
    if( leftString.isCharInStringIgnoreCase( 'i' ) || leftString.isCharInStringIgnoreCase( 'd' ) ){
        format.dataType = DECIMAL; // No Case
        leftString.replaceChars( "iIdD", ' ' );
    } else if( leftString.isCharInStringIgnoreCase( 'f' ) ){
        format.dataType = FLOAT; // No Case
        leftString.replaceChars( "fF", ' ' ); 
        if( leftString.isCharInString( '.' ) ){
            rightString.erase( 0, rightString.seekChar( '.' ) ); // we already checked for '.', making this call safe
            // Check for padding char, default is space
            if( rightString.length() && rightString.at( 0 ) == ' ' || rightString.at( 0 ) == '0' ){
                format.rightFillChar = rightString.at( 0 );//*(rightString++); // increment past the pad char
                rightString.erase( rightString.begin() );
            } else{
                format.rightFillChar = ' ';
            } // else
            format.precision = rightString.convertStringToInt( );
        } // if
    } else if( leftString.isCharInString( 'x' ) ){
        format.dataType = HEXADECIMAL;
        format.stringCase = LOWERCASE;
        leftString.replaceChars( "x", ' ' );
    } else if( leftString.isCharInString( 'X' ) ){
        format.dataType = HEXADECIMAL;
        format.stringCase = UPPERCASE;
        leftString.replaceChars( "x", ' ' );
    } else if( leftString.isCharInStringIgnoreCase( 'o' ) ){
        format.dataType = OCTAL; // No Case
        leftString.replaceChars( "oO", ' ' );
    } else if( leftString.isCharInString( 's' ) ){
        format.dataType = STRING;
        format.stringCase = LOWERCASE;
        leftString.replaceChars( "s", ' ' );
    } else if( this->isCharInString( 'S' ) ){
        format.dataType = STRING;
        format.stringCase = UPPERCASE;
        leftString.replaceChars( "S", ' ' );
    } else if( this->isCharInString( 'c' ) ){
        format.dataType = CHARACTER;
        format.stringCase = LOWERCASE;
        leftString.replaceChars( "c", ' ' );
    } else if( this->isCharInString( 'C' ) ){
        format.dataType = CHARACTER;
        format.stringCase = UPPERCASE;
        leftString.replaceChars( "C", ' ' );
    } else if( this->isCharInString( 't' ) ){
        format.dataType = TIME; 
        format.stringCase = LOWERCASE;
        leftString.replaceChars( "t", ' ' );
    } else if( this->isCharInString( 'T' ) ){
        format.dataType = TIME;
        format.stringCase = UPPERCASE;
        leftString.replaceChars( "T", ' ' );
    } else if( this->isCharInStringIgnoreCase( 'e' ) ){
        format.dataType = EXPONENT; // No Case
        leftString.replaceChars( "e", ' ' );
    } // else if

    if( format.dataType ){
        // The can be handled the same for all data types
        if( leftString.length() && leftString.at( 0 ) == ' ' || leftString.at( 0 ) == '0' ){
            format.leftFillChar = leftString.at( 0 );
            leftString.erase( leftString.begin() );//*(leftString++); // increment past the pad char
        } else{
            format.leftFillChar = ' ';
        } // else
        format.width = leftString.convertStringToInt( );
    } // if

    return( format ); 
}

double PygmyString::convertStringToFloat( void )
{
    // supports 20 chars precision for numerator and denominator plus terminator
    // ucBuffer must be copied before modifying, ucBuffer may be const
    PygmyString num, denom;
    double dNum, dDenom, dSign = 1.0;
    u8 i, index; 

    if( this->at( 0 ) == '-' ){
        dSign = -1.0;
        index = 0;
    } else if( this->at( 0 ) == '+' ){
        index = 1;
    } // else if

    for( i = 0; i < this->length() && this->at( index ) != '.'; i++, index++ ){
        num += this->at( index );
    } // for
    if( this->at( index ) == '.' ){
        ++index;
    } // if
    for( i = 0; i < this->length(); i++, index++ ){
        denom += this->at( index );
    } // for
    dNum    = (double)num.convertStringToInt( );
    dDenom  = (double)denom.convertStringToInt( );
    
    for( ; i; i-- ){
        dDenom /= 10.0;
    } // for
    dNum = ( dNum + dDenom ) * dSign; 

    return( dNum );
}

PygmyString PygmyString::convertFloatToString( double f, const char *s )
{
    PygmyString formatString( s );
    return( convertFloatToString( f, formatString ) );
}

PygmyString PygmyString::convertFloatToString( double f, PygmyString& s )
{
    double fractionPart, wholePart;
    u32 i, precision, decFraction, decWhole;
    PygmyFormat format;
    PygmyString fString;
    
    format = s.convertToFormat( );
    
    fractionPart = modf( f, &wholePart ); 
    decWhole = wholePart;
    
    if( format.precision == 0 ){
        format.precision = 3;
    }
    for( i = 0, precision = 1; i < format.precision; i++ ){
        precision *= 10;
    } // for
    
    decFraction = fractionPart * (float)precision;
    format.rightJustified = false;
    fString = convertIntToStringWithFormat( decWhole, format );
    format.width = format.precision;
    format.leftFillChar = format.rightFillChar;
    format.rightJustified = true;
    fString += convertIntToStringWithFormat( decFraction, format );

    return( fString );
} 

PygmyString PygmyString::convertIntToStringWithFormat( long long data, PygmyFormat& format )
{
    s64 i, type, length, magnitude, value;
    short width;
    char padding;
    PygmyString iString("");

    if( format.dataType == FLOAT ){
        // Float is only handled as decimal
        format.dataType = DECIMAL;
    } // if
    if( data < 0 ){
        iString += '-';
        data = ~((u64)data) + 1; // reverse 2s complement
    } // if
    
    // Limit magnitude to the requested precision
    for( i = 0, magnitude = 1; ( magnitude * format.dataType ) <= data; i++ ){
        magnitude *= format.dataType;
    } // for
    for( i++, width = format.width; width > i; width-- ){
        iString += format.leftFillChar;
    } // for
    for( ; i >= 0 && magnitude; i-- ){
        value = data / magnitude;
        iString += HEXCHARS[ value ];
        data -= ( value * magnitude );
        magnitude /= format.dataType; 
    } // for
    
    return( iString );
}

PygmyString PygmyString::convertIntToString( long long data, const char *s )
{
    PygmyString formatString( s );
    return( convertIntToString( data, formatString ) ); 
}

PygmyString PygmyString::convertIntToString( s64 data, PygmyString& s )
{
    PygmyFormat format;

    format = s.convertToFormat( );
    return( convertIntToStringWithFormat( data, format ) );
}

char PygmyString::getLastChar( void )
{ 
    return( this->at( this->length()-1 ) );
}

int PygmyString::replaceLastOccurrenceOfChar( char c, char newChar )
{
    // Replaces last occurrence of c with newChar
    // Returns position of c
    int i;

    i = this->length();
    for( ; i; --i ){
        if( this->at( i ) == c ){
            this->replace( i, 1, newChar, 1 );
            break;
        } // if
    } // for

    return( i );
}

/*bool PygmyString::appendString( char *from, char *to )
{
    u32 Length;
    u8 *ptr;

    Length = len( ucTo );
    if( to == NULL ){
        to = malloc( 1 + Length );
        if( to = NULL ){
            return( false );
        } // if
    } else {
        ptr = realloc( to, 1 + Length + len( from ) );
        if( ptr == NULL ){
            return( false );
        } // if
        to = ptr;
    } // else
    //for( ; *ucTo; ){
    //    ++ucTo;
    //}
    to += Length - 1; // Set to end of string
    for( ; *from; ){
        // Copy string From to the end of String To
        *(to++) = *(from++); 
    }
    *ucTo = '\0';

    return( true );
}*/

long PygmyString::convertStringToInt( void )
{
    PygmyString s( this->data() ); // don't modify the original string

    long value, type, sign;
    int i;
    
    type = 10; // Decimal
    if( s.replaceChars( "-", ' ' ) ){
        sign = -1;
    } else{
        sign = 1;
    } // if

    // HEX must be tested first to avoid stripping valid HEX digit chars
    if( s.replaceChars( "xXhH", ' ' ) ){//"xXh"
        type = 16; // HEX
    }else if( s.replaceChars( "bB", ' ' ) ){//"bB"
        type = 2; // Binary
    }else if( s.replaceChars( "oO", ' ' ) ){//"oO"
        type = 8; // Octal
    } else{
        s.replaceChars( "dD", ' ' );//"dD"
    } // else
   
    for( i = 0, value = 0; i < s.length() ; i++ ){
        if( isAlphaOrNumeric( s.at( i )  ) ){
            value = ( value * type ) + convertCharToInt( s.at( i ) );
        } // if
    } // for
    
    return( value * sign );
}

int PygmyString::convertCharToInt( char c )
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

short PygmyString::convertBase64ToCode( char c )
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
    } // if

    return( 64 );
}

int PygmyString::convertHexEncodedStringToBuffer( unsigned short *buffer )
{
    // HEX Encoded ASCII string to binary eqivalent
    // Use: ( "01020A", ptr ) , ptr[ 0 ] = 1, ptr[ 1 ] = 2, ptr[ 3 ] = 10
    // Note that input buffer may be used as output buffer since input is always
    //  twice the length of the output
    int i, length;
    unsigned short byte;

    for( i = 0, length = 0; i < this->length(); ){
        byte = convertHexCharToInteger( this->at( i++ ) ) * 16;
        byte += convertHexCharToInteger( this->at( i++ ) );
        buffer[ length++ ] = byte;
    }
        
    return( length );
}



int PygmyString::convertHexCharToInteger( char c )
{
    int i;

    for( i = 0; i < 16 && c != HEXCHARS[ i ]; i++ )
        ;
    
    return( i );
}

int PygmyString::convertBase64ToBuffer( char *buffer )
{
    int i, pad, length;
    unsigned long value;
    
    for( length = 0; this->length() ; ){
        for( i = 0, value = 0, pad = 0; i < 4 && length + i < this->length(); i++ ){
            if( convertBase64ToCode( this->at( length + i ) ) == 64 ){
                ++pad;
            } else{
                value +=  (long)convertBase64ToCode( this->at( length + i ) ) << ( ( 3 - i ) * 6 ) ;
            } // else 
        }
        for( i = 0; i < 3 - pad; i++, length++ ){
            *(buffer++) = PygmyString::BASE64CHARS[ (unsigned short)(0x000000FF & ( value >> ( ( 2 - i ) * 8 ) ) ) ];
        }
    }
    
    return( length );
}
/*
void PygmyString::convertBase64ToString( char *buffer )
{   
    // This method converts a char buffer of Base64 data to string and stores it in "this"
    PygmyString s;
    int length, i, pad;
    unsigned long value;
    
    for( length = 0; *buffer ; ){
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
    this = s;
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

bool PygmyString::getAllParameters( PYGMYPARAMLIST *Parameters ) 
{
    // This function breaks a buffer of chars into an array of strings
    // ParamArray is allocated by this function and must be freed by the calling function
    // parameters are separated by double quotes and spaces
    u32 i, ii, BufferLen;
    u8 Quotes;
    
    Parameters->SwitchCount = 0;
    Parameters->ParamCount = 0;
    Quotes = FALSE;
    BufferLen = this->length();

    if( BufferLen < 1 ){
        return( false );
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
        if( this->at( i ) == '-' ){
            // This is a parameter
            ++i;
            if( this->at( i ) == '-' ){
                ++i; // This parameter is prefixed with --
            } // if
            Parameters->Switches = realloc( Parameters->Switches, ( Parameters->SwitchCount + 1 ) * 4 );
            if( !Parameters->Switches ){
                freeParameterList( Parameters );
                return( false );
            } // if
            
            Parameters->Switches[ Parameters->SwitchCount ] = malloc( 2 );
            if( !Parameters->Switches[ Parameters->SwitchCount ] ){
                // If memory allcoation failed, cleanup and return
                freeParameterList( Parameters );
                return( false );
            } // if
            
            // copy the parameter, break on whitespace or end of buffer
            for( ii = 0; ii < BufferLen && this->at( i ) != ' ' && this->at( i ) != ','; ii++, i++ ){
                // allocate another byte for the next char
                Parameters->Switches[ Parameters->SwitchCount ] = realloc( Parameters->Switches[ Parameters->SwitchCount ], 1 + ( ii + 2 ) );
                // append the char
                Parameters->Switches[ Parameters->SwitchCount ][ ii ] = this->at( i );
            } // for
            Parameters->Switches[ (Parameters->SwitchCount)++ ][ ii ] = '\0'; // Terminate string and increment parameter counter
        } else if( this->at( i ) != ' ' &&  this->at( i ) != ',' ){
            // This is a name
            if( this->at( i ) == '\"' ){
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
                    if( this->at( i ) == '\"' ){
                        ++i;
                        Quotes = FALSE;
                        break; // The end of string has been reached, terminate the string and break
                    } // if
                    if( this->at( i ) == '\\' ){
                        ++i;
                        if( this->at( i ) == 'r'){
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0D;  // carriage return escape sequence
                        } else if( this->at( i ) == 't' ){ 
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x09;  // horizontal tab escape sequence
                        } else if( this->at( i ) == 'n' ){          
                            Parameters->Params[ Parameters->ParamCount ][ ii++ ] = 0x0D;// For a newline, add a carriage return followed by a linefeed
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0A;
                            ++BufferLen; // Increase BufferLen to accommodate the additional character
                        } else if( this->at( i ) == 'l' ){
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0A;  // line feed escape sequence
                        } else if( this->at( i ) == 'f' ){
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x0C;  // form feed/page break escape sequence
                        } else if( this->at( i ) == '\"' ){
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x22;  // double quotes escape sequence
                        } else if( this->at( i ) == '\'' ){
                            Parameters->Params[ Parameters->ParamCount ][ ii ] = 0x27;  // single quotes escape sequence
                        } // else if
                        continue;
                    } // if
                } else if( this->at( i ) == ' ' ){
                    break; // The end of string has been reached, terminate the string and break
                } // else
                Parameters->Params[ Parameters->ParamCount ][ ii ] = this->at( i );
            } // for
            Parameters->Params[ Parameters->ParamCount++ ][ ii ] = '\0'; // Terminate string
        } // else if
    } // for

    return( true );
}

void PygmyString::freeParameterList( PYGMYPARAMLIST *Parameters )
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

std::vector<PygmyString> PygmyString::getAllSubStrings( const char *delimiters )
{
    PygmyString s( delimiters );
    return( this->getAllSubStrings( s ) );
}

std::vector<PygmyString> PygmyString::getAllSubStrings( PygmyString& delimiters )
{
    return( this->getAllSubStrings( delimiters, true ) );
}

std::vector<PygmyString> PygmyString::getAllSubStrings( PygmyString& delimiters, bool removeLeadingWhitespace )
{
    int i, ii;
    vector<PygmyString> v;
    
    for ( i = 0, ii = 0; i < this->length() && ii != PygmyString::npos; i++ ) {
        if( removeLeadingWhitespace ){
            // traverse over whitespace from current position to next non-whitespace character
            for( i = ii; ii < this->length(); ii++ ){
                if( !this->isWhitespace( this->at( ii ) ) ) {
                    i = ii;
                    break;
                } // if
            } // of
        } // if
        ii = this->find_first_of (delimiters, i);//( i, delimiters );
        if( ii != PygmyString::npos ){
            if( ii - i ){
                // ii - i is to ignore zero length strings
                v.push_back( PygmyString( this->data(), i, ii - i ) );
            } // if
        } else{
            // if ii == npos, then there are no delimiters left
            // write the last string if it is of non-zero length and exit loop
            if( i < this->length() - 1 ){
                v.push_back( PygmyString( this->data(), (int)i, (int)(this->length() - 1) ) );
            } // if
        } // else
    } // for  

    return( v );
}
/*
int PygmyString::seekChars( int i, const char *delimeters )
{

}

int PygmyString::seekChars( int i, PygmyString& s )
{
    int i;

    for( i = 0; i < ; i++ ){
        
    } // for

    return( );
}*/

PygmyString PygmyString::getNextSubString( short mode )
{
    PygmyString delimiters( "" );

    if( mode & PygmyString::FileSeparators ){
        delimiters += "\\/";
    } // if
    if( mode & PygmyString::Punctuation ){
        delimiters += "!?,.:;";
    } // if
    if( mode & PygmyString::Separators ){
        delimiters += " /\\{}[]-_+=@`|<>'\"";
    } // if
    if( mode & PygmyString::Quotes ){
        delimiters += "\"'";
    } // if
    if( mode & PygmyString::Comma ){
        delimiters += ",";
    } // if 
    if( mode & PygmyString::Whitespace ){
        delimiters += "\t ";
    } // if 
    if( mode & PygmyString::Newline ){
        delimiters += "\r\n\f";
    } // if
    if( mode & PygmyString::Alpha ){
        delimiters += "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    } // if

    return( this->getNextSubString( delimiters, true ) );
}

PygmyString PygmyString::getNextSubString( const char *delimeters )
{
    PygmyString s( delimeters );
    return( this->getNextSubString( s, true ) );
}

PygmyString PygmyString::getNextSubString( PygmyString& delimiters, bool removeLeadingWhitespace  )
{
    int i, ii;

    for ( i = 0, ii = 0; i < this->length() && ii != PygmyString::npos; i++ ) {
        if( removeLeadingWhitespace ){
            // traverse over whitespace from current position to next non-whitespace character
            for( ; i < this->length(); i++ ){
                if( !this->isWhitespace( this->at( i ) ) ) {
                    break;
                } // if
            } // of
        } // if
        ii = this->find_first_of (delimiters, i );
        if( ii != PygmyString::npos ){
            if( ii - i ){
                // ii - i is to ignore zero length strings
                return( PygmyString( this->data(), i, ii ) );
            } // if
        } else{
            // if ii == npos, then there are no delimiters left
            // write the last string if it is of non-zero length and exit loop
            if( ( this->length() - 1 ) - i ){
                return( PygmyString( this->data(), i, this->length() - 1 ) );
            } // if
        } // else
    } // for  
}
    /*static char *index = NULL;
    static char *bufferCopy = NULL;
    char *start;

    if( buffer ){
        if( bufferCopy ){
            free( bufferCopy );
        } // if
        bufferCopy = (char*)malloc( 1 + strlen( buffer ) );
        if( !bufferCopy ){
            return( NULL );
        } // if
        for( ; *buffer; ){
            *(bufferCopy++) = *(buffer++); 
        }
        *bufferCopy = '\0';
        //copyString( buffer, bufferCopy );
        index = bufferCopy;
        
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
            ( (mode & COMMA) && *index == ',' ) ){
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
int PygmyString::getAllSubStrings( char *strings[], int length, short mode )
{
    //PygmyString s;
    u16 i;
    u8 *ucSub;

    // First clear string buffer to prevent memory errors later
    for( i = 0; i < length; i++ ){
        strings[ i ] = NULL;
    } // for
    ucSub = this->getNextSubString( this->c_str(), mode );
    for( i = 0; i < length && ucSub; i++ ){
        strings[ i ] = ucSub;
        ucSub = this->getNextSubString( NULL, mode );
    } // for

    return( i );
}*/
/*
char *PygmyString::getNextSubString( char *buffer, short mode )
{
    static char *index = NULL;
    static char *bufferCopy = NULL;
    char *start;

    if( buffer ){
        if( bufferCopy ){
            free( bufferCopy );
        } // if
        bufferCopy = (char*)malloc( 1 + strlen( buffer ) );
        if( !bufferCopy ){
            return( NULL );
        } // if
        for( ; *buffer; ){
            *(bufferCopy++) = *(buffer++); 
        }
        *bufferCopy = '\0';
        //copyString( buffer, bufferCopy );
        index = bufferCopy;
        
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
            ( (mode & COMMA) && *index == ',' ) ){
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


