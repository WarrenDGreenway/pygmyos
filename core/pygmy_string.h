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

#pragma once
using namespace std;

#include <string>
#include <vector>
#include "pygmy_type.h"

class PygmyFormat {
    public:
        enum{ FLOAT=1, BINARY=2, OCTAL=8, DECIMAL=10, HEXADECIMAL=16, EXPONENT, STRING, CHARACTER, TIME };
        enum{ NOCASE, UPPERCASE, LOWERCASE };
        
        PygmyFormat( ){
            dataType = DECIMAL;
            width = 0;
            precision = 0;
            rightJustified = false;
            stringCase = NOCASE;
            leftFillChar = ' ';
            rightFillChar = ' ';
        }

        short dataType; 
        short width;
        short precision;
        short stringCase;
        bool rightJustified;
        char leftFillChar;
        char rightFillChar;
}; 


class PygmyString : public std::string {
    public:
        enum Delimiters { FileSeparators=1, Punctuation, Separators, Quotes, Comma, Whitespace, Newline, Alpha};
        enum PinStates{ None, In, Out, Analog, Pullup, Pulldown, Alt };

    
        PygmyString( ) : std::string( ) {}
        PygmyString( std::string& s ) : std::string( s ) {}
        PygmyString(const char *s) : std::string( s ) {}
        PygmyString( std::string& s, int pos, int len ) : std::string( s, pos, len ){}
        PygmyString( PygmyString& s, int pos, int len ) : std::string( s, pos, len ){}
        PygmyString( const PygmyString& s, int pos, int len ) : std::string( s, pos, len ){}

        //using std::string::operator <<;
        using std::string::operator +=;
        using std::string::operator =;
        using std::string::operator [];
        //using std::string::back ;
        

        int convertBase64ToBuffer( char * );
        int convertHexEncodedStringToBuffer( unsigned short * );
        short convertBase64ToCode( char );
        double convertStringToFloat( void );
        int convertCharToInt( char );
        int convertHexCharToInteger( char );
        PygmyString convertFloatToString( double, const char * );
        PygmyString convertFloatToString( double, PygmyString& );
        PygmyString convertIntToStringWithFormat( long long , PygmyFormat& );
        PygmyString convertIntToString( long long, const char * );
        PygmyString convertIntToString( long long, PygmyString& );
        long convertStringToInt( void );
        void convertStringToUpper( void );
        void convertStringToLower( void );
        short convertStringToMode( void );
        std::string convertModeToString( short );
        char convertCharToUpper( char );
        char convertCharToLower( char );
        int convertStringToPort( void );
        PygmyFormat convertToFormat( void );
        char getLastChar( void );
        int replaceLastOccurrenceOfChar( char, char );

        long countCharInString( char );
        void stripLeadingChars( const std::string& );
        void stripLeadingChars( std::string& );
        void stripLeadingWhitespace( void );
        void copyBuffer( char *, char *, long );
        int seekChar( char );
        int seekCharIgnoreCase( char c );
        int seekStringIgnoreCase( const char * );
        int seekStringIgnoreCase( std::string& );
        int seekString( std::string& );
        int replaceChars( const char *, char );
        int replaceChars( std::string&, char );
        void freeParameterList( PYGMYPARAMLIST *Parameters );
        bool getAllParameters( PYGMYPARAMLIST *Parameters );
        std::vector<PygmyString> getAllSubStrings( PygmyString& );
        std::vector<PygmyString> getAllSubStrings( const char * );
        std::vector<PygmyString> getAllSubStrings( PygmyString&, bool );
        PygmyString getNextSubString( short mode );
        PygmyString getNextSubString( const char * );
        PygmyString getNextSubString( PygmyString&, bool );
        bool startsWith( PygmyString& );

        bool areCharsInString( const std::string& );
        bool areCharsInString( std::string& );
        bool isCharInString( char );
        bool isCharInStringIgnoreCase( char );
        //bool isCharInString( char, std::string& );
        bool isPrintable( char );
        bool isAlpha( char );
        bool isNumeric( char );
        bool isAlphaOrNumeric( char );
        bool isHex( char );
        bool isBinary( char );
        bool isOctal( char );
        bool isNewline( char );
        bool isWhitespace( char );
        bool isQuote( char );
        bool isMath( char c );
        bool isFileSeparator( char );
        bool isSeparator( char );
        bool isPunctuation( char );
        bool isCharSameIgnoreCase( char, char );
        bool areStringsSame( std::string& );
        bool areStringsSameIgnoreCase( std::string& );
       
        const static char *HEXCHARS;
        const static char *BASE64CHARS;
};

//const char *PygmyString::HEXCHARS = "0123456789ABCDEF";

//const static char *PygmyString::BASE64CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
