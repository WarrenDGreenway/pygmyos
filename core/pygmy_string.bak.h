/**************************************************************************
    PygmyOS ( Pygmy Operating System )
    Copyright (C) 2011  Warren D Greenway

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

//#pragma once
#ifndef __PYGMY_HEADER_STRING
	#define __PYGMY_HEADER_STRING

#include "pygmy_profile.h"      
      
#define PUNCT               BIT0
#define WHITESPACE          BIT1
#define NEWLINE             BIT2
#define SEPARATORS          BIT3
#define QUOTES              BIT4
#define COMMA               BIT5
#define ALPHA               BIT6
#define FILESEPARATORS      BIT7

extern const PYGMYPAIR PYGMYNEBULAPINS[];
//extern const u8 PYGMYBASE64CHARS[];
//extern const u8 PYGMYHEXCHARS[];
u8 convertMonthStringToInt( u8 *Month );
u8 convertStringToState( u8 *State );
u8* convertStateToString( u8 State );
u8 *convertPinToString( u8 ucPin );
u8 convertStringToPin( u8 *ucBuffer );
u8 convertStringToPort( u8 *Buffer );
void convertU16ToBuffer( u16 uiData, u8 *ucBuffer, u8 ucEndian );
void convertU32ToBuffer( u32 ulData, u8 *ucBuffer, u8 ucEndian );
u16 convertBufferToU16( u8 *ucBuffer, u8 ucEndian );
u32 convertBufferToU32( u8 *ucBuffer, u8 ucEndian );
u8 *removeLeadingWhitespace( u8 *ucString );
u8 isPrintable( u8 ucChar );
s8 isSeparator( u8 ucChar );
s8 isMathChar( u8 ucChar );
s8 isQuoteChar( u8 ucChar );
s8 isCharInString( u8 ucChar, u8 *ucChars );
u8 isCharsInString( u8 *Chars, u8 *String );
s8 isAlpha( u8 ucChar );
s8 isNumeric( unsigned char ucChar );
s8 isAlphaOrNumeric( u8 ucChar );
s8 isHex( u8 ucChar );
s8 isBinary( u8 ucChar );
s8 isOctal( u8 ucChar );
s8 isWhitespace( u8 ucChar );
u8 isFileSeparator( u8 ucChar );
s8 isPunctuation( u8 ucChar );
u8 isCharSameIgnoreCase( u8 ucChar1, u8 ucChar2 );
u8 isStringSameIgnoreCase( u8 *ucString1, u8 *ucString2 );
s8 isStringSame( u8 *ucBuffer, u8 *ucString );
u16 len( u8 *ucString );
u8 replaceLastChar( u8 *ucString, u8 ucChar, u8 ucNewChar );
u8 replaceChars( u8 *ucBuffer, u8 *ucChars, u8 ucChar );
u8 getLastChar( u8 *String );
u8 *seekStringInString( u8 *ucString, u8 *ucBuffer );
u8 *seekStringInStringIgnoreCase( u8 *SeekString, u8 *InString ); 
void freeParameterList( PYGMYPARAMLIST *Parameters );
u8 getAllParameters( u8 *Buffer, PYGMYPARAMLIST *Parameters );
u16 getAllSubStrings( u8 *ucBuffer, u8 *ucStrings[], u16 uiLen, u8 ucMode );
u8 *getNextSubString( u8 *ucBuffer, u8 ucMode );
void convertFloatToString( double fData, u8 *ucFormat, u8 *ucBuffer );
void convertIntToString( s64 ulData, u8 *ucFormat, u8 *ucBuffer );
void copyString( u8 *ucFrom, u8 *ucTo );
void copyBuffer( u8 *ucFrom, u8 *ucTo, u16 uiLen );
u8 *splitString( u8 *ucString, u8 ucChar, s16 sCount );
u8 *stripLeadingChars( u8 *ucString, u8 *ucChars );
u8 appendString( u8 *ucFrom, u8 *ucTo );
u8 convertCharToUpper( u8 ucChar );
u8 convertCharToLower( u8 ucChar );
u8 *seekCharInString( u8 ucChar, u8 *ucString );
u8 *seekCharInStringIgnoreCase( u8 ucChar, u8 *ucString );
u8 isCharInStringIgnoreCase( u8 ucChar, u8 *ucString );
u8 convertFormatString( u8 *formatString, PYGMYFORMAT *Format  );
double convertStringToFloat( u8 *ucBuffer );
s32 convertStringToInt( unsigned char *ucBuffer );
u8 convertCharToInt( u8 ucChar );
s32 seekStringInBuffer( u8 *ucString, u8 *ucBuffer, u32 ulLen  );
u16 convertHexEncodedStringToBuffer( u8 *ucString, u8 *ucBuffer );
u16 convertHexCharToInteger( u8 ucChar );
u8 getBase64Code( u8 ucChar );
u16 convertBase64ToBuffer( u8 *ucBufferIn, u8 *ucBufferOut );
void convertBase64ToString( u8 *ucBuffer, u8 *ucString );
void convertStringToBase64( u8 *ucString, u8 *ucBase64 );
u32 convertDateStringToSeconds( u8 *Buffer );
u32 countCharInString( u8 Char, u8 *String );
u8* convertModeToString( u8 Mode );
u8 convertStringToMode( u8 *Mode );
void convertIntToStringWithFormat( s64 Data, PYGMYFORMAT *Format, u8 *Buffer );
#endif // __PYGMY_HEADER_STRING
