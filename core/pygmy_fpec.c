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

#include "pygmy_profile.h"

u16 *globalBaseAddress;

u32 fpecGetIHEXLength( PYGMYFILE *File )
{
    u32 i, ii, Length;
    u16 ShortInt;
    u8 *SubString, Status, ShortInts[64], Buffer[128], IHEXLen;

    Length = 0;
    for( Status = 0; !( File->Properties.Attributes & EOF ) && Status != 0xFF; ){
        for( i = 0; i < 64; i++ ){
            Buffer[ i ] = fileGetChar( File );
            if( Buffer[ i ] == '\r' ){
                Buffer[ i ] = '\0';
                SubString = getNextSubString( (u8*)Buffer, WHITESPACE|NEWLINE );
                // Add 1 to pointer before passing to skip the ':' packet start char

                Status = fpecProcessIHEX( (u8*)( SubString + 1 ) );
                convertHexEncodedStringToBuffer( SubString, ShortInts );
    
                IHEXLen = ShortInts[ 0 ];
                if( ShortInts[ 3 ] == IHEX_DATA ){ // Location three in ShortInts is Record Type
                    Length += IHEXLen * 2;
                } // switch
                if( ShortInts[ 3 ] == IHEX_EOF ){
                    fileSeek( File, START, 0 );
                    return( Length );
                } // if
            } // if
        } // for
    } // for
    
    fileSeek( File, START, 0 );
  
    return( Length );
}

u32 fpecGetID( void )
{   
    // This function is for use with initial run on JTAG
    // The degug registers are not available outside JTAG on some silicon revisions ( ST bugs )
    // If JTAG registers are unavailable and descriptor has not been written, the return will be invalid
    u32 *ulID;

    if ( SIZEREG->Pages >= 512 ){
        globalBaseAddress = (u16*)0x08000000 + ( ( 510 ) * 2048 );
    } else{
        globalBaseAddress = (u16*) 0x08000000 + ( ( SIZEREG->Pages - 2 ) * 1024 );
    } // else
    // First Check the decriptor page in internal flash
    
    ulID = (u32*)globalBaseAddress;
    if( *ulID == 0xFFFFFFFF ){
        // Well, the descriptor is empty, time to load from debug
        ulID = (u32*)0xE0042000; // Address of 32bit ID and revision code in DBG regs
    } // if  
      
    return( *ulID & 0x00000FFF );
}

u8 fpecProcessIHEX( u8 *Buffer )
{
    static u32 Address, AddrOffset;
    static u8 Dummy = 0;
    u16 i, Data;
    u8 ShortInts[64], Len;

    // Dummy write must be to erased area, 0x08002000 is selected as the first sector past the
    // valid bootloader range which is limited to 8KB max
    
    convertHexEncodedStringToBuffer( Buffer, ShortInts );
    
    Len = ShortInts[ 0 ];
    switch( ShortInts[ 3 ] ){ // Location three in ShortInts is Record Type
        case IHEX_DATA:
            AddrOffset = ( (u16)ShortInts[ 1 ] << 8 ) + ShortInts[ 2 ];
            if( !Dummy ){ // one dummy write to valid erased flash address required after erase
                fpecWriteWord( (u16*)( Address + AddrOffset ), 0xFFFF );
                Dummy = 1;
            } // if
            for( i = 0; i < Len; ){
                Data =  (u16)( ShortInts[ 4 + i ] );
                Data |= (u16)( ShortInts[ 4 + i + 1 ] )<<8;
                
                if( !fpecWriteWord( (u16*)(Address + AddrOffset + i), Data) ){
                    return( 0 ); // Flash failed
                } // if
                i+= 2;
                
            }
            break;
        case IHEX_EXTENDEDLINEARADDR: // 16 MSb of 32 bit address stored in data section
            Address = (u32)(ShortInts[ 4 ]) << 24;
            Address |= (u32)(ShortInts[ 5 ]) << 16;
            break;
        case IHEX_STARTLINEARADDR: // Full 32 byte address stored in data section
            Address = (u32)ShortInts[ 4 ] << 24;
            Address |= (u32)ShortInts[ 5 ] << 16;
            Address |= (u32)ShortInts[ 6 ] << 8;
            Address |= (u32)ShortInts[ 7 ];
            break;
        case IHEX_EOF:
            return( 0xFF );
            break;
    } // switch

    return( 1 );
}

u8 fpecUnlock( u8 Bank )
{
    if( Bank ){
        FPEC->KEYR2 = FPEC_KEY1;
        FPEC->KEYR2 = FPEC_KEY2;
    } else{
        FPEC->KEYR = FPEC_KEY1;
        FPEC->KEYR = FPEC_KEY2;
    } // else
}

u8 fpecLock( void )
{
    FPEC->CR |= FPEC_CR_LOCK;
}

u8 fpecWriteLong( u16 *Address, u32 Data )
{
    u8 Status;

    Status = fpecWriteWord( Address, (u16)Data );
    Status &= fpecWriteWord( Address, (u16)( (u32)Data >> 16 ) );

    return( Status );
}

u8 fpecWriteByte( u32 Address, u8 Data )
{
    //u16 WordData, *WordAddress;
    u8 *ByteAddress;

    ByteAddress = (u8*)Address;
    print( STDIO, "\rAddress: 0x%08X 0x%04X", ByteAddress, Data );

    if( (u32)Address > FPEC_MAXBANK1 ){
        if( FPEC->CR2 & FPEC_CR_LOCK ){
            fpecUnlock( 1 );
        } // if
        FPEC->CR2 |= FPEC_CR_PG;
        *ByteAddress = Data;
        while( FPEC->SR2 & FPEC_SR_BSY )
            ;
    } else{
        if( FPEC->CR & FPEC_CR_LOCK ){
            fpecUnlock( 0 );
        } // if
        FPEC->CR |= FPEC_CR_PG;
        *ByteAddress = Data;
        while( FPEC->SR & FPEC_SR_BSY )
            ;
    } // else
    
    //if( *WordAddress == Data ){
        return( TRUE );
    //} // if

    //return( FALSE );
}


/*u8 fpecWriteByte( u32 Address, u8 Data )
{
    u16 WordData, *WordAddress;
    
    if( !(Address % 2) ){
        // Even address
        
        WordData = ( Data << 8 ) | 0x00FF;
        
    } else{
        ++Address;
        WordData = Data | 0xFF00; 
        
    } // else
    

    if( Address % 2 ){
        --Address;
        WordData = ( Data << 8 ) | 0x00FF;
        
    } else{
        WordData = Data | 0xFF00;
    } // else
    
    WordAddress = (u16*)Address;
    print( COM3, "\rAddress: 0x%08X 0x%04X", WordAddress, WordData );

    if( (u32)Address > FPEC_MAXBANK1 ){
        if( FPEC->CR2 & FPEC_CR_LOCK ){
            fpecUnlock( 1 );
        } // if
        FPEC->CR2 |= FPEC_CR_PG;
        *WordAddress = WordData;
        while( FPEC->SR2 & FPEC_SR_BSY )
            ;
    } else{
        if( FPEC->CR & FPEC_CR_LOCK ){
            fpecUnlock( 0 );
        } // if
        FPEC->CR |= FPEC_CR_PG;
        *WordAddress = WordData;
        while( FPEC->SR & FPEC_SR_BSY )
            ;
    } // else
    
    //if( *WordAddress == Data ){
        return( TRUE );
    //} // if

    //return( FALSE );
}*/

u8 fpecWriteWord( u16 *Address, u16 Data )
{
    // A dummy write, calling this function and pointing to a valid
    // location in FLASH, is required before writing data after an
    // erase operation!!!
    
    if( (u32)Address > FPEC_MAXBANK1 ){
        if( FPEC->CR2 & FPEC_CR_LOCK ){
            fpecUnlock( 1 );
        } // if
        FPEC->CR2 |= FPEC_CR_PG;
        *Address = Data;
        while( FPEC->SR2 & FPEC_SR_BSY )
            ;
    } else{
        if( FPEC->CR & FPEC_CR_LOCK ){
            fpecUnlock( 0 );
        } // if
        FPEC->CR |= FPEC_CR_PG;
        *Address = Data;
        while( FPEC->SR & FPEC_SR_BSY )
            ;
    } // else
    
    if( *Address == Data ){
        return( 1 );
    } // if
    
    return( 0 );
}

/*
u8 fpecWriteDescriptor( u16 uiDescriptor, u32 ulValue )
{
    // Descriptor entries are 32 bits each
    u16 *uiAddress;

    uiAddress = (u16*)globalBaseAddress; 
    uiAddress += uiDescriptor * 2;
    
    if( *((u32*)uiAddress) == 0xFFFFFFFF ){ 
        return( fpecWriteLong( uiAddress, ulValue ) );
    } // if
    
    return( 0 );
}*/

/* Deprecated
u8 fpecEraseDescriptor( void )
{

}

u8 fpecEraseMemory( u16 uiStart, u16 uiEnd )
{

}

u8 fpecEraseBoot( void )
{
    
}*/

u32 fpecFlashSectorSize( void )
{
    if( SIZEREG->Pages > 512 ){
        return( 2048 );
    } // if

    return( 1024 );
}

u32 fpecFlashSize( void )
{
    return( *((unsigned short *)0x1FFFF7E0) );
}

u32 fpecRAMSize( void )
{
    return( *((unsigned short *)0x1FFFF7E2) );
}

u32 fpecMCUID( void )
{
    u32 *ptrID, ID, FlashSize;

    ptrID = (u32*)0xE0042000; // Address of 32bit ID and revision code in DBG regs
    ID = *ptrID & 0x00000FFF;

    if( ID < 0x0400 || ID > 0x0500 ){
        // F103 devices have a silicon bug that causes an incorrect ID to be returned
        // If the ID is out-of-range, check the Flash capacity and return the correct F103 MCUID
        // XL capacity MCUs do not have the bug and will always report correctly
        FlashSize = *((unsigned short *)0x1FFFF7E0);
        if( FlashSize == 64 || FlashSize == 128 ){
            ID = 0x0410;
        } else if( FlashSize >= 256 && FlashSize <= 512 ){
            // F103
            ID = 0x0414;
        } else{
            // F103 Low Density ( 16KB or 32KB )
            ID = 0x0412; 
        } // else
    } // if

    return( ID );
}

u32 fpecSectorSize( void )
{
    // 0x0410 F103 medium density, 1024B pages
    // 0x0412 F103 low density, 1024B pages
    // 0x0414 F103 high density, 2048B pages 
    // 0x0416 L152 and L162 medium density, ?
    // 0x0418 F2XX Connectivity, 2048B pages
    // 0x0420 F100 low density, 1024B pages
    // 0x0428 F100 medium density, 1024B pages
    // 0x0430 F103 XL density, 2048B pages
    // 0x0436 L152 and L162 medium+ and high density, ?
    u32 ID;

    ID = 0x00000FFF & *((u32*)0xE0042000); // Address of 32bit ID and revision code in DBG regs
    
    if( ID == 0x0414 || ID == 0x0418 || ID == 0x0430 ){
        return( 2048 );
    } // if 
    
    return( 1024 ); 
}

u8 fpecEraseRange( u32 AddressStart, u32 AddressEnd )
{
    u32 i, Len, PageSize;

    if( AddressStart >= AddressEnd ){
        return( FALSE );
    } // if
    // SIZEREG changes from pages to KB at 512
    if ( SIZEREG->Pages >= 512 ){
        Len = ( SIZEREG->Pages / 2 ) - 2;
        PageSize = 2048;
        i = 4;
    } else{
        Len = SIZEREG->Pages - 2;
        PageSize = 1024;
        i = 8;
    } // else
    Len = ( AddressEnd - AddressStart ) / PageSize;

    for( ; i < Len; i++ ){ 
        PYGMY_WATCHDOG_REFRESH;
        fpecErasePage( AddressStart + ( i * PageSize ) ); 
    } // for

    return( TRUE );
}

u8 fpecEraseProgramMemory( void ) 
{
    u32 i;
    u16 uiLen, uiPageSize;
    //u8 ucFail;
    
    // SIZEREG changes from pages to KB at 512
    if ( SIZEREG->Pages >= 512 ){
        uiLen = ( SIZEREG->Pages / 2 ) - 2;
        uiPageSize = 2048;
        i = 4;
    } else{
        uiLen = SIZEREG->Pages - 2;
        uiPageSize = 1024;
        i = 8;
    } // else
    
    for( ; i < uiLen; i++ ){ 
        PYGMY_WATCHDOG_REFRESH;
        fpecErasePage( 0x08000000 + ( i * uiPageSize ) ); 
    } // for

    return( 1 );
}

u8 fpecErasePage( u32 Address )
{
    u32 *Data;
    u16 i, Len;

    Data = (u32*)Address;
    if ( SIZEREG->Pages >= 512 ){
        Len = 512;
    } else{
        Len = 256;
    } // else
    
    for( i = 0; i < Len; i++ ){         // Verify the Page is empty
        if( *(Data+i) != 0xFFFFFFFF ){
            break;
        } // if
    } // for
    if( i == Len ){
        // Don't erase a page that is already erased
        return( 1 );
    } // if
    if( (u32)Address >= 0x08080000 ){ 
        fpecUnlock( 1 );
        FPEC->CR2 |= FPEC_CR_PER;            // Page Erase
        FPEC->AR2 = Address;               // Load Address
        FPEC->CR2 |= FPEC_CR_STRT;           // Start Erase
        i = 0; // At least one command must be executed before checking status to avoid F103XL silicon bug
        while( FPEC->SR2 & FPEC_SR_BSY )    // Wait until complete
            ;
        FPEC->CR2 &= ~(FPEC_CR_PER|FPEC_CR_STRT);           // Start Erase
    } else{
        fpecUnlock( 0 );
        FPEC->CR |= FPEC_CR_PER;            // Page Erase
        FPEC->AR = Address;               // Load Address
        FPEC->CR |= FPEC_CR_STRT;           // Start Erase
        i = 0; // At least one command must be executed before checking status to avoid F103XL silicon bug
        while( FPEC->SR & FPEC_SR_BSY )    // Wait until complete
            ;
        FPEC->CR &= ~(FPEC_CR_PER|FPEC_CR_STRT);           // Start Erase
    } // else
    
    for( ; i < Len; i++ ){         // Verify the 1KB Page is empty
        if( *(Data+i) != 0xFFFFFFFF ){
            return( 0 );
        } // if
    } // for
       
    return( 1 );
}
