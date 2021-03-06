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

class Uart{
    public:
        Uart();
    private:
        // physical resources
        u8 pinTX;
        u8 pinRX;
        u8 pinCTS;
        u8 pinRTS;
        u8 pinDTR;
        // UART settings
        u32 Baud;
        u8 FlowControl;
        u8 SoftMode; // software uart mode
        // FIFO resources
        u16 RXBufferLen;
        u16 RXIndex;
        u16 RXLen;
        u16 TXBufferLen;
        u16 TXIndex;
        u16 TXLen;
        u8 *RXBuffer;
        u8 *TXBuffer;
        u8 *ActionChars;
        // Event driven external functions
        PYGMYCMDPTR Put;
        PYGMYCMDPTR Putc;
        PYGMYVOIDPTR Get;
        // Pipe and status properties
        u8 Pipe;
        u8 CR; 
};

Uart::Uart( u8 pinTX, u8 pinRX, Baud )
{

}

Uart::Uart( u8 pinTX, u8 pinRX, pinCTS, pinTS, Baud )
{

}





