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
#pragma once
#include "pygmy_profile.h"

typedef struct PYGMY_XML_NODE PYGMYXMLNODE;
 
struct PYGMY_XML_NODE { 
                u8 *Name;
                u8 *Value;
                u32 PropertyCount;
                u32 ChildCount;
                PYGMYDICTIONARY *Properties;
                PYGMYXMLNODE *Child;
                };
/*
typedef struct{
                //u32 NodeCount;
                PYGMYXMLNODE *Nodes;
                } PYGMYXML;
*/

void xmlPrintNode( PYGMYXMLNODE *Node );
u8 xmlLoad( PYGMYFILE *File, PYGMYXMLNODE *Tree );
u8 xmlAddNextNode( PYGMYFILE *File, PYGMYXMLNODE *Node );
void xmlCopyNode( PYGMYXMLNODE *NodeFrom, PYGMYXMLNODE *NodeTo );
void xmlCopyProperty( PYGMYDICTIONARY *FromProperty, PYGMYDICTIONARY *ToProperty );
u8 xmlAddProperty( PYGMYXMLNODE *Node, u8 *Name, u8 *Value );
//u8 xmlAddNextElement( PYGMYFILE *File, PYGMYXML *Tree );
void xmlAppendLine( PYGMYXMLNODE *Node, u8 *Line );
void xmlInit( PYGMYXML *Tree );
void xmlInitNode( PYGMYXMLNODE *Node );
void xmlInitXML( PYGMYXML *Tree );
u8 xmlAddNode( PYGMYXML *Tree, u8 *Name );
u8 xmlAddNextNode( PYGMYFILE *File, PYGMYXMLNODE *Node );
//u8 xmlAddProperty( PYGMYXML *Tree, u8 *Name, u8 *Value );
u8 xmlSkipComment( PYGMYFILE *File );



