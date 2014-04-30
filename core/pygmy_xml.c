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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "pygmy_xml.h"
#include "pygmy_profile.h"

u8 xmlLoad( PYGMYFILE *File, PYGMYXMLNODE *Tree )
{
    // Root->(FirstNode)
    //     ->Child
    //     ->Child
    //        ->Child
    u8 *Line;

    Line = fileGetString( File ); // Fetch the first line of File
    if( !Line ) return( FALSE ); // File IO Error
    if( !Line || !isStringSameIgnoreCase( Line, "<?xml version=\"1.0\"?>" ) || 
        !isStringSameIgnoreCase( Line, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" ) ) 
        return( FALSE ); // Do not process a file unless it has one of two valid opening statements
    free( Line );
    // Note that this next stage could require a significant amount of stack space, depending on
    //    how deeply nest the hierarchy of the XML file is
    // Call xmlAddNextElement(), which is recursive, until all nodes have been added to the Tree
    // Tree must be initialized before use
    xmlInitNode( Tree );
    return( xmlAddNextNode( File, Tree ) );
}

void xmlPrintNode( PYGMYXMLNODE *Node )
{
    print( STDOUT, "\rName: %s", Node->Name );
    print( STDOUT, "\rValue: %s", Node->Value );
    print( STDOUT, "\rPropertyCount: %d", Node->PropertyCount );
    print( STDOUT, "\rChildCount: %d", Node->ChildCount ;
    print( STDOUT, "\rProperties->Key: ", Node->Properties->Key );
    print( STDOUT, "\rProperties->Value: ", Node->Properties->Value );
}

void xmlInitNode( PYGMYXMLNODE *Node )
{
    Node->Name = NULL;
    Node->Value = NULL;
    Node->PropertyCount = 0;
    Node->Properties = NULL;
    Node->Child = NULL;
}
/*
void xmlInitXML( PYGMYXML *Tree )
{
    Tree->NodeCount = 0;
    Tree->Nodes = NULL;
}*/

void xmlAppendLine( PYGMYXMLNODE *Node, u8 *Line )
{
    appendString( Node->Value, Line );
}

u8 xmlAddBranch( PYGMYXMLNODE *Node, u8 *Name )
{
    // This function branches a node by creating a child node
    // If Node is NULL, this function will allocate a node as root
    // If there is already a child node, another child node will be appended
    PYGMYXMLNODE *TempNode;

    if( Node == NULL ){
        Node = malloc( sizeof( PYGMYXMLNODE ) );
        if( Node == NULL ){
            return( FALSE );
        } // if
        xmlInitNode( Node );
        Node->Name = "root";
    } // if

    if( Node->Child == NULL ){
        Node->Child = malloc( sizeof( PYGMYXMLNODE ) );
        if( Node->Child == NULL ){
            return( FALSE );
        } // if
    } else{
        TempNode = realloc( Node->Child, sizeof( PYGMYXMLNODE ) * ( Node->ChildCount + 1 ) );
        if( TempNode == NULL ){
            return( FALSE );
        } // if
        xmlCopyNode( &Node->Child[ Node->ChildCount++ ], TempNode );
    } // else
    
    Node->Child[ Node->ChildCount++ ].Name = Name;
    xmlPrintNode( Node->Child[ Node->ChildCount - 1 ] );
}

void xmlCopyNode( PYGMYXMLNODE *NodeFrom, PYGMYXMLNODE *NodeTo )
{
    NodeTo->Name          = NodeFrom->Name;
    NodeTo->Value         = NodeFrom->Value;
    NodeTo->PropertyCount = NodeFrom->PropertyCount;
    NodeTo->ChildCount    = NodeFrom->PropertyCount;
    NodeTo->Properties    = NodeFrom->Properties;
    NodeTo->Child         = NodeFrom;
}

/*
u8 xmlAddNode( PYGMYXMLNODE *Nodes, u8 *Name )
{
    PYGMYXMLNODE *TempNode;

    
    } else {
        TempNode = realloc( Tree->Nodes, sizeof( PYGMYXMLNODE ) * ( Tree->NodeCount + 1 ) );
        if( TempNode ){ 
            xmlInitNode( TempNode );
            Tree->Nodes = TempNode;
            Tree->Nodes[ Tree->NodeCount++ ].Name = Name;
            return( TRUE );
        } // if
    } // else

    return( FALSE );
}*/

void xmlCopyProperty( PYGMYDICTIONARY *FromProperty, PYGMYDICTIONARY *ToProperty )
{
    FromProperty->Length = ToProperty->Length;
    FromProperty->KeyAndValue = ToProperty->KeyAndValue;
}

u8 xmlAddProperty( PYGMYXMLNODE *Node, u8 *Name, u8 *Value )
{
    PYGMYDICTIONARYPAIR *TempPair;

    if( Node->Properties == NULL ){
        Node->Properties = malloc( sizeof( PYGMYDICTIONARY ) );
        return( TRUE );
    } else {
        TempPair = realloc( Node->Properties->KeyAndValue, sizeof( PYGMYXMLNODE ) * ( Node->PropertyCount + 1 ) );
        if( TempPair ){ 
            Node->Properties->KeyAndValue = TempPair;
            ( Node->Properties + ( Node->PropertyCount * sizeof( PYGMYDICTIONARYPAIR) ) )->KeyAndValue->Key = Name;
            ( Node->Properties + ( Node->PropertyCount * sizeof( PYGMYDICTIONARYPAIR) ) )->KeyAndValue->Value = Value;
            //Node->Properties[ Node->PropertyCount   ]->KeyAndValue->Key = Name;
            //Node->Properties[ Node->PropertyCount++ ]->KeyAndValue->Value = Value;
            return( TRUE );
        } // if
    } // else

    return( FALSE );
}

u8 xmlSkipComment( PYGMYFILE *File )
{
    u8 *Line;

    while( !fileEOF( File ) ){
        Line = fileGetString( File );
        if( !Line ) return( FALSE ); // Out of Memory error, or reached EOF
        if( seekStringInString( Line, "-->" ) ){ // Found end of comment
            return( TRUE );
        } // if
        free( Line ); // This string is allocated by fileGetString() and must be freed
    } // while

    return( FALSE );
}
