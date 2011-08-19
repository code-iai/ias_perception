/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/************************************************************************
                        Statistics.cpp - Copyright klank


**************************************************************************/

#include "Statistics.h"
#include "XMLTag.h"
using namespace cop;


// Constructors/Destructors
//

Statistics::Statistics ( XMLTag* /*tag*/) {
}

Statistics::~Statistics ( ) { }

//
// Methods
//
XMLTag* Statistics::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_STATISTICS);
	return tag;
}

// Accessor methods
//


// Other methods
//


