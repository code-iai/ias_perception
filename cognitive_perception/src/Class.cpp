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
                        Class.cpp - Copyright klank


**************************************************************************/

#include "Class.h"
#include "XMLTag.h"


using namespace cop;


// Constructors/Destructors
//

Class::Class ( ) :
Elem()
{
}

Class::Class(std::string name, int id) :
  Elem(id),
  m_name(name)
{
  if(name.length() == 0)
  {
    ROS_WARN("Creating Empty Class with empty name\n");
    throw "Error Building Classes";
  }

}


void Class::SetData (XMLTag* tag)
{
  Elem::SetData(tag);
  if(tag == NULL)
  {
    throw "Error loading class";
  }
  m_name = tag->GetProperty(XML_ATTRIBUTE_CLASSNAME);
  if(m_name.length() == 0)
  {
    ROS_WARN("Creating Empty Class from xml\n");
    throw "Error Building Classes";
  }

}


Class::~Class ( ) { }

//
// Methods
//

void Class::SaveTo(XMLTag* tag)
{
	tag->AddProperty(XML_ATTRIBUTE_CLASSNAME, m_name);
}


void Class::SetName(std::string name)
{
	m_name = name;
}

Elem* Class::Duplicate(bool)
{
  Class* copy = new Class(m_name, m_ID);
  return copy;
} 

// Accessor methods
//


// Other methods
//


