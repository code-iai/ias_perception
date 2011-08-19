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
                        Elem.cpp - Copyright klank


**************************************************************************/

#include "Elem.h"
#include "XMLTag.h"
#include "Signature.h"
#include "Object.h"
#include "Descriptor.h"
#include "Class.h"

#include <time.h>


#include <pluginlib/class_loader.h>

using namespace cop;


ObjectID_t Elem::m_LastID = 0;


// Constructors/Destructors
//

Elem::Elem ( ) :
    m_ID(m_LastID++),
    m_timestamp((unsigned long)time(NULL)),
    m_creator(0)
{
}

Elem::Elem ( ObjectID_t id ) :
    m_ID(id),
    m_timestamp((unsigned long)time(NULL)),
    m_creator(0)
{
        if(m_LastID < m_ID && m_ID < FORBIDDEN_ID_RANGE_MIN)
            m_LastID = m_ID + 1;
}

void Elem::SetData ( XMLTag* creator)
{
  ElemWriteLock lk(m_mutexElems);
  if(creator != NULL)
  {
    m_ID = creator->GetPropertyInt(XML_PROPERTY_ELEMID);
    m_creator = creator->GetPropertyInt(XML_PROPERTY_PPID, 0);
    if(m_LastID < m_ID && m_ID < FORBIDDEN_ID_RANGE_MIN)
      m_LastID = m_ID + 1;

    m_timestamp = creator->date();
    m_timestamp -= 2;
  }
  m_timestamp = ((unsigned long)time(NULL));
}

bool StringEquals(std::string name1, std::string name2)
{
    return name1.compare(name2) == 0;
}

pluginlib::ClassLoader<Descriptor> s_descr_loader("cognitive_perception", "Descriptor");

Elem* Elem::ElemFactory ( XMLTag* tag)
{
  if(tag == NULL)
      throw "WRONG NODE";
  std::string name = tag->GetName();
  Elem* elem = NULL;
  if(StringEquals(name, XML_NODE_CLASS))
  {
      elem = new Class();
      elem->SetData(tag);
  }
  else if(StringEquals(name, XML_NODE_DESCRIPTOR))
  {
      elem = new Descriptor();
      elem->SetData(tag);
  }
  else if(StringEquals(name, XML_NODE_ELEMENT))
  {
      elem = new Elem();
      elem->SetData(tag);
  }
  else if(StringEquals(name, XML_NODE_OBJECT))
  {
      elem = new Object();
      elem->SetData(tag);
  }
  else if(StringEquals(name, XML_NODE_SIGNATURE))
  {
     elem = new Signature();
     elem->SetData(tag);
  }
  else
  {


    try
    {
      elem = s_descr_loader.createClassInstance(name);
      elem->SetData(tag);
    }
    catch(pluginlib::PluginlibException& ex)
    {
    //handle the class failing to load
      ROS_WARN("The plugin failed to load for some reason. Error: %s\n", ex.what());
      ROS_WARN("Tag failed: %s\n", tag->GetName().c_str());
    }
  }
  return elem;
}

Elem* Elem::Duplicate(bool bStaticCopy)
{
  XMLTag* tag = this->Save();
  if(!bStaticCopy)
  {
    tag->AddProperty(XML_PROPERTY_ELEMID, m_LastID++);
  }
  Elem* copy = Elem::ElemFactory(tag);
  delete tag;
  return copy;
}

void Elem::Touch()
{
    m_timestamp = (unsigned long)time(NULL);
}



Elem::~Elem ( ) { }

//
// Methods
//
XMLTag* Elem::Save(bool full_pose)
{
  ElemWriteLock lk(m_mutexElems);
  m_fullPose = full_pose;
  XMLTag* ret = new XMLTag(GetNodeName());
  ret->AddProperty(XML_PROPERTY_ELEMID, m_ID);
  ret->AddProperty(XML_PROPERTY_PPID, m_creator);
  SaveTo(ret);
  return ret;
}

// Accessor methods
//


// Other methods
//


