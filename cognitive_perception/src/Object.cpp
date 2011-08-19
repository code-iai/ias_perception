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
                        Object.cpp - Copyright klank


**************************************************************************/

#include "Object.h"
#include "XMLTag.h"
// Constructors/Destructors
//
using namespace cop;

Object::Object ( ) :
  m_relPose(NULL),
  m_bCommunicationCallBack(false)
{
}

void Object::SetData(XMLTag* tag )
{
  Elem::SetData(tag);
  m_relPose = NULL; /*RelPoseFactory::FRelPose(tag->GetChild(XML_NODE_RELPOSE));*/
  m_bCommunicationCallBack = false;
}

Object::~Object ( )
{
  if(m_bCommunicationCallBack)
    delete m_com;
}



//
// Methods
//
void Object::SetCommCallBack(Comm* com)
{
  if(m_bCommunicationCallBack)
    delete m_com;
  m_com = NULL;
  if(com != NULL)
  {
    m_bCommunicationCallBack = true;
    m_com = com;
  }
  else
  {
     m_bCommunicationCallBack = false;
  }
}

void Object::SetPose(RelPose* pose)
{
  if(pose != NULL)
  {
#ifdef _DEBUG
    printf("Setting new pose. (%ld)\n", pose->m_uniqueID );
#endif
    if(m_relPose != NULL && !(pose->m_uniqueID == m_relPose->m_uniqueID))
      RelPoseFactory::FreeRelPose(&m_relPose);
    m_relPose = pose;
    Touch();
  }
  if(m_bCommunicationCallBack)
  {
#ifdef _DEBUG
    printf("Sending update Notification.\n");
#endif
    m_com->NotifyPoseUpdate(pose, true);
  }
}


// Accessor methods
//
void Object::SaveTo(XMLTag* tag)
{
  if(m_relPose != NULL)
  {
    if(!m_fullPose)
      tag->AddChild(m_relPose->Save());
    else
      tag->AddChild(m_relPose->SaveComplete());
  }
}


