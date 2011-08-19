#/*
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
                        Signature.cpp - Copyright klank


**************************************************************************/

#include "Signature.h"
#include "XMLTag.h"
#include "Sensor.h"

#define XML_NODE_CLASSES "Classes"

#include "BoostUtils.h"

#ifdef BOOST_THREAD
#include <boost/thread/mutex.hpp>
#define BOOST(A) A
#else
#define BOOST(A)
#endif

#ifdef _DEBUG
#define DEBUG(A) A
#else
#define DEBUG(A) ;
#endif

using namespace cop;


// Constructors/Destructors
//

Signature::Signature ( )
{

}

void Signature::SetData ( XMLTag* tag )
{
  if(tag == NULL)
    throw ("Trying to feed a null to singature constructor");
  Object::SetData(tag);
  {
    ElemWriteLock lk(m_mutexElems);
    std::string stName = tag->GetName();
    std::string stDefault = GetNodeName();
    if(stName.compare(stDefault) != 0)
    {
      printf("Wrong node detected opening a XML-File for reading a signature\n");
      throw "WRONG NODE";
    }
  }
  //Call Elem Factory
  //Call Class Factory
  try
  {
    XMLTag* classes = tag->GetChild(XML_NODE_CLASSES);
    if(classes  != NULL)
    {
      for(unsigned int i = 0; i < classes->CountChildren(); i++)
      {
        SetClass((Class*)Elem::ElemFactory(classes->GetChild(i)));
      }
    }
    else
    {
      printf("Signature: XML node missing: Classes\n");
    }
  }
  catch(...)
  {
     printf("Error creating Classes\n");
  }
  XMLTag* describingElems = tag->GetChild(0);
  if(describingElems != NULL)
  {
    for(unsigned int i = 0; i < describingElems->CountChildren(); i++)
    {
      XMLTag* tagdescChild = describingElems->GetChild(i);
      if(tagdescChild != NULL)
      {
        try
        {
          SetElem(Elem::ElemFactory(tagdescChild));
        }
        catch(...)
        {
          printf("Error creating Descriptor:  index %d: name %s\n", i, tagdescChild->GetName().c_str());
          printf("Content: %s \n", tagdescChild->WriteToString());
          tagdescChild->FreeAfterWriteToString();
        }
      }
    }
  }
  else
  {
    printf("Signature: XML node missing (descriptors)\n");
  }
}

Signature::~Signature ( )
{
  try
  {
    ElemWriteLock lk(m_mutexElems);
    for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
      iter != m_elems.end(); iter++)
    {
      delete (*iter);
    }
    m_elems.clear();
  }
  catch(const char * text)
  {
    printf("Error deleting Elem of Signature: %s\n", text);
  }
  catch(...)
  {
    printf("Error deleting Elem of Signature\n");
  }

  BOOST(m_mutexClasses.lock());
  try
  {
    for(std::vector<Class*>::const_iterator iter = m_class.begin();
      iter != m_class.end(); iter++)
    {
      delete (*iter);
    }
    m_class.clear();
  }
  catch(const char * text)
  {
    printf("Error deleting Class of Signature: %s\n", text);
  }
  catch(...)
  {
    printf("Error deleting Class of Signature\n");
  }
  BOOST(m_mutexClasses.unlock());

}

//
// Methods
//

Class* Signature::GetClass(int index)
{
  BOOST(m_mutexClasses.lock());
  Class* ret  = NULL;
  if((signed)m_class.size() > index)
    ret = m_class[index];
  BOOST(m_mutexClasses.unlock());
  return ret;
}

bool Signature::HasClass(Class* classToSet)
{
  ElemWriteLock lk(m_mutexClasses);
  size_t size = CountClasses();
  for (unsigned int i = 0 ; i < size; i++)
  {
    if(m_class[i]->m_ID == classToSet->m_ID)
      return true;
  }
  for(unsigned int i = 0 ; i < size; i++)
  {
    if(m_class[i]->GetName().compare(classToSet->GetName()) == 0)
      return true;
  }
  return false;
}

void Signature::Show(Sensor* cam)
{
  Sleeping(0.001);
  try
  {
    if(cam!= NULL)
    {
      cam->Show();
      cam->GetShowLock();

    }
  }
  catch(const char* text)
  {
    printf("Display of sensor data failed: %s\n", text);
  }
  catch (...)
  {
    printf("Display of sensor data failed\n");
  }

  DEBUG(printf("Entering Showing of signature %ld\n", m_ID));
  if(GetObjectPose() != NULL)
  {
    printf("Showing %ld Elements\n",  CountElems());
    for(unsigned int i = 0; i < CountElems(); i++)
    {
      try
      {
          ((Descriptor*)GetElement(i,ELEM))->Show(GetObjectPose(), cam);
      }
      catch(...)
      {
        printf("Showing of elem %d failed ... \n", i);
      }
    }
  }
  else
  {
     printf("Signature has no pose\n");
  }
  if(cam!= NULL)
  {
    cam->ReleaseShowLock();
  }
}

void Signature::SaveTo(XMLTag* tag)
{
  XMLTag* delimiter = new XMLTag("DescibingElems");
  for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
    iter != m_elems.end(); iter++)
  {
    if((*iter) != NULL)
    {
      printf("Saving Descriptor of Type %s\n", (*iter)->GetNodeName().c_str());
      delimiter->AddChild((*iter)->Save(m_fullPose));
    }
  }
  tag->AddChild(delimiter);
  delimiter = new XMLTag("Classes");
  for(std::vector<Class*>::const_iterator iter = m_class.begin();
    iter != m_class.end(); iter++)
  {
    delimiter->AddChild((*iter)->Save());
  }
  tag->AddChild(delimiter);
  Object::SaveTo(tag);

  //TODO
}


Elem* Signature::Duplicate(bool bStaticCopy)
{
  Signature* new_sig = new Signature();

  for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
    iter != m_elems.end(); iter++)
  {
    new_sig->SetElem((*iter)->Duplicate(bStaticCopy));
  }
  for(std::vector<Class*>::const_iterator iter_c = m_class.begin();
    iter_c != m_class.end(); iter_c++)
  {
    new_sig->SetClass((Class*)(*iter_c)->Duplicate(bStaticCopy));
  }
  /** Assign  Object member*/
  if(GetObjectPose() != NULL)
    new_sig->GetObjectPose() = RelPoseFactory::FRelPose(GetObjectPose()->m_uniqueID);
  new_sig->m_bCommunicationCallBack = m_bCommunicationCallBack;
  new_sig->m_com = m_com;

  /** Assign Elem member*/
  new_sig->SetTimeStamp(date());
  new_sig->SetFullPose(m_fullPose);
  SetLastPerceptionPrimitive(GetLastPerceptionPrimitive());

  if(bStaticCopy)
    new_sig->m_ID = m_ID;

  return new_sig;
}


// Private static attribute accessor methods
//


// Private attribute accessor methods
//


// Other methods
//


/**
 * @return Elem--
 * @param  index
 * @param  type
 */
 Elem* Signature::GetElement (const int &index, const ElemType_t &type ) const
 {
  Elem* elem = NULL;

  if(type == 0)
  {
    if(index >= 0 && (unsigned)index < m_elems.size())
      elem = m_elems[index];
  }
  else
  {
    int count = index;
    for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
      iter != m_elems.end(); iter++)
    {
      if((*iter)->GetType() == type)
      {
        if(count == 0)
        {
          elem = (*iter);
          break;
        }
        else
          count--;
      }
    }
  }
  return elem;
}


/**
 * @return int
 * @param  elemToSet
 */
long Signature::SetElem (Elem* elemToSet )
{
  if(elemToSet != NULL)
  {
    BOOST(m_mutexElems.lock());
    m_elems.push_back(elemToSet);
    BOOST(m_mutexElems.unlock());
    int type = elemToSet->GetType();
    if(type > ELEM && type < SIGNATURE)
    {
      SetClass(((Descriptor*)elemToSet)->GetClass());
    }

    return m_elems.size() - 1;
  }
  return -1;
}
void Signature::RemoveElem(Elem* elemToRemove)
{
      BOOST(m_mutexClasses.lock());

      for(std::vector<Elem*>::iterator iter = m_elems.begin(); m_elems.size(); iter++)
      {
        if(*iter == elemToRemove)
        {
          m_elems.erase(iter);
          break;
        }
      }
      BOOST(m_mutexClasses.unlock());
}
long Signature::SetClass (Class* classToSet )
{
  if(classToSet != NULL)
  {
    if(!HasClass(classToSet))
    {
      BOOST(m_mutexClasses.lock());
      m_class.push_back(classToSet);
      BOOST(m_mutexClasses.unlock());

      return m_class.size() - 1;
    }
  }
  return -1;
}


void Signature::Evaluate(const double quality, const double weight)
{
  BOOST(m_mutexElems.lock());
  try
  {
    for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
      iter != m_elems.end(); iter++)
    {
      (*iter)->Evaluate(quality, weight);
    }
  }
  catch(const char * text)
  {
    printf("Error deleting Elem of Signature: %s\n", text);
  }
  catch(...)
  {
    printf("Error deleting Elem of Signature\n");
  }
  BOOST(m_mutexElems.unlock());
}

void Signature::SetPose(RelPose* pose)
{
  Object::SetPose(pose);
  try
  {
    for(std::vector<Elem*>::const_iterator iter = m_elems.begin();
      iter != m_elems.end(); iter++)
    {
      if(((Descriptor*)*iter) != NULL)
        ((Descriptor*)*iter)->PropagatePose(pose);
    }
  }
  catch(const char * text)
  {
    printf("Error in set pose: %s\n", text);
  }
  catch(...)
  {
    printf("Error deleting Elem of Signature\n");
  }


}
