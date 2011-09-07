/**********************************************************************************************/
/**
*              Located Object
*              Copyright  (C) 2008, U. Klank
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 **********************************************************************************************/
#include "lo/ObjectContainer.h"


namespace jlo
{
/**
  * class ObjectContainer
  */


/**
 * @return Matrix
 */
ReturnMatrix ObjectContainer::Get ( )
{
  if(m_semaStatic)
    return m_relativePosition;
  else
    throw "Not implemented";
}

ReturnMatrix ObjectContainer::GetInv ( )
{
  if(m_semaStatic)
    return m_invRelativePosition;
  else
   throw "Not implemented";
}

void ObjectContainer::AddAttachedObject(ServiceLocatedObject* lo)
{
  IncreaseReferenceCounter();
  m_attachedLocatedObjectList.push_back(lo);
  if(lo->GetLOType() != LO_TYPE_PHYSICAL)
  {
    TellParentNeedCopy();
  }
}

void ObjectContainer::RemoveAttachedObject(ServiceLocatedObject* lo)
{
  std::vector<ServiceLocatedObject*>::iterator it = m_attachedLocatedObjectList.begin();
  for(;it!= m_attachedLocatedObjectList.end(); )
  {
     if((*it)->m_uniqueID == lo->m_uniqueID)
     {
       it = m_attachedLocatedObjectList.erase(it);
       DecreaseReferenceCounter();
     }
     else
       it++;
  }
}


bool ObjectContainer::NeedCopy()
{
  /*bool needCopy = false;
  std::vector<ServiceLocatedObject*>::iterator it = m_attachedLocatedObjectList.begin();
  for(;it!= m_attachedLocatedObjectList.end() && !needCopy; it++)
  {
     if((*it)->GetLOType() == LO_TYPE_PHYSICAL)
     {
       needCopy = (*it)->NeedCopy();
     }
     else
     {
       needCopy = true;
     }
  }*/
  return m_needCopy > 0;
}

void ObjectContainer::PropagateMovement(ServiceLocatedObject*(*copy)(ServiceLocatedObject*, ServiceLocatedObject*),
                          unsigned long (*del)(ServiceLocatedObject*), void (*updated)(unsigned long), ServiceLocatedObject* parent)
{
  updated(m_uniqueID);
  ObjectContainer* copyOfThis = NULL;
  bool needCopy = NeedCopy();
  if(!needCopy)
    return;
  m_needCopy = 0;
  std::vector<ServiceLocatedObject*>::iterator it = m_attachedLocatedObjectList.begin();
  for(;it!= m_attachedLocatedObjectList.end(); )
  {
    if((*it)->GetLOType() != LO_TYPE_PHYSICAL)
    {
      if(copyOfThis == NULL)
      {
        copyOfThis = (ObjectContainer*)(*copy)(this, parent);
      }

      copyOfThis->AddAttachedObject(*it);
      (*it)->m_parentID = copyOfThis->m_uniqueID;
      (*it)->m_relation = copyOfThis;
      it = m_attachedLocatedObjectList.erase(it);
    }
    else
    {
      if(copyOfThis == NULL)
      {
        copyOfThis = (ObjectContainer*)(*copy)(this, parent);
      }
      (*it)->PropagateMovement(copy, del, updated, copyOfThis);
      it++;
    }
  }
  del(copyOfThis);
}


unsigned long ObjectContainer::Move (const Matrix &matrix, const Matrix &covMatrix)
{
	return RELOCATE_OBJECT;
}
}
