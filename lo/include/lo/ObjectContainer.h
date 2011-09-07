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
#ifndef OBJECTCONTAINER_H
#define OBJECTCONTAINER_H
#include "lo/ServiceLocatedObject.h"


namespace jlo
{

/********************************************************************/
/**      Class ObjectContainer
*********************************************************************
*
*		@brief The prototype for any located object  that moves itself
*		and has objects that depend on this movement.
*
********************************************************************/
class ObjectContainer : public ServiceLocatedObject
{
public:
  /********************************************************************/
  /**      Constructor ObjectContainer
  *********************************************************************
  *
  *   \param locatedObject The LocatedObject that is passed to this constructor is the
  *   	  preceeding element in the location Tree
  *   \param x  all other parameters can be looked up in \ref LocatedObject
  *
  ********************************************************************/
  ObjectContainer (ServiceLocatedObject* locatedObject,	double x , double y , double z ,
                   double roll , double pitch , double yaw ,
                   double sigmaX=0.0, double sigmaY=0.0 , double sigmaZ=0.0,
                   double sigmaRoll=0.0, double sigmaPitch=0.0 , double sigmaYaw=0.0) :
		ServiceLocatedObject(locatedObject,	x ,  y ,  z ,
                                     roll ,  pitch ,  yaw ,
				     sigmaX,  sigmaY ,  sigmaZ,
				     sigmaRoll,  sigmaPitch ,  sigmaYaw),
		m_semaStatic(true)
	{
            referenceCounter = 1;
	}


	/********************************************************************/
	/**      Constructor ObjectContainer
	*********************************************************************
	*
	*      	\param locatedObject The LocatedObject that is passed to this constructor is the
	*   	preceeding element in the location Tree
	*		\param matrix  all other parameters can be looked up in \ref LocatedObject
	*
	********************************************************************/
	ObjectContainer (ServiceLocatedObject* locatedObject, const Matrix &matrix , const Matrix &covariance) :
		ServiceLocatedObject(locatedObject, matrix, covariance),
		m_semaStatic(true)
	{
	    referenceCounter = 1;
	}

/********************************************************************/
/**     GetLOType
*********************************************************************
*       \brief returns if the object needs its children to be moved
********************************************************************/
  virtual unsigned long GetLOType(){return LO_TYPE_PHYSICAL;}


/********************************************************************/
/**     AddAttachedObject
*********************************************************************
*       \brief Adds an object to the list of dependant objects
*               In case of the object beeing moved, all attached objects
*               will be put to a copy *this
*       \remarks if such an object is of type LO_TYPE_PHYSICAL
*                  it will stay attached even if
********************************************************************/
  virtual void AddAttachedObject(ServiceLocatedObject* lo);

/********************************************************************/
/**     RemoveAttachedObject
*********************************************************************
*       \brief Removes an object from the list of dependant objects
********************************************************************/
  virtual void RemoveAttachedObject(ServiceLocatedObject* lo);


/********************************************************************/
/**     Move this
*********************************************************************
*       \brief Adds an object to the list of dependant objects
*               In case of the object beeing moved, all attached objects
*               will be put to a copy *this
*       \remarks if such an object is of type LO_TYPE_PHYSICAL
*                  it will stay attached even if
********************************************************************/
 virtual  void PropagateMovement(ServiceLocatedObject*(* copy)(ServiceLocatedObject*, ServiceLocatedObject*), unsigned long (*del)(ServiceLocatedObject*), void (*updated)(unsigned long), ServiceLocatedObject* parent_copy = NULL);
protected:
  virtual bool NeedCopy ();
  virtual void TellParentNeedCopy(){ m_needCopy++; m_relation->TellParentNeedCopy();}
  virtual void TellParentNeedNoCopy(){ if(m_needCopy > 0){ m_needCopy--; m_relation->TellParentNeedNoCopy();}}
      
public:


  /**
  *   The list of located objects that depend on this object container.
  *   To be aware which other objects are affected, additionally by a movement
  **/
  std::vector<ServiceLocatedObject*> m_attachedLocatedObjectList;

  /**
  *	  A Variable protecting from unnecessary updates of the matrix
  **/
  bool m_semaStatic;

protected:
	/********************************************************************/
	/**      Get
	*********************************************************************
	* @return Matrix
	* @brief this function has to be overwritten if the object container can move
	********************************************************************/
	virtual ReturnMatrix Get ( );
	/********************************************************************/
	/**      GetInv
	*********************************************************************
	* @return Matrix
	* @brief this function has to be overwritten if the object container can move
	********************************************************************/
	virtual ReturnMatrix GetInv ( ) ;

  /**
   * @param matrix
   * @param covMatrix
   * @return unsigned long: returns NO_MOVE if movement to the wished position is possibe else RELOCATE_OBJECT.
  */
  virtual unsigned long Move (const Matrix &matrix, const Matrix& covMatrix);
};
}
#endif  /**OBJECTCONTAINER_H*/

