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
 #ifndef LOCATEDOBJECT_H
#define LOCATEDOBJECT_H

#ifndef LO_TYPE_PERCEIVED
#define LO_TYPE_PERCEIVED 0
#endif
#include <vector>

/*************************
* 	include RO-BOOB
*************************/
#include "lo/NewMatExhaustive.h"

/* Definition for return values of Set(...) */
#define RELOCATE_OBJECT 1
#define NO_MOVE		0

namespace jlo
{

/********************************************************************/
/**     class LazyLocatedObjectLoader
*********************************************************************
* A class that returns the parent on passing an lo to it
********************************************************************/
  class LazyLocatedObjectLoader;

/********************************************************************/
/**     class LocatedObject
*********************************************************************
* Basic strcture  for located object for lazy parent expansion
* Should be used outside of the lo service, it uses a
********************************************************************/
class LocatedObject
{
public:
  LocatedObject(LazyLocatedObjectLoader* loader);
  LocatedObject(LazyLocatedObjectLoader* loader, unsigned long id, unsigned long  parent_id, const Matrix &matrix , const Matrix &covariance);
  LocatedObject(LazyLocatedObjectLoader* loader, unsigned long id, unsigned long  parent_id,
    double x = 0.0, double y = 0.0, double z = 0.0,
	  double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
	  double sigmaX = 0.0, double sigmaY = 0.0, double sigmaZ = 0.0,
	  double sigmaRoll = 0.0, double sigmaPitch = 0.0, double sigmaYaw = 0.0);
  /********************************************************************/
  /**	m_uniqueID
  *********************************************************************
  *
  *	This member is a unique object ID,
  *	\remarks it has to be unique in a location tree,
  *                 or problems for relative object localisation can appear
  ********************************************************************/
  unsigned long m_uniqueID;

/********************************************************************/
/**	m_parentID
*********************************************************************
*
*	This member is a unique object ID of the parent
*	\remarks it has to be unique in a location tree,
*                 or problems for relative object localisation can appear
********************************************************************/
  unsigned long m_parentID;

/********************************************************************/
/**	m_lazyObjLoader
*********************************************************************
*
* this component must be able to acquire the parent of this object
********************************************************************/
  LazyLocatedObjectLoader* m_lazyObjLoader;


/********************************************************************/
/**     GetMatrix
*********************************************************************
*       Returns the current Matrix relative to a certain id
*       LocatedObject (obj, id, self without param)
*
*       \pi     obj_to_stop
*       \ret    ReturnMatrix
********************************************************************/
  virtual ReturnMatrix GetMatrix (LocatedObject &obj_to_stop );
  virtual ReturnMatrix GetMatrix (const unsigned long &id_to_stop = 0 );
/********************************************************************/
/**     GetInvMatrix
*********************************************************************
*       Returns the current Matrix -inverted and- relative to a certain
*       LocatedObject (obj, id, self without param)
*
*       \pi     obj_to_stop
*       \ret    ReturnMatrix
********************************************************************/
   virtual ReturnMatrix GetInvMatrix (LocatedObject &obj_to_stop );
   virtual ReturnMatrix GetInvMatrix (const unsigned long &obj_to_stop = 0 );


/********************************************************************/
/**     GetCovarianceMatrix
*********************************************************************
*       Returns the current CovarianceMatrix relative to a certain
*       LocatedObject (obj, id, self without param)
*
*       \pi     obj_to_stop
*       \ret    ReturnMatrix
********************************************************************/
   ReturnMatrix GetCovarianceMatrix (LocatedObject &obj_to_stop ) ;
   ReturnMatrix GetCovarianceMatrix (const unsigned long &obj_to_stop = 0) ;
/********************************************************************/
/**     CompareLo
*********************************************************************
*		\brief compares two LocatedObjects
*       \ret returns the size of overlapping possibility cloud volume
*		relative to the smaller covaiance value
*
*       \pi     obj_to_stop
********************************************************************/
   double CompareLo(LocatedObject &obj_to_stop ) ;

/********************************************************************/
/**     GetOrigin
*********************************************************************
*       Returns the current 3D coordinate of the origin point relative
*		to the given LocatedObject (obj, id, self without param)
*
*       \pi     obj_to_stop
*       \ret    ReturnMatrix 4 * 1 Vector
********************************************************************/
   ReturnMatrix GetOrigin (LocatedObject &obj_to_stop ) ;
   ReturnMatrix GetOrigin (const unsigned long &obj_to_stop = 0) ;
/********************************************************************/
/**     GetRotation
*********************************************************************
*       Returns the current rotation vector
*		to the given LocatedObject (obj, id, self without param)
*
*       \pi     obj_to_stop
*       \ret    ReturnMatrix 4 * 1 Vector
********************************************************************/
   ReturnMatrix GetRotation (LocatedObject &obj_to_stop ) ;
   ReturnMatrix GetRotation (const unsigned long &obj_to_stop = 0) ;
 /********************************************************************/
 /**	TransformPointLocally: Transform a 3d point relative to this lo relative to the father (TODO: rel to any)
 *********************************************************************
 *
 * 	\pi x_in  3d point x-coordinate locally
 * 	\pi y_in  3d point y-coordinate locally
 * 	\pi z_in  3d point z-coordinate locally
 * 	\po x_out  3d point x-coordinate transformed
 * 	\po y_out  3d point y-coordinate transformed
 * 	\po z_out  3d point z-coordinate transformed
 ********************************************************************/
 void TransformPointLocally(const double& x_in, const double& y_in, const double& z_in, double& x_out, double& y_out, double& z_out, const double& scale = 1);

 /********************************************************************/
 /**	Checks for circles, by looking for the world
 *********************************************************************
 *
 * 	\pi levelTogo levels to go
 ********************************************************************/
  bool	CheckWorldCoordinates	(const unsigned long &levelsTogo) const;
/********************************************************************/
/**     GetLOType
*********************************************************************
*       \brief returns if the object needs its children to be moved
********************************************************************/
  virtual unsigned long GetLOType(){return LO_TYPE_PERCEIVED;}

protected:


 /********************************************************************/
 /**      m_relativePosition
 *********************************************************************
 *
 *      The transformation matrix to thepreeceding node
 *	in the LocatedObject Tree
 *	\remarks only access through the virtual accessor Get()
 *		and Set also internally
 *
 ********************************************************************/
 Matrix m_relativePosition;

/********************************************************************/
 /**      m_invRelativePosition
 *********************************************************************
 *
 *      The transormation in the inverse direction
 ********************************************************************/
 Matrix m_invRelativePosition;

/********************************************************************/
/**      m_covariance
*********************************************************************
*
*       propagated uncertainty
*
********************************************************************/
 Matrix m_covariance;

 /********************************************************************/
 /**	Virtual Accessor to the Matrix, Always use this get
 *********************************************************************
 *
 * 	@return Matrix
 ********************************************************************/
  virtual ReturnMatrix Get ( ) ;
  virtual ReturnMatrix GetCovariance();

 /********************************************************************/
 /**	Virtual Accessor to the Invers-Matrix, use this to get mat!
 *********************************************************************
 *
 * 	@return Matrix
 ********************************************************************/
  virtual ReturnMatrix GetInv ( ) ;


 /********************************************************************/
 /**	Virtual Accessor to the Matrix,
 *********************************************************************
 *  Set moves relative to the preceeding node, without changing
 *	Move only moves in case of a moving object that has overwritten the function
 * 	\pi matrix
 * 	\pi covMatrix
 ********************************************************************/
  virtual unsigned long Set (const Matrix &matrix, const Matrix &covMatrix ) ;
 /********************************************************************/
 /**	Virtual Accessor to the Matrix,
 *********************************************************************
 *  Set moves relative to the preceeding node, without changing
 *  Move only moves in case of a moving object that has overwritten the function
 *  \pi matrix  the new matrix
 *  \pi covMatrix
 ********************************************************************/
  virtual unsigned long Move (const Matrix &matrix, const Matrix &covMatrix) ;
protected:
 /********************************************************************/
 /**	FindCommonFather
 *********************************************************************
 *		searches for the first common father node in the LocatedObject
 *		tree
 *
 * 	\pi obj
 * 	\ret id of the common father node
 ********************************************************************/
  unsigned long	FindCommonFather		(const LocatedObject* obj) const;


	/********************************************************************/
	/**	helping functions for UnscentedTrans
	*********************************************************************
	*
	********************************************************************/
  ReturnMatrix CalcNewCov(const DiagonalMatrix& eigenValues, const Matrix& eigenVectors, const Matrix& hom) const;
	ReturnMatrix TransformSixVec(const ColumnVector& point, const Matrix& hom) const;
	/********************************************************************/
	/**	Performing a transformation approximation on the covariance
	*********************************************************************
	*
	* 	\pi cov
	* 	\pi mean
	*	\pi hom
	********************************************************************/
	ReturnMatrix UnscentedTrans(const Matrix& cov, const Matrix& mean, const Matrix& hom) const;
  // Static id counter.
  //
};

class LazyLocatedObjectLoader
{
public:
  LazyLocatedObjectLoader(){}
  virtual LocatedObject* GetParent(const LocatedObject& child)= 0;
};

}
#endif // LOCATEDOBJECT_H
