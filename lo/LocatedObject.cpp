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
#include "lo/LocatedObject.h"
#define ID_WORLD 1

#define MAX_TREE_HEIGHT 200


namespace jlo
{


  LocatedObject::LocatedObject(LazyLocatedObjectLoader* loader) :
        m_uniqueID(ID_WORLD),
        m_parentID(0),
        m_lazyObjLoader(loader)
  {
  }
  LocatedObject::LocatedObject(LazyLocatedObjectLoader* loader, unsigned long id, unsigned long parent_id, const Matrix &matrix , const Matrix &covariance) :
    m_uniqueID(id),
    m_parentID(parent_id),
    m_lazyObjLoader(loader)
  {
        Set(matrix, covariance);
  }

  LocatedObject::LocatedObject(LazyLocatedObjectLoader* loader, unsigned long id, unsigned long parent_id,
    double x , double y , double z ,
	  double roll , double pitch , double yaw ,
	  double sigmaX , double sigmaY , double sigmaZ ,
    double sigmaRoll , double sigmaPitch , double sigmaYaw ) :
    m_uniqueID(id),
    m_parentID(parent_id),
    m_lazyObjLoader(loader)
  {
        ColumnVector v(3), rpy_v(3);
        v << x << y << z;
        rpy_v << roll << pitch << yaw;

        DiagonalMatrix temp(6);
        temp << sigmaX  <<  sigmaY << sigmaZ << sigmaRoll << sigmaPitch << sigmaYaw;
        Matrix temp2=temp;
        Set(rpy(rpy_v) * trans(v), temp2);
  }


//
// Methods
//
bool LocatedObject::CheckWorldCoordinates(const unsigned long &levelsTogo) const
{
        if(levelsTogo > 0 )
        {
            LocatedObject* relation = ((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*this));
                if(relation == NULL || relation->m_uniqueID == ID_WORLD)
                        return true;
                else
                        return relation->CheckWorldCoordinates(levelsTogo - 1);
        }
        else
                return false;
}

/**	@remark .. dies on cyclus*/
unsigned long LocatedObject::FindCommonFather(const LocatedObject* bigObject) const
{
   if(bigObject == NULL)
        return ID_WORLD;
   if(bigObject->m_uniqueID == ID_WORLD || m_uniqueID == ID_WORLD)
     return ID_WORLD;
   const LocatedObject* relation = ((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*this));

        //set the identity matrix also this or this->relation
   const LocatedObject* loc = relation;
   while(loc != NULL)
   {
        if(loc->m_uniqueID == bigObject->m_uniqueID)
            return bigObject->m_uniqueID;
        else
        {
            loc = ((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*loc));
        }
    }
    return FindCommonFather(((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*bigObject)));
}

/**
 * @return LocatedObject
 * @param  smallObject
 */
/*LocatedObject LocatedObject::operator+ (LocatedObject &bigObject ) {

     unsigned long id = FindCommonFather(&bigObject);
     Matrix m, n;
         Matrix mCov, nCov;
     m = bigObject.GetInvMatrix(id);
         mCov = bigObject.GetCovarianceMatrix(id);
     n = GetMatrix(id);
         nCov = GetCovarianceMatrix(id);
     return LocatedObject(&bigObject, m*n, mCov * nCov);
}*/




/**
 * @return Matrix
 * @param  id_to_stop
 */
ReturnMatrix LocatedObject::GetMatrix (const unsigned long &id_to_stop )  {
  Matrix m;
  if(m_uniqueID == 0)
  {
    printf("Null id !!\n");
     m=IdentityMatrix(4);
  }
  else if(m_uniqueID == ID_WORLD || id_to_stop == m_uniqueID)
        m=IdentityMatrix(4);
  else
  {
    LocatedObject* relation = ((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*this));
    if(id_to_stop == 0 && relation != NULL)
    {
          m = Get();
    }
    else
    {
      if(relation != NULL && id_to_stop != m_uniqueID && relation->m_uniqueID != 0)
      {
          m = relation->GetMatrix(id_to_stop) * Get();
      } else {
          //return identity -> means we will always start declaring a World as a LocateObject
          m=IdentityMatrix(4);
      }
    }
  }
  m.release();
  return m;
}

ReturnMatrix LocatedObject::GetMatrix (LocatedObject &obj_to_stop )
{
     unsigned long id = obj_to_stop.FindCommonFather(this);
     if(id == ID_WORLD)
       id = FindCommonFather(&obj_to_stop);
     Matrix m, n;
     m = obj_to_stop.GetInvMatrix(id);
     n = GetMatrix(id);

     m = m * n;
     m.release();
     return m;
}


ReturnMatrix LocatedObject::GetInvMatrix (const unsigned long &id_to_stop )  {
  Matrix m;
  LocatedObject* relation = ((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*this));
  if(id_to_stop  == 0 && relation != NULL)
  {
          m = GetInv();
  }
  else if(relation != NULL && id_to_stop != m_uniqueID)
  {
        m = GetInv() *relation->GetInvMatrix(id_to_stop);
  } else {
        m=IdentityMatrix(4);
  }
  m.release();
  return m;
}

ReturnMatrix LocatedObject::GetInvMatrix ( LocatedObject &obj_to_stop )
{
     unsigned long id = obj_to_stop.FindCommonFather(this);
     Matrix m, n;
     m = obj_to_stop.GetMatrix(id);
     n = GetInvMatrix(id);

     m = n * m;
     m.release();
     return m;
}
/**
*	GetOrigin
*/
ReturnMatrix LocatedObject::GetOrigin (LocatedObject &obj_to_stop )
{
	return GetOrigin(obj_to_stop.m_uniqueID);
}

ReturnMatrix LocatedObject::GetOrigin(const unsigned long &id_to_stop)
{
	Matrix m = GetMatrix(id_to_stop);
	ColumnVector t(4);
	double scale = m.element(3,3);
	t << m.element(0,3)/ scale << m.element(1,3)/ scale << m.element(2,3) / scale << 1;
	t.release();
	return t;
}
/**
* GetRotation
*/
ReturnMatrix LocatedObject::GetRotation (LocatedObject &obj_to_stop )
{
	return GetRotation(obj_to_stop.m_uniqueID);
}
ReturnMatrix LocatedObject::GetRotation (const unsigned long &obj_to_stop)
{
	Matrix m = GetMatrix(obj_to_stop);
	ColumnVector rpyMat = irpy(m);
	rpyMat.release();
	return rpyMat;
}

inline ReturnMatrix LocatedObject::TransformSixVec(const ColumnVector& point, const Matrix& hom) const
{
        //Create 4x4 Euklidian Transformation
        ColumnVector t(3), rot(3);
        t = point.rows(1,3);
        rot = point.rows(4,6);
        Matrix m(4,4);
        m = rpy(rot);
        m.element(0,3) = t.element(0);
        m.element(1,3) = t.element(1);
	m.element(2,3) = t.element(2);
        //cout << "Matrix before trans:\n" << m;
        //Transform the point in the new space
        m = hom*m;
        //extract the 6 components
        ColumnVector rpyRot(3);
        rpyRot = irpy(m);
        ColumnVector ret(6);
        ret << m.element(0,3) << m.element(1,3) << m.element(2,3) <<  rpyRot.element(0)<< rpyRot.element(1)<< rpyRot.element(2);
        //cout << "Eigenvec trans\n" <<  ret << "VecTrans\n" << meanTrans;
        //Substract the new mean  (fixed, not newly calculated)
        /*ret -= meanTrans;*/
        /*
        cout << meanTrans.rows(4,6) << endl << rot << endl;
        Matrix rot_mean = rpy(meanTrans.rows(4,6));
        Matrix rot_point =  rpy(rot);
        cout << "RotMean and rot point\n"<< rot_mean << endl << rot_point << endl;
        rot_mean = rot_point * rot_mean.t();
        ColumnVector rot_diff =  irpy(rot_mean);
        cout << "Difference Rot:\n" << rot_diff << endl;*/
	/*if(fabs(ret.element(0)) > 1.0)
	   ret.element(0) = 1.0;
        if(fabs(ret.element(1)) > 1.0)
           ret.element(1) = 1.0;
        if(fabs(ret.element(2)) > 1.0)
           ret.element(2) = 1.0;*/

        ret.release();
        return ret;
}

inline double MinAngleDist(const double &alpha, const double &beta)
{
   double temp =  min (fabs((alpha) - (beta)), fabs( (min((alpha) ,(beta)) + 2*M_PI) -max(alpha, beta)));
//   cout << "MinAngleDist of " << alpha << "and" << beta << "=" << temp << endl;
   return temp;
}

inline void ApplyRotationMean(ColumnVector &point, const double &rot_x, const double &rot_y, const double &rot_z)
{
  Real& x_in_vec =  point.element(3);
  Real& y_in_vec =  point.element(4);
  Real& z_in_vec =  point.element(5);
//   cout << "ApplyRotMean" << rot_x << " --  "<< x_in_vec << endl;
  x_in_vec = MinAngleDist(rot_x, x_in_vec);
  y_in_vec = MinAngleDist(rot_y, y_in_vec);
  z_in_vec = MinAngleDist(rot_z, z_in_vec);
}

inline void MatSqrt(Matrix& hom)
{
  Real* pt = hom.Store();
  for(int i = 0; i < hom.size(); i++)
  {
  	pt[i] = sqrt(fabs(pt[i])) / 2;
  }
}
inline ReturnMatrix LocatedObject::CalcNewCov(const DiagonalMatrix& eigenValues, const Matrix& eigenVectors, const Matrix& hom) const
{
        // decopose mean t a six-vector (6 dofs, covmatrix)
        ColumnVector v(6), rpyRot(3);
        //cout << "Test:\n" << meanTrans << " == test2\ n" << mean*hom;
        //rpyRot = irpy(mean);
        /*v << mean.element( 0,3) << mean.element(1,3) << mean.element(2,3) << rpyRot.element(0)<< rpyRot.element(1)<< rpyRot.element(2);*/
        /*Matrix meantmp =  rpy(rpyRot);
        meantmp.element(0,3) = mean.element(0,3);
        meantmp.element(1,3) = mean.element(1,3);
        meantmp.element(2,3) = mean.element(2,3);*/
        //cout << "v\n" << v<< "rpy\n"<< rpyRot <<  "Test:\n" << meanTrans << "mean\n"<< mean <<  "== test2\n" << meantmp <<"*\n" << hom<<  "test3\n" << meantmp *hom;
        // same for transformed mean
        ColumnVector vNull(6);
        vNull << 0 << 0<< 0<< 0<< 0<< 0;
        //rpyRot = irpy(meanTrans);
        /*vTrans << meanTrans.element(0,3) << meanTrans.element(1,3) << meanTrans.element(2,3) << rpyRot.element(0)<< rpyRot.element(1)<< rpyRot.element(2);*/
        //cout << "compare v\n" << v << "with vTrans\n" << vTrans;
        Matrix m(6, 6);
        //m -> 0 for +=
        memset((Real*)m.Store(), 0, sizeof(Real) * 36);
        double norm_rot_x = 0.0, norm_rot_y = 0.0, norm_rot_z = 0.0;
        ColumnVector point1[6], point2[6];

        //go over all eigenvectors resulting from a svd and add them as noise to the mean
        for(int i = 0; i <= 5; i++)
        {
/*               ColumnVector point1(6), point2(6);*/
               ColumnVector ExtremPlus, ExtremeMinus;
               if(eigenValues.element(i) > 1.0)
                  eigenValues.Store()[i] = 1.0;
               /*TOCHECK: sometimes the eigenvectors are not scaled like expected! could be a Linux <-> Window difference*/
               point1[i] = TransformSixVec(eigenValues.element(i)*eigenVectors.column(i+1), hom);
               point2[i] = TransformSixVec(-eigenValues.element(i)*eigenVectors.column(i+1), hom);
               /*norm += sqrt(point1.sum_square());
               norm += sqrt(point2.sum_square());*/
               /*cout << "p1["<< i <<"] before \n:" << point1[i]<< endl;
               cout << "p2["<< i <<"] before \n:" << point2[i]<< endl;*/


          }
          vNull = TransformSixVec(vNull, hom);
          norm_rot_x = vNull.element(3);
          norm_rot_y = vNull.element(4);
          norm_rot_z = vNull.element(5);
          for(int i = 0; i <= 5; i++)
          {
	       ApplyRotationMean(point1[i], norm_rot_x, norm_rot_y, norm_rot_z);
               ApplyRotationMean(point2[i], norm_rot_x, norm_rot_y, norm_rot_z);
	       /*cout << "p1[" << i << "] after \n:" << point1[i]<< endl;*/

               m += point1[i] * point1[i].t();
               m += point2[i] * point2[i].t();

              /*cout << "Matrix step" << i  << endl << m <<endl;*/
              double cross_check =  sqrt(m.sum_square());
              if(cross_check > 6.0)
              {
                  /*printf("Problematic Matrix (step %d):  %f \n",i, cross_check);
                  cout << "p1\n" << point1 << "(\n" << eigenValues.element(i) << endl << "*" << eigenVectors.column(i+1) << endl << hom << endl;*/
              }
              /*else
              {
              	cout << "!!good p1\n" << point1;
              }*/
        }
        MatSqrt(m);
        //normalize for the 12 points that were used
        //m /= 12;
	double cross_check =  sqrt(m.sum_square());
	if(cross_check > 6.0)
	{
	   /*printf("Problematic Matrix:  %f \n", cross_check);
	   cout << "eigenValues:\n" << eigenValues << endl << "EigenVectors:\n" << eigenVectors << endl;// << "Mean\n"<< mean<< "MeanTrans\n" << meanTrans << "Hom\n" << hom;*/
	   /*cout << "Cov\n" << m;*/
	}
        m.release();
        return m;
}

inline bool eqd(const double &d1, const double &d2)
{
   if(fabs(d1-d2) < 0.0000000001)
   {
      return true;
   }
   return false;
}

inline bool IsIdentity(const Matrix m)
{
//  cout << "Is m: ident?\n" << m << endl;

   for(int i = 0; i < m.nrows(); i++)
     for(int  j= 0; j < m.ncols(); j++)
         if(!eqd(m.element(i,j), ((i == j) ? 1.0 : 0.0)))
         {
  //       cout << "No\n";
           return false;
           }
    //      cout << "Yes\n";
   return true;
}
#define EIGENVECTORS_ARE_SCALED
inline ReturnMatrix LocatedObject::UnscentedTrans(const Matrix& cov, const Matrix& mean, const Matrix& hom) const
{
   try
   {
        int i = cov.nrows() * cov.ncols() ;
        if(i != 0 && !cov.is_zero() && !IsIdentity(hom))
        {
                DiagonalMatrix D;
                Matrix U, V;
                //Decompose Covariance
                SVD(cov, D, U, V);
                //Evtl. Correct scaling
#ifdef EIGENVECTORS_ARE_SCALED
#else
                if(D.maximum() >= 1.0)
                  printf("Undefined Behaviour in Unscented Trans!\n");
#endif /*EIGENVECTORS_ARE_SCALED*/
                //Calculate the new Covariance
                Matrix hom_d = rpy(irpy(mean*hom));

                Matrix m = CalcNewCov(D, U, hom_d);
                m.release();
                return m;
        }

   }
   catch(...)
   {
     printf("Error in LocatedObject::UnscentedTrans\n");
   }
   return cov;
}
/**
 * @return Matrix
 * @param  id_to_stop
 */
ReturnMatrix LocatedObject::GetCovarianceMatrix (const unsigned long &id_to_stop )
{
  Matrix m;
  if(m_uniqueID == ID_WORLD || id_to_stop == m_uniqueID)
  {
      m = Matrix(6, 6);
      memset((Real*)m.Store(), 0, sizeof(Real) * 36);
  }
  else if (id_to_stop == 0)
  {
    return GetCovariance();
  }
  else
  {
    LocatedObject* relation = m_lazyObjLoader->GetParent(*this);
    Matrix n, n2;
    Matrix cov1, cov2;
    try
    {
       cov1 = GetCovariance();
       cov2 = relation->GetCovarianceMatrix(id_to_stop);
    }
    catch(...)
    {
       printf("Error in LocatedObject::GetCovarianceMatrix, inner\n");
    }
    if(cov1.is_zero() && cov2.is_zero())
    {
       m = cov1;
    }
    else
    {
      try
      {
        n = relation->GetInvMatrix(id_to_stop);
//      cout << "Relation: " << relation->m_uniqueID << endl << n;
        n2 = GetMatrix();
//      cout << "Mean: " << m_uniqueID << endl << n2;
//      cout << "Unscented Transform from "<<m_uniqueID <<" to "<< id_to_stop <<"\n";
        try
        {
                m = cov1 + UnscentedTrans(cov2, n, n2);
        }
        catch(...)
        {
                printf("Error in sum cov2 + UnscentedTrans (%d,%d %d )\ncov2:\n", m_uniqueID, relation->m_uniqueID, id_to_stop);
                cout << cov2 << endl;
                cout << "cov1:\n" << cov1 << endl;
                cout << "n2, n\n" << n2 << endl << n << endl;
                cout << "results of unscent:"<< UnscentedTrans(cov1, n2, n);

        }
      }
      catch(...)
      {
                printf("Error in LocatedObject::GetCovarianceMatrix, lower\n");
      }
    }
    if(m.size() != 36)
    {
      printf("Error calculating cov for %d\n", m_uniqueID);
    }
  }
  m.release();
  return m;
}

ReturnMatrix LocatedObject::GetCovarianceMatrix (LocatedObject &obj_to_stop )
{

  unsigned long id = obj_to_stop.FindCommonFather(this);
  Matrix cov1, cov2;
  Matrix n, n2;
  try
  {
    cov2 = obj_to_stop.GetCovarianceMatrix(id);
    cov1 = GetCovarianceMatrix(id);
  }
  catch(...)
  {
    printf("Error in LocatedObject::GetCovarianceMatrix\n");
  }
  if(!cov1.is_zero())
  {
     n = obj_to_stop.GetInvMatrix(id);
     n2 = GetMatrix(id);
     /*cout << "Unscented Transform from "<< m_uniqueID <<" to "<< obj_to_stop.m_uniqueID <<"\n";*/
     cov1 = cov1 + UnscentedTrans(cov2, n, n2);
  }
  cov1.release();
  return cov1;
}


double LocatedObject::CompareLo(LocatedObject &obj_to_stop )
{
        double comp = 1.0;

        Matrix cov = GetCovarianceMatrix(obj_to_stop);
        Matrix pos = GetMatrix(obj_to_stop);
        ColumnVector rot = irpy(pos);
        ColumnVector xyz(3);
        xyz << pos.element(0,3)<< pos.element(1,3) << pos.element(2,3);
        double dist[6];

        dist[0] = (fabs(xyz.element(0)) - fabs(cov.element(0,0))) > 0 ? 2 : fabs(xyz.element(0)) + fabs(cov.element(0,0)) / 2;
        dist[1] = (fabs(xyz.element(1)) - fabs(cov.element(1,1))) > 0 ? 2 : fabs(xyz.element(1)) + fabs(cov.element(1,1)) / 2;
        dist[2] = (fabs(xyz.element(2)) - fabs(cov.element(2,2))) > 0 ? 2 : fabs(xyz.element(2)) + fabs(cov.element(2,2)) / 2;
        dist[3] = (fabs(rot.element(0)) - fabs(cov.element(3,3))) > 0 ? 2 : fabs(rot.element(0)) + fabs(cov.element(3,3)) / 2;
        dist[4] = (fabs(rot.element(1)) - fabs(cov.element(4,4))) > 0 ? 2 : fabs(rot.element(1)) + fabs(cov.element(4,4)) / 2;
        dist[5] = (fabs(rot.element(2)) - fabs(cov.element(5,5))) > 0 ? 2 : fabs(rot.element(2)) + fabs(cov.element(5,5)) / 2;

        /*cout << pos << "\n" << cov << "\n" << xyz << "\n"  << rot << "\n"  << GetMatrix() << "\n"  << obj_to_stop.GetMatrix();*/

        for(int i = 0; i < 6; i++)
        {
                comp += dist[i];
        }
        return 1 / comp - 1/12;
}


/**
 * @return Matrix
 */
ReturnMatrix LocatedObject::Get ( ) {
  return m_relativePosition;
}


/**
 * @return Matrix
 */
ReturnMatrix LocatedObject::GetCovariance ( )
{
    return m_covariance;
}

/**
 * @return Matrix
 */
ReturnMatrix LocatedObject::GetInv ( )
{
    return m_invRelativePosition;
}


/**
 * @param  matrix
  */
unsigned long LocatedObject::Set(const Matrix &matrix, const Matrix &covMatrix )
{
        m_relativePosition = matrix;
        if(matrix.nrows() != 0)
        {
            try
            {
              /* Check for non rotational matrixes*/
              ColumnVector v = irpy (matrix);
              Matrix  m = rpy(v);
              m.element(0,3) = matrix.element(0,3);
              m.element(1,3) = matrix.element(1,3);
              m.element(2,3) = matrix.element(2,3);
              m_invRelativePosition = m.i();

              /*m_relativePosition = m;*/
              /*m_invRelativePosition = matrix.i();*/
            }
            catch(...)
            {
                m_invRelativePosition = matrix;
            }
        }
        m_covariance = covMatrix;
        return RELOCATE_OBJECT;
        //Safer way : transpose R, change sign of translation, multiply the two 4x4 matrixes
        //Disabled for now, seems to be slower than just the inverse
        /*
        Matrix rot=m_relativePosition.submatrix(1,3,1,3).t();
        rot.resize_keep(4,4);
        rot(4,4)=1;  //Now a Homogeneous transform with only a rotation (needs a 1 at the right down corner)
        //cout << "Rot: \n" << rot << "\n";
        Matrix pos= m_relativePosition.submatrix(1,3,4,4);
        //cout << "Pos: \n" << pos << "\n";
        m_invRelativePosition=rot*trans(-pos);
        */
}

unsigned long LocatedObject::Move(const Matrix &matrix, const Matrix& covMatrix)
{
  LocatedObject* relation = ((m_uniqueID == ID_WORLD) ? NULL : m_lazyObjLoader->GetParent(*this));
  Set(matrix, covMatrix);
  return NO_MOVE;
}

void LocatedObject::TransformPointLocally(const double& x_in, const double& y_in, const double& z_in, double& x_out, double& y_out, double& z_out, const double& scale)
{
	Matrix m = GetMatrix();
	ColumnVector d(4);
	d << x_in / scale << y_in  / scale << z_in / scale<< 1;
	ColumnVector f = m * d;
//	cout << m << " \n* \n" << d << "\n=\n" << f;
	x_out = f.element(0);
	y_out = f.element(1);
	z_out = f.element(2);
	//cout << x_out << ", " << y_out << ", " << z_out << "\n";
}

}// end namespace lo
