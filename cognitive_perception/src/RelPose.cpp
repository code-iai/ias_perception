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
                        RelPose.cpp - Copyright klank
**************************************************************************/

#include "RelPose.h"
#include "XMLTag.h"
#include "Object.h"

// Constructors/Destructors
//

using namespace cop;


void TransformPointLocally(Matrix m, double x_in, double y_in, double z_in, double &x_out,double &y_out,double &z_out, double scale = 1.0)
{
  ColumnVector d(4);
  d << x_in / scale << y_in  / scale << z_in / scale<< 1;
  ColumnVector f = m * d;
//  cout << m << " \n* \n" << d << "\n=\n" << f;
  x_out = f.element(0);
  y_out = f.element(1);
  z_out = f.element(2);

}

ReturnMatrix GetExtremePoses(const Matrix& cov)
{
  Matrix m(6, 12);
  try
  {

  int i = cov.nrows() * cov.ncols() ;
  if(i != 0 && !cov.is_zero())
  {
    memset((Real*)m.Store(), 0, sizeof(Real) * 72);
    DiagonalMatrix eigenValues;
    Matrix eigenVectors, V;
            //Decompose Covariance
    SVD(cov, eigenValues, eigenVectors, V, false);
    ColumnVector ExMax(6), ExMin(6), Diff(6);
    Real EigVal;
    for(int i = 1; i <= 6; i++)
    {
      Diff = eigenVectors.column(i);
      EigVal = eigenValues.element(i-1);
      for (int j = 0; j<6; j++)
      {
          Diff.element(j)*=EigVal;
      }
      ExMax = Diff; // transposed i-th eigenvector
      ExMin = Diff*(-1);
      m.column(2*i-1)=ExMax;
      m.column(2*i)=ExMin;
    }
    m.release();
    return m;
  }
  }catch(...)
  {
   printf("Big problem in Extreme Poses\n");
  }
  return m;
}


void cop::RelPoseToRect(RelPose* pose, LocatedObjectID_t relation, MinimalCalibration* calib, int &r1, int &c1, int &r2, int &c2)
{
  int  max_r = 0, max_c = 0, min_r = calib->height, min_c = calib->width;
  try
  {
    Matrix extreme = GetExtremePoses(pose->GetCovariance(relation));
    Matrix m  = pose->GetMatrix(relation);
    for(int vp=0; vp<12;vp++)
    {
       double x = extreme.element(0, vp);
       double y = extreme.element(1, vp);
       double z = extreme.element(2, vp);
       double  xt, yt, zt, row, column;

       TransformPointLocally(m, x,y,z, xt, yt, zt);
       calib->Project3DPoint(xt, yt, zt, row, column);
       if(max_r < row && row < calib->height)
         max_r = row;
       if(min_r > row && row > 0)
         min_r  = row;
       if(max_c < column && column < calib->width)
         max_c = column;
       if(min_c > column && column > 0)
         min_c  = column;
    }
  }
  catch(const char* text)
  {
    printf("Error  in RelPoseToRect: %s\n", text);
    throw "RelPoseToRect: Error creating search space rectangle";
  }
  catch(...)
  {
    throw "RelPoseToRect: Error creating search space rectangle";
  }
#ifndef MIN
#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif
  c1 = MAX(0, min_c*0.9);
  c2 = MIN(calib->width, max_c*1.1);
  r1 = MAX(0, min_r*0.9);
  r2 = MIN(calib->height, max_r*1.1);
}



#ifndef NO_LOLIBADDED



RelPose::RelPose(jlo::LazyLocatedObjectLoader* loader) :
jlo::LocatedObject(loader),
m_qualityMeasure(0.0)
{

}
RelPose::RelPose(jlo::LazyLocatedObjectLoader* loader, LocatedObjectID_t id, LocatedObjectID_t parentID, Matrix m, Matrix cov, std::string name) :
jlo::LocatedObject(loader, id, parentID, m, cov),
m_qualityMeasure(0.0)
{
  m_mapstring = name;
}

RelPose::RelPose(jlo::LocatedObject& pose) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
}



/*RelPose::RelPose(RelPose* pose, Matrix m, Matrix cov) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
  Set(m, cov);
}*/

/*RelPose::RelPose(jlo::LocatedObject* pose, Matrix m, Matrix cov) :
jlo::LocatedObject(pose),
m_qualityMeasure(0.0)
{
  Set(m, cov);
}*/

/*void RelPose::TransformPointLocally(const double& x_in, const double& y_in, const double& z_in, double& x_out, double& y_out, double& z_out, const double& scale)
{
//	ColumnVector cm = GetRotation();
//	cout << cm << "\n";

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
*/
XMLTag* RelPose::Save()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);
  if(m_mapstring.length() > 0)
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_mapstring);
  else
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);

  return ret;
}

#ifdef NO_LO_SERVICE_AVAILABLE
XMLTag* RelPose::SaveComplete()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);
  Matrix m = GetMatrix(ID_WORLD);
  Matrix cov = GetCovarianceMatrix(ID_WORLD);

  ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);
  ret->AddProperty(XML_ATTRIBUTE_LOIDFATHER, ID_WORLD);
  ret->AddChild(XMLTag::Tag(GetMatrix(ID_WORLD), XML_NODE_MATRIX));
  ret->AddChild(XMLTag::Tag(GetCovarianceMatrix(ID_WORLD), XML_NODE_COVARIANCE));

  return ret;
}
#else
XMLTag* RelPose::SaveComplete()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);

  if(m_mapstring.length() > 0)
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_mapstring);
  else
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);

  if(m_uniqueID != ID_WORLD)
    ret->AddProperty(XML_ATTRIBUTE_LOIDFATHER, m_parentID);
  ret->AddChild(XMLTag::Tag(GetMatrix(0), XML_NODE_MATRIX));
  ret->AddChild(XMLTag::Tag(GetCovarianceMatrix(0), XML_NODE_COVARIANCE));

  return ret;
}
#endif


#else
RelPose::RelPose ( ) :
  m_qualityMeasure(0.0)
{

initAttributes();
}

RelPose::RelPose (Pose pose, Object* relation) :
  m_pose(pose),
  m_relation(relation),
  m_qualityMeasure(0.0)
{

}

RelPose::RelPose (XMLTag* tag) :
  m_qualityMeasure(0.0)
{
  if(tag != NULL)
  {
    if(tag->GetName().compare(XML_NODE_RELPOSE) == 0)
    {
      m_pose = Pose(tag->GetChild(0));
      if(tag->CountChildren() > 1)
        m_relation = (Object*)Elem::ElemFactory(tag->GetChild(1));
      else
        m_relation = NULL;
    }
    else
      throw "WRONG NODE";
  }
}

RelPose::~RelPose ( ) { }

//
// Methods
//


// Accessor methods
//
#define MAX_DEPTH 5

inline bool CheckCircles(Object* relation, int depth)
{
  if(depth <= 0)
    return false;
  if(relation != NULL && relation->m_relPose != NULL)
  {
    return CheckCircles(relation->m_relPose->m_relation, depth - 1);
  }
  return true;
}
XMLTag* RelPose::Save()
{
  XMLTag* ret = new XMLTag(XML_NODE_RELPOSE);
  ret->AddChild(m_pose.Save());
  if(m_relation != NULL)
  {
    if ( CheckCircles ( m_relation, MAX_DEPTH) )
      ret->AddChild(m_relation->Save());
  }
  return ret;
}

RelPose& RelPose::operator +=( const Pose &p)
{
  this->m_pose += p;
  return *this;
}

RelPose RelPose::operator + (const Pose &p) const
{
  RelPose rel (this->m_pose, this->m_relation);
  rel.m_pose += p;
  return rel;
}


RelPose RelPose::operator - ( const int &levels ) const
{
  RelPose rel(this->m_pose, this->m_relation);
  if(m_relation != NULL)
  {
    if(levels > 0)
    {
      if(m_relation->m_relPose != NULL)
        rel = (*m_relation->m_relPose) + m_pose;
      else
        rel.m_relation = NULL;
      if(levels > 1)
        rel = rel - (levels - 1);
    }
  }
  return rel;
}




// Public attribute accessor methods
//

// Protected static attribute accessor methods
//


// Protected attribute accessor methods
//


// Private static attribute accessor methods
//


// Private attribute accessor methods
//


// Other methods
//

void RelPose::initAttributes ( ) {
}
#endif


Matrix RelPose::GetMatrix(LocatedObjectID_t id)
{
  if(id == 0 || m_parentID == id)
    return LocatedObject::Get();
  else
  {
    RelPose* pose = RelPoseFactory::GetRelPose(m_uniqueID, id);
    if(pose != NULL)
    {
      Matrix m =  pose->GetMatrix(0);
      RelPoseFactory::FreeRelPose(&pose, true);
      return m;
    }
    else
    {
      throw ("Error requesting Matrix");
    }
  }
}

Matrix RelPose::GetCovariance(LocatedObjectID_t id)
{
  if(id == 0 || m_parentID == id)
    return LocatedObject::GetCovariance();
  else
  {
    RelPose* pose = RelPoseFactory::GetRelPose(m_uniqueID, id);
    if(pose != NULL)
    {
      Matrix m =  pose->GetCovariance(0);
      RelPoseFactory::FreeRelPose(&pose, true);
      return m;
    }
    else
    {
      throw("Error requesting cov\n");
    }
  }
}



namespace sgp_copy
{

typedef struct
{
  double x;
  double y;
  double z;
}
Point3D;

typedef struct
{
  double sx; double sxy; double sxz;
  double syx; double sy; double syz;
  double szx; double szy; double sz;
}
CovariancePoint;


/**
   Determinant
        a           b           c
   double sx; double sxy; double sxz;
      d            e             f
    double syx; double sy; double szy;
      g             h            i
      double szx; double syz; double sz;

*/
double det(CovariancePoint cov)
{
  return cov.sx * cov.sy * cov.sz -
         cov.sx * cov.szy * cov.syz +
         cov.sxy * cov.szy * cov.szx -
         cov.sxy * cov.syx * cov.sz +
         cov.sxz * cov.syx * cov.syz -
         cov.sxz * cov.sy * cov.szx;
}
/**
 Approximation of the probability density
 Implementation of this formula:

 /%4 (cov_sy cov_sz - cov_syz cov_szy)   %3 (-cov_szx cov_syz + cov_syx cov_sz)
|------------------------------------ - --------------------------------------
\                 %2                                      %2

       %1 (cov_szx cov_sy - cov_syx cov_szy)\      /  %4 (cov_sxy cov_sz - cov_sxz cov_szy)
     - -------------------------------------| %4 + |- -------------------------------------
                        %2     1.504695             /      \                   %2

       %3 (-cov_szx cov_sxz + cov_sx cov_sz)   %1 (cov_szx cov_sxy - cov_sx cov_szy)\      /
     + ------------------------------------- + -------------------------------------| %3 + |
                        %2                                      %2                  /      \

    %4 (cov_sxy cov_syz - cov_sxz cov_sy)   %3 (-cov_syx cov_sxz + cov_sx cov_syz)
    ------------------------------------- - --------------------------------------
                     %2                                       %2

       %1 (-cov_syx cov_sxy + cov_sx cov_sy)\
     + -------------------------------------| %1
                        %2                  /

1% = dz := point_test_z - mean_z

2% = cov_bla := cov_szx cov_sxy cov_syz - cov_szx cov_sxz cov_sy - cov_syx cov_sxy cov_sz

     + cov_syx cov_sxz cov_szy + cov_sx cov_sy cov_sz - cov_sx cov_syz cov_szy

3% = dy := point_test_y - mean_y

4% = dx := point_test_x - mean_x

*/
double prob(Point3D point_test, Point3D mean, CovariancePoint cov)
{
  double c_temp, expo, det_temp;
  double dx =  mean.x - point_test.x;
  double dy =  mean.y - point_test.y;
  double dz =  mean.z - point_test.z;
  double cov_bla = cov.szx * cov.sxy * cov.syz - cov.szx * cov.sxz *  cov.sy - cov.syx * cov.sxy * cov.sz
         + cov.syx * cov.sxz * cov.szy + cov.sx * cov.sy * cov.sz - cov.sx * cov.syz * cov.szy;
  if(cov_bla == 0.0)
  {
    cov.sx = 0.0000001;
    cov.sy = 0.0000001;
    cov.sz = 0.0000001;
    cov_bla = cov.szx * cov.sxy * cov.syz - cov.szx * cov.sxz *  cov.sy - cov.syx * cov.sxy * cov.sz
         + cov.syx * cov.sxz * cov.szy + cov.sx * cov.sy * cov.sz - cov.sx * cov.syz * cov.szy;
  }
  double t1 = (( dx * (cov.sy   * cov.sz  - cov.syz * cov.szy)  / cov_bla)-
    ( dy * (-cov.szx * cov.syz + cov.syx * cov.sz)   / cov_bla) -
    ( dz * (cov.szx  * cov.sy  - cov.syx * cov.szy)  / cov_bla)) * dx;
  double t2 =  ((-dx * (cov.sxy  * cov.sz  - cov.sxz * cov.szy)) / cov_bla +
    ( dy * (-cov.szx * cov.sxz + cov.sx  * cov.sz)   / cov_bla) +
    ( dz * (cov.szx  * cov.sxy - cov.sx  * cov.szy)  / cov_bla)) * dy;
  double t3 = (( dx * (cov.sxy  * cov.syz - cov.sxz * cov.sy)   / cov_bla) -
    ( dy * (-cov.syx * cov.sxz + cov.sx  * cov.syz)  / cov_bla) +
    ( dz * (-cov.syx * cov.sxy + cov.sx  * cov.sy)   / cov_bla)) * dz;


  /*printf("cov_bla %f, dx %f dy %f dz %f\n",cov_bla,  dx, dy, dz);*/
  det_temp = det(cov);
  if(det_temp == 0)
    det_temp = 0.00000001;
  c_temp = 1.0 /( pow(2.0 * M_PI, 1.5) * sqrt(fabs(det_temp)));
  /*printf("t1 %f ,t2 %f, t3 %f, cov_bla = %f\n", t1, t2, t3, cov_bla);*/

  expo = (-1.0/2.0) * (t1 + t2 + t3);

  /*printf("expo = %f\n", expo);*/

  c_temp *= exp(expo);
  /*printf("ctemp = %f\n", c_temp);*/
  return c_temp;
}

}

double RelPose::ProbabilityOfCorrespondence(RelPose* pose, bool no_rotation)
{
  using namespace sgp_copy;
  double equality = 0.0;
  if(no_rotation)
  {
    Point3D pose_this;
    Point3D pose_in;
    CovariancePoint cov;

    /** Probability of of pose beeing in this*/

    pose_this.x = 0.0;
    pose_this.y = 0.0;
    pose_this.z = 0.0;

    Matrix m = pose->GetMatrix(m_uniqueID);

    pose_in.x = m.element(0,3);
    pose_in.y = m.element(1,3);
    pose_in.z = m.element(2,3);

    Matrix cov1 = GetCovariance(m_uniqueID);

    cov.sx  = cov1.element(0,0);
    cov.sxy = cov1.element(0,1);
    cov.sxz = cov1.element(0,2);

    cov.syx = cov1.element(1,0);
    cov.sy  = cov1.element(1,1);
    cov.syz = cov1.element(1,2);

    cov.szx = cov1.element(2,0);
    cov.szy = cov1.element(2,1);
    cov.sz  = cov1.element(2,2);

    equality = prob(pose_in, pose_this, cov);
  }
  else
    throw "RelPose::ProbabilityOfCorrespondence with Rotation is not yet implemented";

  return equality;
}

double RelPose::DistanceTo(LocatedObjectID_t id)
{
  Matrix m =  GetMatrix(id);
  double dist = sqrt( m.element(0,3) * m.element(0,3) + m.element(1,3) * m.element(1,3) + m.element(2,3) * m.element(2,3))  +
                fabs(1 - m.element(0,0)) + fabs(1 - m.element(1,1)) + fabs(1 - m.element(2,2));
  return dist;
}

