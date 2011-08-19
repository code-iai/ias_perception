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
                        RelPose.h - Copyright klank


**************************************************************************/

#ifndef RELPOSE_H
#define RELPOSE_H


#define XML_NODE_RELPOSE "RelPose"
#define XML_ATTRIBUTE_LOID "loid"
#define XML_ATTRIBUTE_LOIDFATHER "loidfather"
#define XML_NODE_MATPOSE "Pose"
#define XML_NODE_COVARIANCE "Cov"

#include <string>

#include <lo/lo.h>
#include <ElemTypes.h>

#ifndef ID_WORLD
#define ID_WORLD 1
#endif


namespace cop
{
  class XMLTag;
  /**
  *   class RelPose
  *   @brief Specialization of jlo::LocatedObject, manages a list of such objects
  */
  class RelPose : public jlo::LocatedObject
  {
  public:
    ~RelPose(){}
    /**
    *     Constructor for a Locate Object Reference (see lo, jlo)
    *       Construction should be done using the static functions of RelPoseFactory
    */
    RelPose(jlo::LazyLocatedObjectLoader* loader, LocatedObjectID_t, LocatedObjectID_t parentID, Matrix m, Matrix cov, std::string name = "");

     /** Temporary variable for holding the algorithmic success used to generate this lo*/
     double m_qualityMeasure;
     std::string m_mapstring;
    Matrix GetMatrix(LocatedObjectID_t id);
    Matrix GetCovariance(LocatedObjectID_t id);

    double ProbabilityOfCorrespondence(RelPose* pose, bool no_rotation = true);
    double DistanceTo(LocatedObjectID_t id);
  private:
    RelPose(jlo::LazyLocatedObjectLoader* );
    RelPose(jlo::LocatedObject& pose);
    RelPose(RelPose* pose, Matrix m, Matrix cov);
    RelPose(jlo::LocatedObject* pose, Matrix m, Matrix cov);

  public:
    XMLTag* SaveComplete() ;
  /*	void TransformPointLocally(const double& x_in, const double& y_in, const double& z_in, double& x_out, double& y_out, double& z_out, const double& scale);*/
    XMLTag* Save() ;
  private:
    friend class RelPoseFactory;
    friend class RelPoseHTuple;
  };
  class MinimalCalibration;
  void RelPoseToRect(RelPose* pose, LocatedObjectID_t relation, MinimalCalibration* calib, int &r1, int &c1, int &r2, int &c2);
}
#endif // RELPOSE_H
