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


#ifndef RELPOSEFACTORY_H
#define RELPOSEFACTORY_H

#include "RelPose.h"
#include "Comm.h"
#define XML_NODE_RELPOSELIST "RelPoseList"


namespace cop
{
  #ifdef NO_LO_SERVICE_AVAILABLE
    static std::vector<RelPose*> s_relPoses;
  #else  /*NO_LO_SERVICE_AVAILABLE*/
  #endif /*NO_LO_SERVICE_AVAILABLE*/




  /**
  * Class interfacing the lo pose service
  */
  class RelPoseFactory
  {
  private:
    RelPoseFactory(void);
  public:
    ~RelPoseFactory(void);

    static RelPose* FRelPoseWorld();
    static RelPose* FRelPose(jlo::LocatedObject& pose);
    static RelPose* FRelPose(XMLTag* tag);
    static RelPose* FRelPose(RelPose* pose, Matrix m, Matrix cov);
    static RelPose* FRelPose(LocatedObjectID_t parent, Matrix m, Matrix cov);
    static RelPose* FRelPose(RelPose* pose, LocatedObjectID_t parent, Matrix m, Matrix cov);
    /**
    *   Create a child of a pose without offset
    */
    static RelPose* FRelPoseIdentityChild(RelPose* parent);

    static RelPose* GetRelPose(LocatedObjectID_t id, LocatedObjectID_t parent_if);
    static RelPose* GetRelPose(std::string name);

    static RelPose* FRelPose(LocatedObjectID_t id);

    static void DisposeList();
    static void FreeRelPose(RelPose** pose, bool temporary = false);
    /**
    *   Clones a pose, throws exception on passing NULL
    */
    static RelPose* CloneRelPose(RelPose* pose);
    static RelPose* CloneRelPose(LocatedObjectID_t uniqueID);

  private:
    static RelPose* GetRelPose(LocatedObjectID_t id);
    static int		SetRelPose(RelPose* pose);

  #ifdef NO_LO_SERVICE_AVAILABLE
  public:
    static XMLTag*	SaveList();
    static void LoadList( XMLTag* tag);
  private:
    static RelPose* GetRelPoseIndex(int index);
    static std::vector<RelPose*> s_relPoses;
  #else  /*NO_LO_SERVICE_AVAILABLE*/
  public:
    static Comm* s_loService;
  #endif /*NO_LO_SERVICE_AVAILABLE*/
  private:
    friend class RelPoseHTuple;
  };
}
#endif /*RELPOSEFACTORY_H*/
