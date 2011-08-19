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


#ifndef COMM_H
#define COMM_H


#define STD_LO_RPC_PORT_INTERNAL "/located_object"
#define XML_PROPERTY_LO_RPC_PORT "LO_SERVICE_PORT"
class Matrix;
#include "ElemTypes.h"

namespace cop
{
  class RelPose;
  class Signature;

  class Comm
  {
  public:
    Comm()
    {
      m_commID = s_lastCommID;
    }

    virtual void NotifyPoseUpdate(RelPose* /*pose*/, bool /*sendObjectRelation = true*/)=0;/*{throw "Comm NotifyPoseUpdate: Not implemented";}*/
    virtual void NotifyNewObject(Signature*, RelPose*){throw "Comm CreateNewPose: Not implemented";};/*{throw "Comm NotifyPoseUpdate: Not implemented";}*/
    virtual RelPose* CreateNewPose(RelPose* /*pose*/, Matrix* /*mat*/, Matrix* /*cov*/){throw "Comm CreateNewPose: Not implemented";}
    virtual RelPose* CreateNewPose(LocatedObjectID_t /*parent*/, Matrix* /*mat*/, Matrix* /*cov*/){throw "Comm CreateNewPose: Not implemented";}
    virtual RelPose* UpdatePose(RelPose* /*pose*/, LocatedObjectID_t /*parent*/, Matrix* /*mat*/, Matrix* /*cov*/){throw "Comm CreateNewPose: Not implemented";}
    virtual RelPose* GetPose(LocatedObjectID_t /*poseId*/){throw "Comm GetPose: Not implemented";}
    virtual RelPose* GetPose(const std::string /*poseId*/, bool wait = true){throw "Comm GetPose: Not implemented";}
    virtual RelPose* GetPoseRelative(LocatedObjectID_t /*poseId*/, LocatedObjectID_t /*parentPoseId*/){throw "Comm GetPoseRelative: Not implemented";}
    virtual bool FreePose(LocatedObjectID_t /*poseID*/){return false;}

    unsigned long  GetCommID(){return m_commID;}
 private:
    unsigned long m_commID;
    static unsigned long s_lastCommID;
  };

}
#endif /**COMM_H*/


