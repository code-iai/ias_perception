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


#ifndef ROSJLOCOMM_H_INCLUDED
#define ROSJLOCOMM_H_INCLUDED

#ifndef USE_YARP_COMM

#include "RelPose.h"
#include "Comm.h"

#include <ros/ros.h>
#include <vision_srvs/srvjlo.h>

namespace cop
{
  /*****************************************************************************************
  *  class ROSjloComm                                                                      */
  /*****************************************************************************************
  *  Class organizing the communication to a the jlo ROS service
  ******************************************************************************************/
  class ROSjloComm : public Comm, public jlo::LazyLocatedObjectLoader
  {
  public:
    /*****************************************************************************************
    *  Constructor
    * @param nodeName
    ******************************************************************************************/
     ROSjloComm(std::string nodeName);
     /*****************************************************************************************
    *  Destructor
    ******************************************************************************************/
     ~ROSjloComm();

    /*****************************************************************************************
    *  NotifyPoseUpdate                                                                        */
    /*****************************************************************************************
    *  Call back that is called whenever a new pose is set for a certain model
    *  This callback must be told the signature that is tracked
    ******************************************************************************************/
    virtual void NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation = true);

    /*****************************************************************************************
    *  CreateNewPose                                                                          */
    /*****************************************************************************************
    ******************************************************************************************/
    virtual RelPose* CreateNewPose(RelPose* pose, Matrix* mat, Matrix* cov);
    virtual RelPose* CreateNewPose(LocatedObjectID_t parent, Matrix* mat, Matrix* cov);
    /*****************************************************************************************
    *  UpdatePose                                                                           */
    /*****************************************************************************************
    *  @brief updates a relpose (sending new information to jlo) including a new parent
    *  @param pose    the old pose, that should be replaced
    *  @param parent  the new parent
    *  @param mat     the new matrix containing the transformation from the parent
    *  @param cov     the new matrix containing the covariance relative to mat
    *
    ******************************************************************************************/
    virtual RelPose* UpdatePose(RelPose* pose, LocatedObjectID_t parent, Matrix* mat, Matrix* cov);

    /*****************************************************************************************
    *  GetPose                                                                              */
    /*****************************************************************************************
    *  @param   poseId The lo-id of the position to get
    *  @remarks return NULL if the id does not exist
    ******************************************************************************************/
    virtual RelPose* GetPose(LocatedObjectID_t poseId);
    /*****************************************************************************************
    *  GetPose                                                                              */
    /*****************************************************************************************
    *  @param   poseId The name of a lo of the position to get
    *  @param   wait   specifies wether this function should block if the name does no exist.
    *
    *  Getting names from jlo could depend on other system to come up,
    *  so waiting might be a good idea in some situations.
    *
    *  @remarks if wait = false the function returns NULL if the name does does not exist
    ******************************************************************************************/
    virtual RelPose* GetPose(const std::string poseId, bool wait = true);

    virtual jlo::LocatedObject* GetParent(const jlo::LocatedObject& child);

    virtual RelPose* GetPoseRelative(LocatedObjectID_t poseId, LocatedObjectID_t parentPoseId);

    virtual bool FreePose(LocatedObjectID_t poseId);

  private:
    ROSjloComm& operator=(ROSjloComm&){throw "Error";}
    bool GetJloServiceClient(vision_srvs::srvjlo &msg);


    /**
      The sevice name of lo
    */
    std::string m_service;
    ros::ServiceClient m_client;
  };
}
#endif /* USE_YARP_COMM */
#endif /* ROSJLOCOMM_H_INCLUDED */
