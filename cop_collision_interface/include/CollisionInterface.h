#ifndef COLLISIONINTERFACE_H
#define COLLISIONINTERFACE_H

#include <tabletop_collision_map_processing/collision_map_interface.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include "cop.h"
#include <ros/ros.h>
#include <vision_srvs/cop_add_collision.h>
#include <vision_srvs/cop_get_object_shape.h>
#include "RelPoseFactory.h"

#include <mapping_msgs/CollisionObject.h>


#define COLLISION_PROCESSING_SERVICE_NAME "/tabletop_collision_map_processing/tabletop_collision_map_processing"

#define PICKUP_ACTION_NAME  "/object_manipulator/object_manipulator_pickup"
#define PLACE_ACTION_NAME  "/object_manipulator/object_manipulator_place"
#define GET_MODEL_DESCRIPTION_NAME  "/objects_database_node/get_model_description"

namespace cop
{

  class CollisionInterface
  {
  public:
    CollisionInterface(cop_world &cop_reference, ros::NodeHandle &nh) : m_myCop(cop_reference)
    {
      if (!nh.ok()) exit(0);
        m_collision_processing_srv = nh.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
          (COLLISION_PROCESSING_SERVICE_NAME, true);

       RelPose* pose = RelPoseFactory::GetRelPose("/base_link");
       m_base_link_id = pose->m_uniqueID;
       RelPoseFactory::FreeRelPose(&pose);
       std::string servicename = "/cop_collision";
       std::string service_shape = "/cop_geometric_shape";
      
       printf("n.advertiseService<cop_add_collsion>(%s, ...)\n", servicename.c_str());
       printf("n.advertiseService<cop_get_object_shape>(%s, ...)\n", service_shape.c_str());

       m_copCollisionService = nh.advertiseService(servicename, &CollisionInterface::AddCollisionCB, this);
       m_copGeometricShapeService = nh.advertiseService(service_shape, &CollisionInterface::GetGeometricShape, this);
       m_object_in_map_pub = nh.advertise<mapping_msgs::CollisionObject>("/collision_object", 10);


    }
    bool AddCollisionCB(vision_srvs::cop_add_collision::Request& msg, vision_srvs::cop_add_collision::Response&  answer);

    std::string AddCollisionObject(Signature* current_object, mapping_msgs::CollisionObject &object, bool ignore_pcd=false);
    
    bool GetGeometricShape(vision_srvs::cop_get_object_shape::Request &msg, vision_srvs::cop_get_object_shape::Response &answer);

    void ResetCollisionMap(Signature* current_object);

  private:
    cop_world &m_myCop;

    ros::ServiceClient m_collision_processing_srv;
    ros::ServiceServer m_copCollisionService;
    ros::ServiceServer m_copGeometricShapeService;
    ros::Publisher m_object_in_map_pub;
    LocatedObjectID_t m_base_link_id;
  };


}

#endif /*COLLISIONINTERFACE_H*/
