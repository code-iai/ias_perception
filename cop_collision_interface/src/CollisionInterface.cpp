
extern volatile bool g_stopall;
#define NO_MAIN_PRG

#include "CollisionInterface.h"


#include "Signature.h"
#include "SegmentPrototype.h"

using namespace cop;


void SetShape( geometric_shapes_msgs::Shape& sout, const GeometricShape& shape);

typedef struct
{
  double x;
  double y;
  double z;
}
Point3D;

geometry_msgs::Pose RelPoseToRosPose(RelPose* relpose, LocatedObjectID_t rel = 4)
{
    geometry_msgs::Pose pose;
    Matrix m = relpose->GetMatrix(rel);
    pose.position.x = m.element(0,3);
    pose.position.y = m.element(1,3);
    pose.position.z = m.element(2,3);


    double trace = m.element(0,0) + m.element(1,1) + m.element(2,2);
    if(trace > 0) {
      double s = 0.5 / sqrt(1.0 + trace);
      pose.orientation.w = 0.25 / s;
      pose.orientation.x = (m.element(2,1) - m.element(1,2)) * s;
      pose.orientation.y = (m.element(0,2) - m.element(2,0)) * s;
      pose.orientation.z = (m.element(1,0) - m.element(0,1)) * s;
    } else {
      if (m.element(0,0) > m.element(1,1) && m.element(0,0) > m.element(2,2)) {
        double s = 0.5 / sqrt(1.0 + m.element(0,0) - m.element(1,1) - m.element(2,2));
        pose.orientation.w = (m.element(2,1) - m.element(1,2)) * s;
        pose.orientation.x = 0.25 / s;
        pose.orientation.y = (m.element(0,1) + m.element(1,0)) * s;
        pose.orientation.z = (m.element(0,2) + m.element(2,0)) * s;
      } else if (m.element(1,1) > m.element(2,2)) {
        double s = 0.5 / sqrt(1.0 + m.element(1,1) - m.element(0,0) - m.element(2,2));
        pose.orientation.w = (m.element(0,2) - m.element(2,0)) * s;
        pose.orientation.x = (m.element(0,1) + m.element(1,0)) * s;
        pose.orientation.y = 0.25 / s;
        pose.orientation.z = (m.element(1,2) + m.element(2,1)) * s;
      } else {
        double s = 0.5 / sqrt(1.0 + m.element(2,2) - m.element(0,0) - m.element(1,1));
        pose.orientation.w = (m.element(1,0) - m.element(0,1)) * s;
        pose.orientation.x = (m.element(0,2) + m.element(2,0)) * s;
        pose.orientation.y = (m.element(1,2) + m.element(2,1)) * s;
        pose.orientation.z = 0.25 / s;
      }
    }
    return pose;
}




geometry_msgs::Pose RelPoseToRosPose____obs(RelPose* relpose, LocatedObjectID_t rel = 4)
{
  geometry_msgs::Pose pose;
  Matrix m = relpose->GetMatrix(rel);
  pose.position.x = m.element(0,3);
  pose.position.y = m.element(1,3);
  pose.position.z = m.element(2,3);

  /** Transform to quaternion */
  /** Calculate Trace*/

  double S, T = 1 + m.element(0,0) + m.element(1,1) + m.element(2,2);


  /* If the trace of the matrix is greater than zero, then
  perform an "instant" calculation.
  Important note wrt. rouning errors: */
  if ( T > 0.00000001 )
  {
      S = sqrt(T) * 2;
      pose.orientation.x = ( m.element(1,2) - m.element(2,1) ) / S;
      pose.orientation.y = ( m.element(2,0) - m.element(0,2) ) / S;
      pose.orientation.z = ( m.element(0,1) - m.element(1,0) ) / S;
      pose.orientation.w = 0.25 * S;
  }
  else
  {

    /*If the trace of the matrix is equal to zero then identify
    which major diagonal element has the greatest value.
    Deending on this, calculate the following:*/

    if ( m.element(0,0) > m.element(1,1) && m.element(0,0) > m.element(2,2) )  {	// Column 0:
        S  = sqrt( 1.0 + m.element(0,0) - m.element(1,1) - m.element(2,2) ) * 2;
        pose.orientation.x = 0.25 * S;
        pose.orientation.y = (m.element(0,1) + m.element(1,0) ) / S;
        pose.orientation.z = (m.element(2,0) + m.element(0,2) ) / S;
        pose.orientation.w = (m.element(1,2) - m.element(2,1) ) / S;
    } else if ( m.element(1,1) > m.element(2,2) ) {			// Column 1:
        S  = sqrt( 1.0 + m.element(1,1) - m.element(0,0) - m.element(2,2) ) * 2;
        pose.orientation.x = (m.element(0,1) + m.element(1,0) ) / S;
        pose.orientation.y = 0.25 * S;
        pose.orientation.z = (m.element(1,2) + m.element(2,1) ) / S;
        pose.orientation.w = (m.element(2,0) - m.element(0,2) ) / S;
    } else {						// Column 2:
        S  = sqrt( 1.0 + m.element(2,2) - m.element(0,0) - m.element(1,1) ) * 2;
        pose.orientation.x = (m.element(2,0) + m.element(0,2) ) / S;
        pose.orientation.y = (m.element(1,2) + m.element(2,1) ) / S;
        pose.orientation.z = 0.25 * S;
        pose.orientation.w = (m.element(0,1) - m.element(1,0) ) / S;
    }
  }

  return pose;
}


tabletop_object_detector::Table SegmentPrototypeToTable(SegmentPrototype* cluster, LocatedObjectID_t base_link_id)
{
  /**
   Fillling this stupid message:
     tabletop_object_detector/Table table
    geometry_msgs/PoseStamped pose
      Header header
        uint32 seq
        time stamp
        string frame_id
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
    float32 x_min
    float32 x_max
    float32 y_min
    float32 y_max
  */
  tabletop_object_detector::Table ret;
  ret.pose.header.stamp = ros::Time::now();
  ret.pose.header.frame_id = "/base_link"; /**TODO fix this... id is known, must be retranslated to f** tf*/
  std::pair<LocatedObjectID_t, sensor_msgs::PointCloud> locpcd = cluster->GetTable();
  if(locpcd.first == 0)
  {
    printf("No table\n");
    return ret;
  }
  RelPose* pose = RelPoseFactory::FRelPose(locpcd.first);
  if(pose == NULL)
  {
    printf("No table \n");
    return ret;
  }
  ret.pose.pose = RelPoseToRosPose(pose, base_link_id);
  ret.x_min = 10000000;
  ret.y_min = 10000000;
  ret.x_max = -10000000;
  ret.y_max = -10000000;
  for(size_t i = 0; i < locpcd.second.points.size(); i++)
  {
    if(locpcd.second.points[i].x < ret.x_min)
    {
       ret.x_min = locpcd.second.points[i].x;
    }
    if(locpcd.second.points[i].x > ret.x_max)
    {
       ret.x_max = locpcd.second.points[i].x;
    }
    if(locpcd.second.points[i].y < ret.y_min)
    {
       ret.y_min = locpcd.second.points[i].y;
    }
    if(locpcd.second.points[i].y > ret.y_max)
    {
       ret.y_max = locpcd.second.points[i].y;
    }
  }
  RelPoseFactory::FreeRelPose(&pose);

  return ret;
}

bool CollisionInterface::AddCollisionCB(vision_srvs::cop_add_collision::Request& msg, vision_srvs::cop_add_collision::Response&  answer)
{
  Signature* sig = m_myCop.s_sigDb->GetSignatureByID(msg.object_id);
  if(sig == NULL)
  {
    ROS_ERROR("Wrong object ID %ld\n", msg.object_id);
    return false;
  }
  std::string collisionname =  AddCollisionObject(sig, answer.added_object, true);
  if(collisionname.length() > 0)
  {
    answer.collisionname = collisionname;
    return true;
  }
  return false;
}

bool comparePointX(const PointShape &pt1, const PointShape &pt2)
{
  return pt1.x < pt2.x;
}

bool comparePointY(const PointShape &pt1, const PointShape &pt2)
{
  return pt1.y < pt2.y;
}

bool comparePointZ(const PointShape &pt1, const PointShape &pt2)
{
  return pt1.z < pt2.z;
}

std::string CollisionInterface::AddCollisionObject(Signature* sig, mapping_msgs::CollisionObject &object, bool ignore_pcd)
{
  if(sig == NULL)
  {
    ROS_ERROR("Wrong object ID\n");
    return "";
  }
  ROS_INFO("Got signature with %ld elems",  sig->CountElems());
  GeometricShape shape;
  shape.type = 0;
  bool hasShape = false;

  for(size_t id = 0; id < sig->CountElems(); id++)
  {
    Descriptor* elem = (Descriptor*)sig->GetElement (id, ELEM);
    printf("Descriptor: %s\n", elem->GetNodeName().c_str());
    if(elem->GetShape(shape))
    {
      if(ignore_pcd && shape.type==4)
      {
        shape.type = geometric_shapes_msgs::Shape::BOX;
        shape.dimensions.resize(3);

        shape.dimensions[0] =
          (std::max_element(shape.vertices.begin(), shape.vertices.end(), &comparePointX)->x
            - std::min_element(shape.vertices.begin(), shape.vertices.end(), &comparePointX)->x) * 1.1;
        shape.dimensions[1] =
          (std::max_element(shape.vertices.begin(), shape.vertices.end(), &comparePointY)->y
            - std::min_element(shape.vertices.begin(), shape.vertices.end(), &comparePointY)->y) * 1.1;
        shape.dimensions[2] =
          (std::max_element(shape.vertices.begin(), shape.vertices.end(), &comparePointZ)->z
            - std::min_element(shape.vertices.begin(), shape.vertices.end(), &comparePointZ)->z) * 1.1;

        shape.triangles.clear();
        shape.vertices.clear();
      }
      hasShape = true;
      printf("has shape\n");
    }
    else
    {
      printf("has NO shape (but hasSahpe = %s)\n", hasShape ? "true" : "false");
    }
  }

  geometric_shapes_msgs::Shape answer_shape;
  if(hasShape)
  {
    SetShape(answer_shape, shape);
  }
  else
  {
    ROS_INFO("No Shape found");
    return "";
  }

  char objectid[90];
  sprintf(objectid, "Object%ld", sig->m_ID);
  object.id = objectid;
  object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  object.header.frame_id = "map";
  object.header.stamp = ros::Time::now();
  if(sig->GetObjectPose() == NULL)
  {
    ROS_ERROR("Object not localized, can not add it to collision map");
    return "";
  }
  geometry_msgs::Pose pose = RelPoseToRosPose(sig->GetObjectPose(), 1);
  object.poses.push_back(pose);
  object.shapes.push_back(answer_shape);


  m_object_in_map_pub.publish(object);


  return objectid;

}

void CollisionInterface::ResetCollisionMap(Signature* current_object)
{
/***
Filling this service:

[tabletop_collision_map_processing/TabletopCollisionMapProcessing]:
tabletop_object_detector/TabletopDetectionResult detection_result
  int32 NO_CLOUD_RECEIVED=1
  int32 NO_TABLE=2
  int32 OTHER_ERROR=3
  int32 SUCCESS=4
  tabletop_object_detector/Table table
    geometry_msgs/PoseStamped pose
      Header header
        uint32 seq
        time stamp
        string frame_id
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
    float32 x_min
    float32 x_max
    float32 y_min
    float32 y_max
  sensor_msgs/PointCloud[] clusters
    Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Point32[] points
      float32 x
      float32 y
      float32 z
    sensor_msgs/ChannelFloat32[] channels
      string name
      float32[] values
  household_objects_database_msgs/DatabaseModelPose[] models
    int32 model_id
    geometry_msgs/PoseStamped pose
      Header header
        uint32 seq
        time stamp
        string frame_id
       geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
  int32[] cluster_model_indices
  int32 result

bool reset_collision_models
bool reset_attached_models
bool reset_static_map
bool take_static_collision_map
string desired_frame

*/
/*  SegmentPrototype* cluster = (SegmentPrototype*)(current_object->GetElement(0, DESCRIPTOR_SEGMPROTO));
  if(cluster == NULL)
  {
    // TODO check for shapemodels?
    return;
  }

  tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_srv;

  processing_srv.request.detection_result.result = 4; // = SUCCESS
  processing_srv.request.detection_result.table = SegmentPrototypeToTable(cluster, m_base_link_id);
  if(current_object->GetObjectPose() != NULL)
  {
    processing_srv.request.detection_result.clusters.push_back(cluster->GetPointCloud(current_object->GetObjectPose()->m_uniqueID));
    household_objects_database_msgs::DatabaseModelPose model_pose;
    model_pose.pose.pose = RelPoseToRosPose(current_object->GetObjectPose(), m_base_link_id);
    model_pose.pose.header.stamp = ros::Time::now();
    model_pose.pose.header.frame_id = "/base_link"; //TODO fix this... id is known, must be retranslated to f** tf
    processing_srv.request.detection_result.models.push_back(model_pose);
    processing_srv.request.detection_result.cluster_model_indices.push_back(0);
  }
  else
  {
    ROS_ERROR("Trying to add a non-located object to the collision\n");
    return;
  }


  processing_srv.request.reset_collision_models = true;
  processing_srv.request.reset_attached_models = true;
  processing_srv.request.reset_static_map = true;
  processing_srv.request.take_static_collision_map = true;
  processing_srv.request.desired_frame = "base_link";
  if (!m_collision_processing_srv.call(processing_srv))
  {
    ROS_ERROR("Collision map processing service failed");
    return;
  }
  std::cout << "Detected " << processing_srv.response.graspable_objects.size() << " graspable object(s):\n";
  printObjects(objects_info_.graspable_objects, objects_info_.collision_object_names);*/
}

void SetShape( geometric_shapes_msgs::Shape& sout, const GeometricShape& shape)
{
  sout.type = shape.type;
  size_t i;
  for(i = 0; i <  shape.dimensions.size(); i++ )
  {
    sout.dimensions.push_back(shape.dimensions[i]);
  }
  for(i = 0; i <  shape.triangles.size(); i++ )
  {
    sout.triangles.push_back(shape.triangles[i]);
  }
  for(i = 0; i < shape.vertices.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = shape.vertices[i].x;
    p.y = shape.vertices[i].y;
    p.z = shape.vertices[i].z;
    sout.vertices.push_back(p);
  }

}

bool CollisionInterface::GetGeometricShape(vision_srvs::cop_get_object_shape::Request &msg, vision_srvs::cop_get_object_shape::Response &answer)
{
  ROS_INFO("In CollisionInterface::GetGeometricShape");
  Signature* sig = m_myCop.s_sigDb->GetSignatureByID(msg.object_id);
  if(sig == NULL)
  {
    ROS_ERROR("Wrong object ID\n");
    return false;
  }
  ROS_INFO("Got signature");
  GeometricShape shape;
  shape.type = 0;
  bool hasShape = false;
  for(size_t id = 0; id < sig->CountElems(); id++)
  {
    Descriptor* elem = (Descriptor*)sig->GetElement (id, ELEM);
    if(elem->GetShape(shape))
      hasShape = true;
  }
  if(hasShape)
  {
    SetShape(answer.shape, shape);
  }
  else
  {
    ROS_INFO("No Shape found");
    return false;
  }
  return true;
}
