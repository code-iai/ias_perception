#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
using namespace std;

class ImageConverter {

public:
  string image_file_;
  IplImage *cv_image_;
  double rate_;
  ImageConverter(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    image_pub_ = it_.advertise("image_topic_2",50);
  }
  

  ~ImageConverter()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Spin (!)
  bool spin ()
  {
    //double interval = rate_ * 1e+6;
    ros::Rate loop_rate(rate_);
    while (n_.ok ())
      {     
        ROS_INFO ("Publishing data on topic %s.", n_.resolveName ("image_topic_2").c_str ());
        try
          {
            image_pub_.publish(bridge_.cvToImgMsg(cv_image_));
          }
        catch (sensor_msgs::CvBridgeException error)
          {
            ROS_ERROR("Error converting image");
          }
        
        if (rate_ == 0)  
          break;
        loop_rate.sleep();
        ros::spinOnce ();
      }
    return (true);
  }

protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;

};

int main(int argc, char** argv)
{
  if (argc < 3)
    {
      ROS_ERROR ("Syntax is: %s <image file> [publishing_rate (in Hz)]", argv[0]);
      return (-1);
    }
  ros::init(argc, argv, "openCv_to_ros");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ic.image_file_ = string(argv[1]);
  ic.rate_ = atof (argv[2]);
  ROS_INFO ("Loading file %s...", ic.image_file_.c_str ());
  ic.cv_image_ = cvLoadImage(ic.image_file_.c_str(), 1);  
  // if (ic.cv_image_->width > 60 && ic.cv_image_->height > 60)
  //   cvCircle(ic.cv_image_, cvPoint(50,50), 10, CV_RGB( 0, 255, 0 ), 5);
  ic.spin();
  return 0;
}
