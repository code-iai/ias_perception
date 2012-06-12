/*
 * zbar_image_reader.cpp
 *
 *  Created on: Jun 12, 2012
 *      Author: Nacer Khalil
 */
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "std_msgs/String.h"
#include <zbar.h>
#include <Magick++.h>
#include "comp_barcoo/send_barcode.h"


using namespace std;
using namespace zbar;
class zbar_image_reader
{
	protected:
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	ros::Publisher barcode_pub_;
	ros::ServiceClient client;
	comp_barcoo::send_barcode srv;
	public:
	zbar_image_reader(ros::NodeHandle &n,string img_path): n_(n), it_(n_)
	{
		cv::Mat image = cv::imread(img_path,0);
		client = n.serviceClient<comp_barcoo::send_barcode>("/send_barcode");

		extract_publish_barcode(image);
	}

	~zbar_image_reader() {
		// TODO Auto-generated destructor stub
	}

	void extract_publish_barcode(cv::Mat img)
	{
		// create a reader
		ImageScanner scanner;

		// configure the reader
		scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

		int width = img.cols;   // extract dimensions
		int height = img.rows;

		// obtain image data
		Magick::Blob blob(img.ptr(0), width * height);

		const void *raw = blob.data();

		// wrap image data
		Image image(width, height, "Y800", raw, width * height);

		// scan the image for barcodes
		int n = scanner.scan(image);

		// extract results
		std::stringstream ss;
		for( Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol )
		{
		  // do something useful with results
		  ROS_INFO_STREAM("Publishing: barcode type: " << symbol->get_type_name() << " barcode value " << symbol->get_data());

		  std_msgs::String msg;

		  ss << symbol->get_data();
		  srv.request.barcode.data = ss.str();
		  if (client.call(srv))
		  {
			  std::cout << "received: " << srv.response.recieved << "\n";
		  }
		}
		if (n == 0)
		{
			ROS_WARN("Barcode not found");
			return;
		}

		if (n < 0)
		{
			ROS_ERROR("Error occured while finding barcode");
			return;
		}

	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_image_node");
  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();
  ros::NodeHandle n("~");
  zbar_image_reader reader(n,argv[1]);
  ros::spinOnce();

  return 0;
}
