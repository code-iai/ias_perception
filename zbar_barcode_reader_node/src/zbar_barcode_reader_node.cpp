
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "std_msgs/String.h"
#include <sstream>
#include "curl/curl.h" 
#include "tinyxml.h"
#include <highgui.h>
#include <zbar_barcode_reader_node/enable_barcode_reader.h>
#include <cv_bridge/cv_bridge.h>
//Magick++ lib
#include <Magick++.h>
//zbar
#include <zbar.h>

using namespace std;
using namespace zbar;

class BarcodeReaderNode {
	std:: string link1_;// = "http://www.barcoo.com/api/get_product_complete?pi=";
	std:: string link2_;// = "&pins=ean&format=xml&source=ias-tum";
	std:: string tag1_;// = "answer";
	std:: string tag2_;// = "picture_high";
	std:: string tag3_;// = "picture_low";
	std:: string pattern_;// = "<meta property=\"og:image\" content=\"";
	TiXmlDocument doc;
	int enable_barcode_reader_;
	int image_received_;
	int image_found_;
public:
  struct memoryStruct {
    char *memory;
    size_t size;
  };

  struct UserData {
    memoryStruct *memory;
    BarcodeReaderNode *self;
    UserData(memoryStruct *memory, BarcodeReaderNode *self)
      : memory(memory), self(self) {}
  };

  BarcodeReaderNode(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    n_.param ("input_image_topic", input_image_topic_, std::string("/image_raw"));
    n_.param ("outout_barcode_topic", output_barcode_topic_, std::string("barcode"));
    n_.param ("link1", link1_, std::string("http://www.barcoo.com/api/get_product_complete?pi="));
    n_.param ("link2", link2_, std::string("&pins=ean&format=xml&source=ias-tum"));
    n_.param ("tag2", tag2_, std::string("picture_high"));
    n_.param ("tag3", tag3_, std::string("picture_low"));
    n_.param ("image_pattern", pattern_, std::string("<meta property=\"og:image\" content=\""));
    image_sub_ = it_.subscribe(
                               input_image_topic_, 1, &BarcodeReaderNode::imageCallback, this);
    barcode_pub_ =
    n_.advertise<std_msgs::String>(output_barcode_topic_, 1);
    enable_barcode_reader_ = 0;
    service_ = n.advertiseService("enable_barcode_reader_service", &BarcodeReaderNode::enable_barcode_reader,this);
    image_received_ = 0;
  }

  ~BarcodeReaderNode()
  {
    cv::destroyAllWindows();
  }
  
  bool enable_barcode_reader(zbar_barcode_reader_node::enable_barcode_reader::Request  &req,
		  zbar_barcode_reader_node::enable_barcode_reader::Response &res )
  {
	  enable_barcode_reader_ = req.enable;
	  while(!image_received_)
	  {
		  	sleep(1);
	  }

	  if(image_found_)
	  {
	  	  	res.title.data = product_title;
	  	  	res.subtitle.data = product_producer;
	  	  	res.category_key.data = product_category;
	  	  	res.image_msg = *ros_image;
	  	  	enable_barcode_reader_ = 0;
	  	    image_received_ = 0;
	  	    image_found_ = 0;
	  	    enable_barcode_reader_ = 0;

	  	    return true;
	  }
	  else
	  {
		  image_received_ = 0;
		  enable_barcode_reader_ = 0;
		  return false;
	  }
  }
  
  static void* CURL_realloc(void *ptr, size_t size)
  {
    /* There might be a realloc() out there that doesn't like reallocing
       NULL pointers, so we take care of it here */
    if(ptr)
      return realloc(ptr, size);
    else
      return malloc(size);
  }
  
  static size_t WriteMemoryCallback
  (void *ptr, size_t size, size_t nmemb, void *data)
  {
    UserData *userdata = reinterpret_cast<UserData *>(data);
    struct memoryStruct *mem = userdata->memory;
    //BarcodeReaderNode *self = userdata->self;
    size_t realsize = size * nmemb;
    
    mem->memory = (char *)
      CURL_realloc(mem->memory, mem->size + realsize + 1);
    if (mem->memory) {
      memcpy(&(mem->memory[mem->size]), ptr, realsize);
      mem->size += realsize;
      mem->memory[mem->size] = 0;
    }
    return realsize;
  }

    // This is the writer call back function used by curl  
  static int writer(char *data, size_t size, size_t nmemb,  
                    std::string *buffer)  
  {  
    // What we will return  
    int result = 0;  
    
    // Is there anything in the buffer?  
    if (buffer != NULL)  
      {  
        // Append the data to the buffer  
        buffer->append(data, size * nmemb);  
        
        // How much did we write?  
        result = size * nmemb;  
      }  
    
    return result;  
  } 
 
  int getImage(std::string image, cv::Mat * imgTmp)
  {
    CURL *curl;       // CURL objects
    CURLcode res;
    memoryStruct buffer; // memory buffer
    UserData userdata(&buffer, this);
    
    curl = curl_easy_init(); // init CURL library object/structure
    
    if(curl) 
      {
      // set up the write to memory buffer
      // (buffer starts off empty)
      buffer.memory = NULL;
      buffer.size = 0;
      
      // (N.B. check this URL still works in browser in case image has moved)
      curl_easy_setopt(curl, CURLOPT_URL, image.c_str());
      curl_easy_setopt(curl, CURLOPT_VERBOSE, 1); // tell us what is happening
      
      // tell libcurl where to write the image (to a dynamic memory buffer)

      curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::WriteMemoryCallback);
      curl_easy_setopt(curl,CURLOPT_WRITEDATA, (void *) &userdata);
      
      // get the image from the specified URL
      res = curl_easy_perform(curl);
      // decode memory buffer using OpenCV
      *imgTmp = cv::imdecode(cv::Mat(1, buffer.size, CV_8UC1, buffer.memory), CV_LOAD_IMAGE_UNCHANGED);
      // always cleanup

      curl_easy_cleanup(curl);
      free(buffer.memory);
  }
  return 1;
  }

  void findElement( TiXmlNode* pParent, std::string & picture, std:: string tag)
  {
    //if ( !pParent ) return;
    
    TiXmlNode* pChild;
    std::string pstring = pParent->Value();
    
    //Search for tag
    size_t found=pstring.find(tag);
    if (found!=std::string::npos)
    {
    	ROS_INFO_STREAM("First child: " << pParent->FirstChild()->Value());
    	picture =  pParent->FirstChild()->Value();
    	return;
    }
    
    for( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
    {	
    	findElement(pChild, picture,tag);
    }
  }


  int getPictureLink(std::string buffer,std::string & picture)
  {
    doc.Parse((const char*)buffer.c_str(), 0, TIXML_ENCODING_UTF8);
    
    //Check if there is a result
    std:: string result;
    findElement(&doc,result,tag1_);
    if(result.compare("0") == 0) //This condition checks if there is a result in the XML file if not returns 0
    {
      ROS_INFO_STREAM("Barcode does not correspond to any object in the barcoo database");
    	return 0;
    }
    
    //Search for first tag
    findElement (&doc, picture,tag2_);

    if(picture == "")
       	findElement (&doc, picture,tag3_); 	//Search for second tag

    ROS_INFO_STREAM ("Picture link: " << picture);
    if (picture == "")
      return -1;
    
    return 1;
  }


  int getBarcooXML(std::string bar_code, std::string & buffer)
  {
    char errorBuffer[CURL_ERROR_SIZE];
    // Our curl objects  
    CURL *curl;  
    CURLcode result;  
    // Create our curl handle  
    curl = curl_easy_init();  
    std::string full_url = link1_ + bar_code + link2_;
    ROS_INFO_STREAM("full_url: " << full_url);
   
    if (curl)  
      {  
	// Now set up all of the curl options  
	curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);  
	curl_easy_setopt(curl, CURLOPT_URL, full_url.c_str());  
	curl_easy_setopt(curl, CURLOPT_HEADER, 0);  
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);  
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::writer);  
    //	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);  
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);  

	// Attempt to retrieve the remote page  
	result = curl_easy_perform(curl);  

	// Always cleanup  
	curl_easy_cleanup(curl);

	// Did we succeed?  
	if (result == CURLE_OK)  
	  {  
	    ROS_INFO_STREAM ("CURLE_OK");
	    return 1;
	  }  
	else  
	  {  
	    ROS_INFO_STREAM ( "CURLE_NOT OK");
	    ROS_ERROR_STREAM ("Error: [" << result << "] - " << errorBuffer);
	    return -1;
	  }  
      }
    return -1;
  }
  
  int getHTMLpage(std::string url, std::string & buffer)
  {
	  char errorBuffer[CURL_ERROR_SIZE];
	  // Our curl objects
	  CURL *curl;
	  CURLcode result;
	  // Create our curl handle
	  curl = curl_easy_init();
      if (curl)
      {
    	  // Now set up all of the curl options
    	  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);
    	  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    	  curl_easy_setopt(curl, CURLOPT_HEADER, 0);
    	  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
    	  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &BarcodeReaderNode::writer);
	      //	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);
    	  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

    	  // Attempt to retrieve the remote page
    	  result = curl_easy_perform(curl);

    	  // Always cleanup
    	  curl_easy_cleanup(curl);

    	  // Did we succeed?
    	  if (result == CURLE_OK)
	  	  {

	  	    return 1;
	  	  }
    	  else
	  	  {
	  	    return -1;
	  	  }
	   }
	   return -1;
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
	  
	  if(!enable_barcode_reader_)
		  return;
    ROS_INFO("[BarcodeReaderNode: ] Image received");
    try
      {
	cv_bridge_ptr_ = cv_bridge::toCvCopy(msg_ptr, "mono8");
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Error converting ROS Image");
      }
    
    // create a reader
    ImageScanner scanner;

    // configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    int width = cv_bridge_ptr_->image.cols;   // extract dimensions
    int height = cv_bridge_ptr_->image.rows;

    // obtain image data
    Magick::Blob blob(cv_bridge_ptr_->image.ptr(0), width * height);

    const void *raw = blob.data();

    // wrap image data
    Image image(width, height, "Y800", raw, width * height);

    // scan the image for barcodes
    int n = scanner.scan(image);

    // extract results
    std::stringstream ss;
    for(Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) 
    {
      // do something useful with results
      ROS_INFO_STREAM("Publishing: barcode type: " << symbol->get_type_name()
                      << " barcode value " << symbol->get_data());

      std_msgs::String msg;

      //ss << symbol->get_type_name() << "," << symbol->get_data();
      ss << symbol->get_data();
      msg.data = ss.str();
      barcode_pub_.publish(msg);
    }
    if (n == 0)
    {
    	ROS_WARN("Barcode not found");
    	image_received_ = 1;
    	return;
    }

    if (n < 0)
    {
    	ROS_ERROR("Error occured while finding barcode");
    	image_received_ = 1;
    	return;
    }

    std::string buffer;
    //Get xml file from barcoo database
    int res = getBarcooXML(ss.str(), buffer);
    if(res != 1)
    {
    	image_received_ = 1;
    	return;
    }

    //Search for info in the xml file
    std::string pictureLink;
    res = getPictureLink (buffer,pictureLink);


    if(res == 0) //If barcode does not exist in the database return
    {
      ROS_INFO_STREAM("Barcode does not exist in the barcoo datbase");
    	image_received_ = 1;
    	return;
    }
    if(res == -1) //This condition is true when the image link is not given
    {
    	//Look for an image in the HTML file

    	//First find the link to the barcoo website of the file
    	std::string htmlPage;
    	std::string htmlLink;
    	findElement(&doc,htmlLink,"back_link");
    	ROS_INFO_STREAM ("html link:  " << htmlLink);
    	//Download HTML page
    	getHTMLpage(htmlLink,htmlPage);
    	std::string ss  = pattern_;
    	long position = htmlPage.find(ss, ss.length()) + ss.length();

    	std::string link;
    	while(htmlPage.at(position) !='"')
    	{
    		link += htmlPage.at(position);
    		position++;
    	}
    	pictureLink = link;

    }
    //Get Image
    if(!pictureLink.empty())
    {
    	cv::Mat  imgTmp; // image object
    	getImage (pictureLink ,& imgTmp);
    	// display image
    	//publish image
    	if (!(imgTmp.empty()))
    	{
    		cv_bridge_ptr_->header.stamp = ros::Time::now();
    		cv_bridge_ptr_->header.frame_id = "frame";
    		cv_bridge_ptr_->encoding = "bgr8";
    		cv_bridge_ptr_->image = imgTmp;
    		ros_image = cv_bridge_ptr_->toImageMsg();
    		findElement(&doc,product_title,"title");
    		findElement(&doc,product_producer,"producer");
    		findElement(&doc,product_category,"category_key");
    		image_found_ = 1;
    	}
    	image_received_ = 1;
    }
    cv::waitKey(3);
  }

protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher barcode_pub_;
  sensor_msgs::CvBridge bridge_;
  std::string input_image_topic_, output_barcode_topic_;
  sensor_msgs::ImagePtr ros_image;
  std::string product_title;
  std::string product_producer;
  std::string product_category;
  ros::ServiceServer service_;
  cv_bridge::CvImagePtr cv_bridge_ptr_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_reader_node");
  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();
  ros::NodeHandle n("~");
  BarcodeReaderNode br(n);
  ros::spin();

  return 0;
}
