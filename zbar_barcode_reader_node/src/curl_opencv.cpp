#include <stdio.h>
#include <curl/curl.h>

#include <cv.h>
#include <highgui.h>
using namespace cv;

// *****************************************************************************

struct memoryStruct {
  char *memory;
  size_t size;
};

static void* CURL_realloc(void *ptr, size_t size)
{
  /* There might be a realloc() out there that doesn't like reallocing
     NULL pointers, so we take care of it here */
  if(ptr)
    return realloc(ptr, size);
  else
    return malloc(size);
}

size_t WriteMemoryCallback
(void *ptr, size_t size, size_t nmemb, void *data)
{
  size_t realsize = size * nmemb;
  struct memoryStruct *mem = (struct memoryStruct *)data;

  mem->memory = (char *)
    CURL_realloc(mem->memory, mem->size + realsize + 1);
  if (mem->memory) {
    memcpy(&(mem->memory[mem->size]), ptr, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;
  }
  return realsize;
}

// *****************************************************************************

int main(void)
{
  CURL *curl;       // CURL objects
  CURLcode res;
  cv::Mat imgTmp; // image object
  memoryStruct buffer; // memory buffer

  curl = curl_easy_init(); // init CURL library object/structure

  if(curl) {

    // set up the write to memory buffer
    // (buffer starts off empty)

    buffer.memory = NULL;
    buffer.size = 0;

    // (N.B. check this URL still works in browser in case image has moved)

    curl_easy_setopt(curl, CURLOPT_URL, "http://www.froodies.de/FroodiesImages/images/products/large/29278.jpg");
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1); // tell us what is happening

    // tell libcurl where to write the image (to a dynamic memory buffer)

    curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl,CURLOPT_WRITEDATA, (void *) &buffer);

    // get the image from the specified URL

    res = curl_easy_perform(curl);

    // decode memory buffer using OpenCV

    imgTmp = cv::imdecode(cv::Mat(1, buffer.size, CV_8UC1, buffer.memory), CV_LOAD_IMAGE_UNCHANGED);

    // display image (if we got / decoded it correctly)

    namedWindow("Image from URL", CV_WINDOW_AUTOSIZE);
    if (!(imgTmp.empty()))
      {
	imshow("Image from URL", imgTmp);
      }
    waitKey(0);


    // always cleanup

    curl_easy_cleanup(curl);
    free(buffer.memory);

  }
  return 0;
}
