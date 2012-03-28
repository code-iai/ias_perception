#ifndef ZBARGUI_H
#define ZBARGUI_H

#include <QtGui/QMainWindow>
#include "ui_zbargui.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

namespace Ui {
class zbarGui;
}

class zbarGui : public QMainWindow, public Ui::zbarGui
{
public:
    Q_OBJECT
    
public:
    explicit zbarGui(QWidget *parent = 0);
    ~zbarGui();

    ros::NodeHandle n_;
    image_transport::Subscriber image_sub_;
    void cameraDisplay(const sensor_msgs::ImageConstPtr& msg_ptr);
    ros::Subscriber uvc_sub;
    sensor_msgs::CvBridge bridge_;
    ros::ServiceClient client;

//private:
//    Ui::zbarGui *ui;

signals:
    	void SIG_updateImage(const IplImage*);
    	void SIG_updateImage2(const IplImage*);

public slots:
    void doStart();
    void doQuit();
    void SLT_updateImage(const IplImage* pIplImage);
    void SLT_updateImage2(const IplImage* pIplImage);
};



#endif // ZBARGUI_H
