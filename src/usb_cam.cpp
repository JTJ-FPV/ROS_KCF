#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "ROS_KCF/kcftracker.hpp"
#include <dirent.h>

// 保存订阅的图像
cv::Mat img;
// roi区域，被追踪的区域
cv::Rect roi;
// 追踪结果
cv::Rect result;
// 追踪计数器
int nFrames(0);
bool draw;
bool range_roi;
// 发布追踪后的图像信息
image_transport::Publisher image_pub;
sensor_msgs::ImagePtr trackptr;
static const char WINDOW[] = "IMGAE VIEWER";

// KCF初始化参数
const bool HOG(true), FIXWINDOW(true), MULTISCALE(false), SILENT(false), LAB(false);
// 初始化KCF
KCFTracker tracker(HOG, FIXWINDOW, MULTISCALE, LAB);

void VaildRoi(cv::Rect &roi)
{
    if(roi.width < 0)
    {
        roi.x += roi.width;
        roi.width *= -1;
    }
    if (roi.height < 0)
    {
        roi.y += roi.height;
        roi.height *= -1;
    
    }
}

void onMouse(int event, int x, int y, int flags, void *param)
{
    switch (event)
    {
    //按下鼠标左键
    case cv::EVENT_LBUTTONDOWN:
        //初始化起始矩形框  
        roi =cv:: Rect(x, y, 0, 0);
        draw = true;
        break;

    //松开鼠标左键      
    case cv::EVENT_LBUTTONUP:
        if (roi.width < 0){
            roi.x += roi.width;
            roi.width *= -1;
        }
        if (roi.height < 0){
            roi.y += roi.height;
            roi.height *= -1;
        }
        draw = false;
        range_roi = true;
        break;
 
        //移动光标
    case cv::EVENT_MOUSEMOVE:
        if (draw)
        {
            roi.width = x - roi.x;
            roi.height = y - roi.y;
        }
        break;
    }
}


static void CompressImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try{
        // ROS_INFO("5" );
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr_compressed->image;
        if(0 == nFrames && range_roi)        // 第一帧需要初始化
        {
            ROS_INFO("Init Before");
            tracker.init(roi, img);
            ROS_INFO("Init After");
            // 框出区域
            cv::rectangle(img, roi, cv::Scalar(0, 255, 255), 1);
            nFrames++;
        }
        else if(nFrames > 0)
        {
            ROS_INFO("Tracking before");
            result = tracker.update(img);
            ROS_INFO("Tracking after");
            cv::rectangle(img, result, cv::Scalar(0, 255, 255), 3);
            nFrames++;
            trackptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            image_pub.publish(trackptr);
        }
        imshow(WINDOW, img);
    }
    catch(cv_bridge::Exception &e){
        ROS_INFO("convert fail" );
    }
}

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "usb_NetKCF");

    ros::NodeHandle nh("usb_cam");
    std::string image_topic = "/usb_cam/image_raw/compressed";
    std::string image_param= "/usb_cam/image_raw/compressed/format";
    range_roi = false;
    std::string image_format;
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::CompressedImage>(image_topic, 1, CompressImageCallback);
    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("Tracker/camera/image", 1);
    cv::namedWindow(WINDOW);
    cv::startWindowThread();
    ros::Rate Loop_rate(30);        // 30FPS
    while(ros::ok())
    {
         if(img.data!=NULL && 0 == nFrames)
        {   
            ROS_INFO("receivce the img");
            nh.getParam(image_param, image_format);
            ROS_INFO_STREAM("The format of image is :" <<  image_format);
            cvSetMouseCallback(WINDOW, onMouse, NULL);
            VaildRoi(roi);
            ROS_INFO_STREAM("roi.x:" <<  roi.x);
            ROS_INFO_STREAM( " roi.y:" << roi.y); 
            ROS_INFO_STREAM( "roi.height:" << roi.height);
            ROS_INFO_STREAM( "roi.width:" << roi.width);
        }
        ros::spinOnce();
        Loop_rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}
