#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

cv::Mat img;
static const char WINDOW[] = "TRACKER SUB";

static void CompressImageCallback1(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try{
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr_compressed->image;
        cv::imshow(WINDOW, img);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_INFO("convert fail");
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    if(argv[1] == NULL)
    {
        ROS_INFO("argv[1]=NULL\n");
        ROS_INFO("请输入参数！");
        return 1;
    }
    std::istringstream image_topic(argv[2]);
    std::string image_sub_topic;
    if(!(image_topic >> image_sub_topic))
    {
        ROS_INFO("image topic is invalid");
        return 1;
    }
    ROS_INFO_STREAM("The Trakcer Topic is : " << image_sub_topic);
    ros::init(argc, argv, "KCF_SUB");
    
    ros::NodeHandle nh("kcf_sub");
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::CompressedImage>(image_sub_topic, 1, CompressImageCallback1);
    cv::namedWindow(WINDOW);
    cv::startWindowThread();
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}
