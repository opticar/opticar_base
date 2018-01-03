#include <cmath>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


ros::Time gTimeLastLoop;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "base_controller");
  
  //ros::NodeHandle nhRoot;
  //ros::NodeHandle nhOpticar("~");
  
  //ros::Publisher pubOdometry = nhRoot.advertise<nav_msgs::Odometry>("odom", 50);
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  const int width = 256;
  const int height = 256;
  
  unsigned char red = 255;
  
  cv::Mat image (width, height, CV_8UC3, cv::Scalar(0,0,0));
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  
  ros::Rate loop_rate(10);
  while(nh.ok())
  {
    int v = 0;
    for (v = 0; v < height/2; ++v)
    {
      for (int u = 0; u < width; ++u)
      {
        auto& pixel = image.at<cv::Vec3b>(v, u);
        pixel[0] = 0;
        pixel[1] = 255 - red;
        pixel[2] = red;
      }
    }
    for (; v < height; ++v)
    {
      for (int u = 0; u < width; ++u)
      {
        auto& pixel = image.at<cv::Vec3b>(v, u);
        pixel[0] = 0;
        pixel[1] = red;
        pixel[2] = 255 - red;
      }
    }
    
    --red;
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}