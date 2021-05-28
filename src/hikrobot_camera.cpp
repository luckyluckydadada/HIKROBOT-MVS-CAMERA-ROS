#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    
    //********** 10 Hz        **********/
    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }

        cv_ptr->image = src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
    }
    return 0;
}
