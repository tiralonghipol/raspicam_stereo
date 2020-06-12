#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/v4l2-controls.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <opencv2/xphoto/white_balance.hpp>
#include "raspicam_stereo/arducam_mipicamera.h"
#include "raspicam_stereo/raspicam_stereo.h"


cv::Mat *get_image(CAMERA_INSTANCE camera_instance, int width, int height) {
    IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    BUFFER *buffer = arducam_capture(camera_instance, &fmt, 3000);
    if (!buffer) 
        return NULL;
    
    // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format 
	// and the IMAGE_ENCODING_I420 format are aligned, 
    // width 32 bytes aligned, and height 16 byte aligned.
    width = VCOS_ALIGN_UP(width, 32);
    height = VCOS_ALIGN_UP(height, 16);
    cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2RGB_I420);
    arducam_release_buffer(buffer);
    return image;
}

int main(int argc, char **argv) {
    
	ros::init(argc, argv, "raspicam_stereo_node");
	ros::NodeHandle n("raspicam_stereo_node");

	n.getParam("node_rate_loop", _node_rate_loop);
	n.getParam("width", _width);
	n.getParam("height", _height);
	n.getParam("auto_exposure", _auto_exposure);
	n.getParam("auto_white_balance", _auto_white_balance);

	image_transport::ImageTransport it(n);
    _pub_image_left = it.advertise("cam_left", 3);
    _pub_image_right = it.advertise("cam_right", 3);

	CAMERA_INSTANCE camera_instance;

    ROS_INFO("Opening camera...");
    int res = arducam_init_camera(&camera_instance);
    if (res) {
        ROS_INFO("init camera status = %d", res);
        return -1;
    }
    ROS_INFO("Setting resolution...");
    res = arducam_set_resolution(camera_instance, &_width, &_height);
    if (res) {
        ROS_INFO("set resolution status = %d", res);
        return -1;
    } else {
        ROS_INFO("current resolution is %dx%d", _width, _height);
    }
	if(_auto_exposure)
	{
		ROS_INFO("Enabling Auto Exposure...");
		arducam_software_auto_exposure(camera_instance, 1);
	}
	if(_auto_white_balance)
	{
		ROS_INFO("Enable Auto White Balance...");
    	if (arducam_software_auto_white_balance(camera_instance, 1)) {
        	ROS_INFO("Mono camera does not support automatic white balance.");
    	}
	}
	int seq = 0;
 	ros::Rate loop_rate(_node_rate_loop);
	while (n.ok()) 
	{
		cv::Mat *image = get_image(camera_instance, _width, _height);
        if(!image)
            continue;
		cv::Mat image_in = image->clone();
		
		// Mat h_flippedImg;
		// Mat flippedImg;
		// cv::flip(*image,h_flippedImg, 0);
		// cv::flip(h_flippedImg,flippedImg, 1);
		// Mat image_left = *image(Rect(0, image.rows/2, image.cols, image.rows/2));
		Mat image_left = image_in(Rect(0, 0, image_in.cols/2, image_in.rows)); 
		Mat image_right = image_in(Rect(image_in.cols/2, 0, image_in.cols/2, image_in.rows));

		// h = std_msgs::Header();
		seq = seq+1;
		sensor_msgs::Image::Ptr image_left_out = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_left).toImageMsg();
		sensor_msgs::Image::Ptr image_right_out = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_right).toImageMsg();
		
		image_left_out->header.frame_id = "cam_left";
		image_left_out->header.stamp = ros::Time::now();
		image_left_out->header.seq = seq;

		image_right_out->header.frame_id = "cam_right";
		image_right_out->header.stamp = ros::Time::now();
		image_right_out->header.seq = seq;

        if (_pub_image_left.getNumSubscribers() > 0)
            _pub_image_left.publish(image_left_out);
        if (_pub_image_right.getNumSubscribers() > 0)
            _pub_image_right.publish(image_right_out);

		ros::spinOnce();
		loop_rate.sleep();
	
		delete image;
    }
    
    ROS_INFO("Close camera...");
    res = arducam_close_camera(camera_instance);
    if (res) 
    	ROS_INFO("close camera status = %d", res);
    
    return 0;
}