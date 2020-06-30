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
    IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, _quality};
    BUFFER *buffer = arducam_capture(camera_instance, &fmt, 300);
    if (!buffer) 
        return NULL;
    
    // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format 
	// and the IMAGE_ENCODING_I420 format are aligned, 
    // width 32 bytes aligned, and height 16 byte aligned.
    width = VCOS_ALIGN_UP(width, 32);
    height = VCOS_ALIGN_UP(height, 16);
    cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
    arducam_release_buffer(buffer);
    return image;
}

int main(int argc, char **argv) {
    
	ros::init(argc, argv, "raspicam_stereo_node");
	ros::NodeHandle n("raspicam_stereo_node");

	n.getParam("node_rate_loop", _node_rate_loop);
	n.getParam("width", _width);
	n.getParam("height", _height);
	n.getParam("quality", _quality);
	n.getParam("auto_exposure", _auto_exposure);
	n.getParam("exposure_value", _exposure_value);
	n.getParam("auto_white_balance", _auto_white_balance);
	n.getParam("auto_wb_compensation", _auto_wb_compensation);
	n.getParam("red_gain", _red_gain);
	n.getParam("blue_gain", _blue_gain);
	n.getParam("binning", _binning);
	n.getParam("binning_type", _binning_type);

	image_transport::ImageTransport it(n);
    _pub_full_image = it.advertise("image_full", 1);

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
	else
	{
		if (arducam_set_control(camera_instance, V4L2_CID_EXPOSURE, (int)(_exposure_value*0xFFFF/200.0))) {
            ROS_INFO("Failed to set exposure, the camera may not support this control.");
        }
	}
	if(_auto_white_balance)
	{
		ROS_INFO("Enable Auto White Balance...");
    	if (arducam_software_auto_white_balance(camera_instance, 1)) {
        	ROS_INFO("Automatic white balance not supported");
    	}
	}
	if(_auto_wb_compensation)
	{
		ROS_INFO("Enable Auto White Balance Compensation...");
    	arducam_manual_set_awb_compensation(_red_gain,_blue_gain);
	}
//  ------------------- registers messup ---------------------
// binning mode:
// 2: no-binning, 1: x2-binning,
// 0: x4-binning, 3: x2-analog (special)
        ROS_INFO("Enable Binning Mode H_A...");
        if (arducam_write_sensor_reg(camera_instance, 0x0174, (uint16_t)_binning)) {
            ROS_WARN("Failed to write sensor register.");
        }
        ROS_INFO("Enable Binning Mode H_A...");
        if (arducam_write_sensor_reg(camera_instance, 0x0175, (uint16_t)_binning)) {
            ROS_WARN("Failed to write sensor register.");
        }
// binning tyoe (H-direction).
// 0 :average, 1: sum
        ROS_INFO("Enable BINNING_CAL_MODE_H_A...");
        if (arducam_write_sensor_reg(camera_instance, 0x0176, (uint16_t)_binning_type)) {
            ROS_WARN("Failed to write sensor register.");
        }
        ROS_INFO("Enable BINNING_CAL_MODE_V_A...");
        if (arducam_write_sensor_reg(camera_instance, 0x0177, (uint16_t)_binning_type)) {
            ROS_WARN("Failed to write sensor register.");
        }
// ---------------------------------------------------

	int seq = 0;
 	ros::Rate loop_rate(_node_rate_loop);
	while (n.ok()) 
	{
		cv::Mat *image = get_image(camera_instance, _width, _height);
        if(!image)
            continue;
		cv::Mat image_in = image->clone();

		seq = seq+1;
		sensor_msgs::Image::Ptr image_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_in).toImageMsg();
		// sensor_msgs::Image::Ptr image_right_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right).toImageMsg();
		
		image_out->header.frame_id = "cam_full";
		image_out->header.stamp = ros::Time::now();
		image_out->header.seq = seq;

		// image_right_out->header.frame_id = "cam_right";
		// image_right_out->header.stamp = ros::Time::now();
		// image_right_out->header.seq = seq;

        if (_pub_full_image.getNumSubscribers() > 0)
            _pub_full_image.publish(image_out);

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
