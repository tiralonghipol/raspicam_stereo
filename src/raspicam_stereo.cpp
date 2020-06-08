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
    cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
    arducam_release_buffer(buffer);
    return image;
}

void CountTemperature(const Mat result,Mat &Temperature ,const short phi=180)
{
    Mat_<Vec3f>::const_iterator rit=result.begin<Vec3f>();
    Mat_<Vec3f>::const_iterator ritend=result.end<Vec3f>();
    float Y=0,Cb=0,Cr=0,n=0,Z=0,Y1=0,Cb1=0,Cr1=0;
    for(;rit!=ritend;++rit,++n)
    {
        Y1=(*rit)[0];
        Cb1=(*rit)[1];
        Cr1=(*rit)[2];
        Z=Y1-abs(Cb1)-abs(Cr1);
        if (Z>phi)
            {Y+=Y1;
             Cb+=Cb1;
             Cr+=Cr1;
            }
    }
    Temperature.at<Vec3f>(0,0)[0]=Y/n;
    Temperature.at<Vec3f>(0,0)[1]=Cb/n;
    Temperature.at<Vec3f>(0,0)[2]=Cr/n;
}

void gain(Mat Temperature,float &u,float&v,float lamda=0.05)
{
    float Y_T=Temperature.at<Vec3f>(0,0)[0];
    float Cb_T=Temperature.at<Vec3f>(0,0)[1];
    float Cr_T=Temperature.at<Vec3f>(0,0)[2];

    if(abs(Cb_T)>abs(Cr_T))
        if(Cb_T>0)
            u-=lamda;
        else
           u+=lamda;
    else
        if(Cr_T>0)
            v-=lamda;
        else
           v+=lamda;
}

void correctionImage(const Mat image,Mat &result,const float u ,const float v)
{
    Mat_<Vec3b>::const_iterator it=image.begin<Vec3b>();
    Mat_<Vec3b>::const_iterator itend=image.end<Vec3b>();
    Mat_<Vec3b>::iterator rit=result.begin<Vec3b>();
    Mat_<Vec3b>::iterator ritend=result.end<Vec3b>();
    for(;it!=itend;++it,++rit)
    {
        (*rit)[0]=saturate_cast<uchar>(u*(*it)[0]);//B
        (*rit)[1]=saturate_cast<uchar>((*it)[1]);//G
        (*rit)[2]=saturate_cast<uchar>(v*(*it)[2]);//R
    }
}

Mat choiceWhitePoint(Mat YCrCb, int mBlocks, int nBlocks) {
	std::vector<cv::Mat> channels;
	cv::split(YCrCb, channels);
	Mat Cr = channels[1];
	Mat Cb = channels[2];
	int height = Cr.rows, width = Cr.cols;
	int blockWidth = width / nBlocks;
	int blockHeight = height / mBlocks;
	Mat RL = cv::Mat::zeros(Cr.rows, Cr.cols, CV_8UC1);
	for (int i = 0; i < mBlocks - 1; i++) {
		int startRow = blockHeight * i;
		int endRow = blockHeight * (i + 1);
		for (int j = 0; j < nBlocks - 1; j++) {
			int startCol = blockWidth * j;
			int endCol = blockWidth * (j + 1);
			WhitePointMask(Cr(Range(startRow, endRow), Range(startCol, endCol)), 
					Cb(Range(startRow, endRow), Range(startCol, endCol)),
					RL(Range(startRow, endRow), Range(startCol, endCol)));
		}
		WhitePointMask(Cr(Range(startRow, endRow), Range(blockWidth * (nBlocks-1), width)), 
				Cb(Range(startRow, endRow), Range(blockWidth * (nBlocks-1), width)), 
				RL(Range(startRow, endRow), Range(blockWidth * (nBlocks-1), width)));
	}
	WhitePointMask(Cr(Range(blockHeight * (mBlocks-1), height), Range(blockWidth * (nBlocks-1), width)), 
			Cb(Range(blockHeight * (mBlocks-1), height), Range(blockWidth * (nBlocks-1), width)), 
			RL(Range(blockHeight * (mBlocks-1), height), Range(blockWidth * (nBlocks-1), width)));
	return RL;
}
void WhitePointMask(Mat Cr, Mat Cb, Mat RL) {
	//imshow("Cr", Cr);
	//imshow("Cb", Cb);
	//waitKey(0);
	Mat savg,sfangcha;
	meanStdDev(Cb, savg, sfangcha);
	double Mb=savg.at<double>(0);
	double Db=sfangcha.at<double>(0);//求出第一部分cb的均值和均方差
	meanStdDev(Cr,savg,sfangcha);
	double Mr=savg.at<double>(0);
	double Dr=sfangcha.at<double>(0);;
	double b,r;
	if (Mb<0)//计算mb+db*sign（mb）
	{ 
		b=Mb+Db*(-1);
	}
	else
		b=Mb+Db;

	
	if (Mr<0)//计算1.5*mr+dr*sign（mb）；
	{
		r=1.5*Mr+Dr*(-1);
	}
	else
		r=1.5*Mr+Dr;
	//Mat mask =  abs(Cb - b) < 1.5 * Db;
	Mat mask;
	//bitwise_and(mask, abs(Cr - r) < 1.5 *Dr, mask);
	bitwise_and(abs(Cb - b) < 1.5 * Db, abs(Cr - b) < 1.5 * Dr, mask);
	RL.setTo(255, mask);
	//imshow("Cbmask", abs(Cb - b) < 1.5 * Db);
	//imshow("Crmask", abs(Cr - b) < 1.5 * Dr);
   	//imshow("and", mask);	
	//waitKey(0);

}

int getWhitePointThre(Mat whiteRegion) {
	int value[256] = {0};
	int width = whiteRegion.cols;
	int height = whiteRegion.rows;
	int counter = 0;
	for (int i = 0; i < height; i++) {
		uchar *rowDataPtr = whiteRegion.ptr<uchar>(i);
		for (int j = 0; j < width; j++) {
			if (rowDataPtr[j] != 0) {
				value[rowDataPtr[j]]++;
				counter++;
			}
		}
	}
	int sum = 0;
	int thre;
	for (thre = 255; thre > 0; thre--) {
		sum += value[thre];
		if (sum >= counter * 0.1)
			break;
	}
	return thre;
}


Mat dynamicThreWihteBalance(Mat image) {
	Mat YCrCb;
	cv::cvtColor(image, YCrCb, cv::COLOR_BGR2YCrCb);
	int mBlocks=3, nBlocks = 4;
	Mat RLMask = choiceWhitePoint(YCrCb, mBlocks, nBlocks);
	//imshow("RLMask", RLMask);
	std::vector<cv::Mat> channels;
	cv::split(YCrCb, channels);
	Mat RL = channels[0];
	Mat Yimg = channels[0].clone();
	double Ymin, Ymax;
	cv::minMaxLoc(Yimg, &Ymin, &Ymax);
	Mat notRLMask;
	cv::bitwise_not(RLMask, notRLMask);
	//imshow("notRL mask", notRLMask);
	RL.setTo(0, notRLMask);
	//imshow("RL", RL);
	int thre = getWhitePointThre(RL);
	cout << "thre:" << thre << endl;
	RLMask.setTo(0, RL < thre);
	cv::split(image, channels);
	Mat B, G, R;
	B = channels[0].clone();
	G = channels[1].clone();
	R = channels[2].clone();
	cv::bitwise_not(RLMask, notRLMask);
	//imshow("notRLMask", notRLMask);
	//waitKey(0);
	float meanB = sum(B.setTo(0, notRLMask))[0] / float(countNonZero(RLMask));
	float meanG = sum(G.setTo(0, notRLMask))[0] / float(countNonZero(RLMask));
	float meanR = sum(R.setTo(0, notRLMask))[0] / float(countNonZero(RLMask));
	Ymax /= 3;
	float gainB = Ymax / meanB;
	float gainG = Ymax / meanG;
	float gainR = Ymax / meanR;
	cout << "gain:" << gainB << " " << gainG << " " << gainR << " " << Ymax << endl;
	channels[0] = channels[0] * gainB;	
	channels[1] = channels[1] * gainG;	
	channels[2] = channels[2] * gainR;
	cv::merge(channels, image);

	return image;
}

int main(int argc, char **argv) {
    
	ros::init(argc, argv, "raspicam_stereo_node");
	ros::NodeHandle n("raspicam_stereo_node");

	n.getParam("node_rate_loop", _node_rate_loop);
	n.getParam("width", _width);
	n.getParam("height", _height);

	image_transport::ImageTransport it(n);
    _pub_image_left = it.advertise("cam_left", 1);
    _pub_image_right = it.advertise("cam_right", 1);

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
    ROS_INFO("Enabling Auto Exposure...");
    arducam_software_auto_exposure(camera_instance, 1);
	ROS_INFO("Enable Auto White Balance...");
    if (arducam_software_auto_white_balance(camera_instance, 1)) {
        ROS_INFO("Mono camera does not support automatic white balance.");
    }

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

		sensor_msgs::Image::Ptr image_left_out = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_left).toImageMsg();
		sensor_msgs::Image::Ptr image_right_out = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_right).toImageMsg();
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