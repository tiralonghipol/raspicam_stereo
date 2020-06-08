#pragma once

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <numeric>
#include <algorithm>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/optflow.hpp>
#include <eigen3/Eigen/Dense>

#define VCOS_ALIGN_DOWN(p,n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p,n) VCOS_ALIGN_DOWN((ptrdiff_t)(p)+(n)-1,(n))

using namespace std;
using namespace cv;

cv::Mat dynamicThreWihteBalance(cv::Mat image);
int getWhitePointThre(cv::Mat whiteRegion);
void WhitePointMask(cv::Mat Cr, cv::Mat Cb, cv::Mat RL);
cv::Mat choiceWhitePoint(cv::Mat YCrCb, int mBlocks, int nBlocks);

image_transport::Publisher _pub_image_left;
image_transport::Publisher _pub_image_right;
int _frame_count = 0;
int _width;
int _height;

int _node_rate_loop;

