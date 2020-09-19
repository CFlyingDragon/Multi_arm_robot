#ifndef __CAPTURE_H
#define __CAPTURE_H


#include <iostream>
#include "fstream"
#include <ctime>
#include <stdlib.h>
#include <iterator>
#include <valarray>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>

#include "FlyCapture2.h"

#include "IprovedFDCM.h"
#include "Rect_Detect.h"
#include "process.h"
#include "print_matrix.h"

#define PI 3.1415926


int menbashou_pose(cv::Mat image,float weizi[6]);

int lock_pose(Mat image,float tag_T_cam[4][4]);

int lock_capture_pose(float tag_T_cam[4][4]);

int handle_capture_pose(float twist[6]);

#endif
