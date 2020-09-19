#ifndef __LOCK_POSE
#define __LOCK_POSE
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include <highgui.h>
#include "fstream"
#include <ctime>
#include <cv.h>
#include <stdlib.h>
#include <iterator>
#include <valarray>
#include "IprovedFDCM.h"
#include "Rect_Detect.h"

int lock_pose(Mat image,float tag_T_cam[4][4]);

#endif
