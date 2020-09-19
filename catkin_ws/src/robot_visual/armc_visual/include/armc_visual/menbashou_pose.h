#ifndef __MENBASHOU_POSE
#define __MENBASHOU_POSE

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include <highgui.h>
#include <cv.h>
#include "fstream"
#include <ctime>
#include <stdlib.h>
#include <iterator>
#include <valarray>
#include "process.h"
#include "print_matrix.h"


int menbashou_pose(cv::Mat image,float weizi[6]);

#endif
