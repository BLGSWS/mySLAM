# pragma once 

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 

/*视差检测模块*/
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Tools.h"

using namespace cv;

Mat BM_get_disparity(const Mat &left_rgb, const Mat &right_rgb);
Mat GC_get_disparity(const Mat &left_rgb, const Mat &right_rgb);