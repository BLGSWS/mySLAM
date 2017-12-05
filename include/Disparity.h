# pragma once 

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 

/*视差检测模块*/
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

Mat BM_get_disparity(const Mat &left_rgb, const Mat &right_rgb);
Mat GC_get_disparity(const Mat &left_rgb, const Mat &right_rgb);


template<class T>
void mat_print(const cv::Mat &mat)
{
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            if(x%3 == 0 && y%3 == 0)
            {
                T pix = mat.at<T>(y, x);
                cout << pix << " ";
            }
            else
            {}
        }
        cout << endl;
    }
}