#include <iostream>
using namespace std;

#include "Disparity.h"

Mat BM_get_disparity(const Mat &left_rgb, const Mat &right_rgb)
{
    Mat left_gray, right_gray, disp_temp, disp;
    StereoBM bm;

    /*设置参数*/
    int SADWindowSize = SADWIN_SIZE;
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize: 9; //SDA窗口大小
    bm.state->minDisparity = 0;  
    bm.state->numberOfDisparities = DISP_NUM;//最大搜索视差数
    bm.state->textureThreshold = 10;  
    bm.state->uniquenessRatio = UNIQUE_RATIO; //唯一视差百分比
    bm.state->speckleWindowSize = 10;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;

    /*判断是否为rgb图*/
    if(left_rgb.channels() != 3
        && left_rgb.channels() != 4
    || (right_rgb.channels() != 3
        && right_rgb.channels() != 4))
    {
        cout << "number of left_rgb channel is " << left_rgb.channels() << endl;
        cout << "number of right_rgb channel is " << right_rgb.channels() << endl;
        cout << "not 3 or 4 channel picture" << endl;
        throw exception();
    }
    else
    {}

    /*转换为灰度图像*/
    cvtColor(left_rgb, left_gray, CV_BGR2GRAY);
    cvtColor(right_rgb, right_gray, CV_BGR2GRAY);

    /*扩充边界，防止黑边*/
    int border = BORDER;
    copyMakeBorder(left_gray, left_gray, 0, 0, border, 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(right_gray, right_gray, 0, 0, border, 0, IPL_BORDER_REPLICATE);

    bm(left_gray, right_gray, disp_temp, CV_32F);
    disp_temp.convertTo(disp, CV_16U, 1, 0);
    
    /*剪裁边界*/
    disp = disp.colRange(border, left_gray.cols);

    return disp;
}

Mat GC_get_disparity(const Mat &left_rgb, const Mat &right_rgb)
{
    Mat left_gray, right_gray;

    cvtColor(left_rgb, left_gray, CV_BGR2GRAY);
    cvtColor(right_rgb, right_gray, CV_BGR2GRAY);

    CvStereoGCState *state = cvCreateStereoGCState( 16, 2 );
    CvMat left_cvmat = left_gray;
    CvMat right_cvmat = right_gray;
    CvMat *p_left = &left_cvmat;
    CvMat *p_right = &left_cvmat;
    CvMat *p_left_disp = cvCreateMat( p_left->rows, p_left->cols, CV_32F );  
    CvMat *p_right_disp = cvCreateMat( p_right->rows, p_right->cols, CV_32F );  
    cvFindStereoCorrespondenceGC( p_left, p_right, p_left_disp, p_right_disp, state, 0 ); 
    cvReleaseStereoGCState( &state );

    Mat left_disp(p_left_disp);
    return left_disp;
}