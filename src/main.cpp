//#define DIS 
//#define DEP 
#define DEC

#include <iostream> 
#include <string> 
using namespace std;

#include "MyPointCloud.h"
#include "Disparity.h"
#include "Features.h"

//焦距
const double Factor = 1000;
//平移
const double Cx = 325.5;
const double Cy = 253.5;
//缩放系数
const double Fx = 518.0;
const double Fy = 519.0;
//目距
const double Tx = 40;

int main()
{
#ifdef DIS
    Mat raw_left = imread("./data/left1.png");
    Mat raw_right = imread("./data/right1.png");
    if(raw_left.empty() || raw_right.empty())
    {
        cout << "cannt find images" << endl;
        return -1;
    }
    else
    {}
    Mat disp = BM_get_disparity(raw_left, raw_right);
    imwrite("./data/disp.png", disp);
    create_point_cloud_by_disp(raw_left, disp);
#endif

#ifdef DEP
    Mat rgb = imread("./dep_data/rgb.png");
    Mat depth = imread("./dep_data/depth.png", -1);
    if(rgb.empty() || depth.empty())
    {
        cout << "cannt find images" << endl;
        return -1;
    }
    else
    {}
#endif

#ifdef DEC
    Mat rgb_before = imread("./dec_data/rgb1.png");
    Mat rgb_after = imread("./dec_data/rgb2.png");
    Mat depth_before = imread("./dec_data/depth1.png", -1);
    Mat depth_after = imread("./dec_data/depth2.png", -1);
    if(rgb_before.empty() || depth_before.empty())
    {
        cout << "cannt find images" << endl;
        return -1;
    }
    MyCamera camera = MyCamera(Factor, Cx, Cy, Fx, Fy, Tx);
    Feature_detector detector = Feature_detector(camera, "ORB");
    Frame frame1 = detector.detect_features(rgb_before, depth_before);
    Frame frame2 = detector.detect_features(rgb_after, depth_after);
    vector<DMatch> matches = detector.match_features(frame1, frame2);
    Result_of_PNP trans_mat = detector.estimate_motion(frame1, frame2, matches);
    cout << "inliers = " << trans_mat.inliers << endl;
    cout << "R = " << trans_mat.rvec << endl;
    cout << "t = "<< trans_mat.tvec << endl;
#endif

    cv::waitKey();
    return 0;
}