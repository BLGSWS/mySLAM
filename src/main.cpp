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
#ifdef DEC
    /*定义相机对象*/
    MyCamera camera = MyCamera(Factor, Cx, Cy, Fx, Fy, Tx);
    /*定义特征处理对象*/
    Feature_detector detector = Feature_detector(camera, "ORB");
    /*定义点云对象*/
    My_point_cloud pcloud(camera);

    /*读取图片*/
    Mat rgb_before = imread("./dec_data/rgb1.png");
    Mat rgb_after = imread("./dec_data/rgb2.png");
    Mat depth_before = imread("./dec_data/depth1.png", -1);
    Mat depth_after = imread("./dec_data/depth2.png", -1);
    if(rgb_before.empty() || depth_before.empty())
    {
        cout << "cannt find images" << endl;
        return -1;
    }
    
    /*求解相机运动*/
    Frame frame1 = detector.detect_features(rgb_before, depth_before);
    Frame frame2 = detector.detect_features(rgb_after, depth_after);
    vector<DMatch> matches = detector.match_features(frame1, frame2);
    Result_of_PNP trans_mat = detector.estimate_motion(frame1, frame2, matches);
    cout << "inliers = " << trans_mat.inliers << endl;
    cout << "R = " << trans_mat.rvec << endl;
    cout << "t = "<< trans_mat.tvec << endl;

    /*生成并合成点云*/
    pcloud.create_first_point_cloud(frame1);
    Point_cloud::Ptr pc_ptr = pcloud.join_point_cloud(frame2, trans_mat);
    pcloud.save_point_cloud("./dec_data/join.pcd", pc_ptr);
#endif

    cv::waitKey();
    return 0;
}