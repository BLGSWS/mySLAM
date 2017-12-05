# pragma once 

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>//KeyPoint
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

const string TYPE = "depth";
const double GRIDSIZE = 0.02;

class Frame
{
public:
    Frame(const Mat &rgb_img, const Mat &depth_img):
    rgb(rgb_img), depth(depth_img) 
    {}
    Mat rgb;
    Mat depth;
    Mat desp; //特征描述子
    vector<KeyPoint> key_points; //关键点集
private:
    Frame();
};

class Result_of_PNP
{
public:
    Result_of_PNP(const Mat &r, const Mat &t, const int &i):
    rvec(r), tvec(t), inliers(i)
    {}
    Mat rvec, tvec;
    int inliers;
private:
    Result_of_PNP();
};

class Tools
{
public:
    static Eigen::Isometry3d cvmat_to_eigen(const Mat& rvec, const Mat& tvec);
};