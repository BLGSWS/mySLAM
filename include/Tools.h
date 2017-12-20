# pragma once 
/*STL*/
#include <string>
#include <map>
#include <iostream>
#include <fstream>
/*Eigen*/
#include <Eigen/Core>
#include <Eigen/Geometry>
/*opencv*/
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>//KeyPoint
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

class Param_reader
{
public:
    Param_reader(const string &fp);
    int get_int(const string &key) const;
    double get_double(const string &key) const;
    string get_string(const string &key) const;
    bool get_bool(const string &key) const;
private:
    Param_reader();
    string file_path;
    map<string, string> params;//const成员函数访问map必须使用const型迭代器！！！
};

const Param_reader param("./paraments.ini");//在头文件中定义非const类型对象会出现重定义错误

/*相机内参*/
//焦距
const double Factor = param.get_double("Factor");
//平移
const double Cx = param.get_double("Cx");
const double Cy = param.get_double("Cy");
//缩放系数
const double Fx = param.get_double("Fx");
const double Fy = param.get_double("Fy");
//目距
const double Tx = param.get_double("Tx");

const string TYPE = param.get_string("TYPE");
const double GRID_SIZE = param.get_double("GRID_SIZE");
const int MIN_MATCHES = param.get_int("MIN_MATCHES");
const int MIN_INLIERS = param.get_int("MIN_INLIERS");
const double MAX_NORM = param.get_double("MAX_NORM");
const bool VISUAL = param.get_bool("VISUAL");
const int FREQUE = param.get_int("FREQUE");
const int PIC_NUM = param.get_int("PIC_NUM");
const string DETECT_TYPE = param.get_string("DETECT_TYPE");
//视差
const int DISP_NUM = param.get_int("DISP_NUM");
const int BORDER = param.get_int("BORDER");
const int SADWIN_SIZE = param.get_int("SADWIN_SIZE");
const int UNIQUE_RATIO = param.get_int("UNIQUE_RATIO");
const double DOFFS = param.get_int("DOFFS");

class Frame
{
public:
    Frame()
    {}
    Frame(const Mat &rgb_img, const Mat &depth_img):
    rgb(rgb_img), depth(depth_img) 
    {}
    Mat rgb;
    Mat depth;
    Mat desp; //特征描述子
    vector<KeyPoint> key_points; //关键点集
};

class Transform_mat
{
public:
    Transform_mat(const Mat &r, const Mat &t, const int &i);
    Mat rvec, tvec;
    int inliers;
    Eigen::Isometry3d T;
    double norm;
private:
    void cvmat_to_eigen();
    void norm_of_transform();
    Transform_mat();
};