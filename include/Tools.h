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
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

typedef unsigned int uint32;

class Param_reader
{
public:
    Param_reader(const string &fp);
    const int get_int(const string &key) const;
    const uint32 get_uint(const string &key) const;
    const double get_double(const string &key) const;
    const string get_string(const string &key) const;
    const bool get_bool(const string &key) const;
private:
    Param_reader();
    string file_path;
    map<string, string> params;//const成员函数访问map必须使用const型迭代器！！！
};

const Param_reader param("./paraments.ini");//在头文件中定义非const类型对象会出现重定义错误

const uint32 START_INDEX = param.get_uint("START_INDEX");
const uint32 END_INDEX = param.get_uint("END_INDEX");
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
const double MIN_NORM = param.get_double("MIN_NORM");
const bool VISUAL = param.get_bool("VISUAL");
const int FREQUE = param.get_int("FREQUE");
const string DETECT_TYPE = param.get_string("DETECT_TYPE");
const int NEARBY_CHECK_NUM = param.get_int("NEARBY_CHECK_NUM");
const int RANDOM_CHECK_NUM = param.get_int("RANDOM_CHECK_NUM");
//视差
const int MAX_DISP_NUM = param.get_int("MAX_DISP_NUM");
const int BORDER = param.get_int("BORDER");
const int SADWIN_SIZE = param.get_int("SADWIN_SIZE");
const int UNIQUE_RATIO = param.get_int("UNIQUE_RATIO");
const double DOFFS = param.get_int("DOFFS");

class DImage
{
public:
    DImage()
    {}
    DImage(const Mat &rgb_info, const Mat &depth_info);
    /*私有成员接口*/
    const Mat& rgb_info() const { return rgb_mat; }
    const Mat& depth_info() const { return depth_mat; }
private:
    Mat rgb_mat;
    Mat depth_mat;
};

class Frame
{
public:
    Frame()
    {}
    Frame(const DImage &dimage, const Mat &description, const vector<KeyPoint> &keypoints, const uint32 &i);
    /*私有成员接口*/
    const Mat& depth_info() const { return d_image.depth_info(); }
    const Mat& rgb_info() const { return d_image.rgb_info(); }
    const Mat& desp_info() const { return desp; }
    const DImage& get_dimage() const { return d_image; }
    const vector<KeyPoint>& key_points() const { return key_points_list; }
    uint32 get_id() const { return id; }
private:
    uint32 id;
    DImage d_image;
    Mat desp; //特征描述子
    vector<KeyPoint> key_points_list; //关键点集
};

class Transform_mat
{
public:
    /*由旋转矩阵r，位移向量t构造变换矩阵*/
    Transform_mat(const Mat &r, const Mat &t, const int &i);
    /*构造空变换矩阵*/
    static Transform_mat Empty_Transform_mat();
    /*私有成员接口*/
    const Eigen::Isometry3d& eigen_T() const { return T; }
    bool is_empty() const { return empty_flag; }
    double get_norm() const;
    int get_inliers() const { return inliers; }
private:
    Transform_mat();
    Mat rvec, tvec;//旋转矩阵 ,平移向量
    int inliers;
    Eigen::Isometry3d T;//齐次变换矩阵
    /*cv类型矩阵转换为eigen类型矩阵*/
    void cvmat_to_eigen();
    bool empty_flag;//是否为空矩阵
};