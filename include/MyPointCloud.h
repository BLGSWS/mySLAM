# pragma once 

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

//usr/inculde下寻找
//PCL
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 

#include "Camera.h"
#include "Tools.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Point_cloud;

class My_point_cloud
{
public:
    My_point_cloud(const MyCamera &c): camera(c),
    cloud(Point_cloud::Ptr(new Point_cloud)),
    cloud_mount(Point_cloud::Ptr(new Point_cloud))
    {}
    Point_cloud::Ptr create_point_cloud_by_disp(const Mat &rgb, const Mat &disp);
    Point_cloud::Ptr create_point_cloud_by_depth(const Mat &rgb, const Mat &disp);
    Point_cloud::Ptr join_point_cloud(const Frame &frame, const Eigen::Isometry3d &T);
    void save_point_cloud(const string &file_path, Point_cloud::Ptr pc);
    void read_point_cloud(const string &file_path);
private:
    My_point_cloud();
    MyCamera camera;
    Point_cloud::Ptr cloud;
    Point_cloud::Ptr cloud_mount;
};