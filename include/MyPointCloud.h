# pragma once 

#include <string>

#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 

#include <pcl/common/transforms.h> //变换矩阵
#include <pcl/filters/voxel_grid.h> //滤波器
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

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
    Point_cloud::Ptr join_point_cloud(const Frame &frame, const Result_of_PNP &pnp_result);
    Point_cloud::Ptr create_first_point_cloud(const Frame &frame);
    Point_cloud::Ptr get_cloud() const;
    void save_point_cloud(const string &file_path);
    void read_point_cloud(const string &file_path);
private:
    void create_point_cloud_by_disp(const Mat &rgb, const Mat &disp);
    void create_point_cloud_by_depth(const Mat &rgb, const Mat &disp);
    My_point_cloud();
    MyCamera camera;
    Point_cloud::Ptr cloud;
    Point_cloud::Ptr cloud_mount;
};