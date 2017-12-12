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
    My_point_cloud(MyCamera *c, const double &voxel_grid_size = 0.01);
    Point_cloud::Ptr join_point_cloud(const Frame &frame, const Transform_mat &t_mat);
    Point_cloud::Ptr create_first_point_cloud(const Frame &frame);
    Point_cloud::Ptr get_cloud() const;
    void save_point_cloud(const string &file_path) const;
    //void read_point_cloud(const string &file_path);
private:
    Point_cloud::Ptr create_point_cloud(const Mat &rgb, const Mat &depth);
    My_point_cloud();
    MyCamera *camera;
    double grid_size;
    Point_cloud::Ptr cloud;
};