# pragma once 

#include <string>

#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 

#include <pcl/common/transforms.h> //变换矩阵
#include <pcl/filters/voxel_grid.h> //网格滤波器
#include <pcl/filters/passthrough.h> //z方向滤波器
#include <pcl/visualization/cloud_viewer.h>

#include "Camera.h"
#include "Tools.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Point_cloud;
typedef pcl::visualization::CloudViewer Cloud_viewer;

class Point_cloud_map
{
public:
    Point_cloud_map(MyCamera *c, Cloud_viewer* viewer = 0, const double &voxel_grid_size = GRID_SIZE);
    Point_cloud::Ptr join_point_cloud(const DImage &dimage, const Eigen::Isometry3d &trans_mat);
    Point_cloud::Ptr get_cloud() const;
    void save_point_cloud(const string &file_path) const;
    //void read_point_cloud(const string &file_path);
protected:
    Point_cloud::Ptr create_point_cloud(const DImage &dimage);
private:
    Point_cloud_map();
    Cloud_viewer* cloud_viewer;
    MyCamera *camera;
    double grid_size;
    Point_cloud::Ptr cloud;
};