#include <iostream>
using namespace std;

#include "Map.h"

Point_cloud_map::Point_cloud_map(MyCamera *mycamera, Cloud_viewer* viewer, const double &voxel_grid_size): 
cloud_viewer(viewer), grid_size(voxel_grid_size),
cloud(Point_cloud::Ptr(new Point_cloud()))
{
    /*入参检测*/
    if(mycamera != 0)
        camera = mycamera;
    else
    {
        cout << "Point_cloud_map: null camera ptr" << endl;
        throw exception();
    }
}

Point_cloud::Ptr Point_cloud_map::create_point_cloud(const DImage &dimage)
{
    Point_cloud::Ptr cloud_mount(new Point_cloud());

    /*深度信息图必须由ushort类型的深度图储存*/
    if(dimage.depth_info().type() != 2)
    {
        cout << "create_point_cloud warning: depth info does not store by ushort gray" << endl;
    }
    else
    {}

    /*重建点云*/
    for(unsigned int i = 0u; i < dimage.depth_info().rows; i++)
    {
        for(unsigned int j = 0u; j < dimage.depth_info().cols; j++)
        {
            ushort z = dimage.depth_info().ptr<ushort>(i)[j];
            if (z <= 0) continue;
            else
            {
                PointT p;
                p.x = (double)j;
                p.y = (double)i;
                p.z = (double)z;
                /*确定坐标*/
                PointT np = camera->point2dto3d(p);
                /*染色*/
                np.b = dimage.rgb_info().at<Vec3b>(i, j)[0];
                np.g = dimage.rgb_info().at<Vec3b>(i, j)[1];
                np.r = dimage.rgb_info().at<Vec3b>(i, j)[2];

                cloud_mount->points.push_back(np);
            }
        }
    }
    cloud_mount->height = 1;
    cloud_mount->width = cloud_mount->points.size();
    cloud_mount->is_dense = false;
    return cloud_mount;
}

Point_cloud::Ptr Point_cloud_map::join_point_cloud(const DImage &dimage, const Transform_mat &t_mat)
{
    if(cloud->points.size() == 0 && t_mat.is_empty())
    {
        cloud = create_point_cloud(dimage);
        cout << "join_point_cloud: number of points is " << cloud->points.size() << endl;
        return cloud;
    }
    else
    {}

    Point_cloud::Ptr cloud_mount(new Point_cloud());
    /*建立点云*/
    cloud_mount = create_point_cloud(dimage);

    /*旋转点云*/
    Point_cloud::Ptr transformed_cloud(new Point_cloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, t_mat.eigen_T().matrix());//旋转的是原始点云？？？
    *cloud_mount += *transformed_cloud;

    /*滤波*/
    static pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(grid_size, grid_size, grid_size);
    voxel.setInputCloud(cloud_mount);
    voxel.filter(*cloud);
    cout << "join_point_cloud: number of points is " << cloud->points.size() << endl;

    /*显示点云*/
    if(cloud_viewer != 0)
    {
        cloud_viewer->showCloud(cloud);
    }
    else
    {}

    return cloud;
}

Point_cloud::Ptr Point_cloud_map::get_cloud() const
{
    if(cloud != 0)
        return cloud;
    else
        throw exception();
}

void Point_cloud_map::save_point_cloud(const string &file_path) const
{
    pcl::io::savePCDFile(file_path, *cloud);
    cout << "Point cloud saved in " << file_path << endl;
}
