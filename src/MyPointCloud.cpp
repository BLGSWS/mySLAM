#include <iostream>
using namespace std;

#include "MyPointCloud.h"

Point_cloud::Ptr My_point_cloud::create_point_cloud_by_disp(const Mat &rgb, const Mat &disp)
{
    Point_cloud::Ptr cloud_mount(new Point_cloud());

    if(rgb.rows != disp.rows || rgb.cols != disp.cols)
    {
        cout << "size of rgb is " << rgb.rows << ", " << rgb.cols << endl;
        cout << "size of disp is " << disp.rows << ", " << disp.cols << endl;
        throw exception();
    }
    else
    {}

    for (unsigned int u = 0u; u < rgb.rows; ++u)
    {
        for (unsigned int v = 0u; v < rgb.cols; ++v)
        {
            if((double)disp.at<Vec3b>(u, v)[0] <= 0) continue;
            else
            {
                PointT p;
                p.z = (double)disp.ptr<ushort>(u)[v];
                p.x = (double)v;
                p.y = (double)u;

                PointT np = camera.point2dto3d_by_disp(p);
                np.b = rgb.ptr<uchar>(u)[v*3];
                np.g = rgb.ptr<uchar>(u)[v*3 + 1];
                np.r = rgb.ptr<uchar>(u)[v*3 + 2];

                cloud_mount->points.push_back(np);
            }
        }
    }
    cloud_mount->height = 1;
    cloud_mount->width = cloud_mount->points.size();
    cout << "point cloud size = " << cloud_mount->points.size() << endl;
    cloud_mount->is_dense = false;
    return cloud_mount;
}

Point_cloud::Ptr My_point_cloud::create_point_cloud_by_depth(const Mat &rgb, const Mat &depth)
{
    Point_cloud::Ptr cloud_mount(new Point_cloud());

    if(rgb.rows != depth.rows || rgb.cols != depth.cols)
    {
        cout << "size of rgb is " << rgb.rows << ", " << rgb.cols << endl;
        cout << "size of disp is " << depth.rows << ", " << depth.cols << endl;
        throw exception();
    }
    else
    {}

    for(unsigned int u = 0u; u < depth.rows; u++)
    {
        for(unsigned int v = 0u; v < depth.cols; v++)
        {
            if (depth.ptr<ushort>(u)[v] <= 0) continue;
            else
            {
                PointT p;
                p.x = (double)v;
                p.y = (double)u;
                p.z = (double)depth.ptr<ushort>(u)[v];
                /*确定坐标*/
                PointT np = camera.point2dto3d_by_depth(p);
                /*染色*/
                np.b = rgb.ptr<uchar>(u)[v*3];
                np.g = rgb.ptr<uchar>(u)[v*3 + 1];
                np.r = rgb.ptr<uchar>(u)[v*3 + 2];

                cloud_mount->points.push_back(np);
            }
        }
    }
    cloud_mount->height = 1;
    cloud_mount->width = cloud_mount->points.size();
    cout << "point cloud size = " << cloud_mount->points.size() << endl;
    cloud_mount->is_dense = false;
    return cloud_mount;
}

Point_cloud::Ptr My_point_cloud::join_point_cloud(const Frame &frame, const Result_of_PNP &pnp_result)
{
    if(cloud->points.size() == 0)
    {
        cout << "join_point_cloud: need first point cloud" << endl;
        throw exception();
    }

    Point_cloud::Ptr cloud_mount(new Point_cloud());
    /*建立点云*/
    if(TYPE == "depth")
        cloud_mount = create_point_cloud_by_depth(frame.rgb, frame.depth);
    else
        cloud_mount = create_point_cloud_by_disp(frame.rgb, frame.depth);

    /*旋转点云*/
    Point_cloud::Ptr transformed_cloud(new Point_cloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, pnp_result.T.matrix());//旋转的是原始点云？？？
    *cloud_mount += *transformed_cloud;

    /*滤波*/
    static pcl::VoxelGrid<PointT> voxel;
    double grid_size = GRID_SIZE;
    voxel.setLeafSize(grid_size, grid_size, grid_size);
    voxel.setInputCloud(cloud_mount);
    voxel.filter(*cloud);
    cout << "join_point_cloud: number of points is " << cloud->points.size() << endl;
    return cloud;
}

Point_cloud::Ptr My_point_cloud::create_first_point_cloud(const Frame &frame)
{
    /*建立点云*/
    if(TYPE == "depth")
        cloud = create_point_cloud_by_depth(frame.rgb, frame.depth);
    else
        cloud = create_point_cloud_by_disp(frame.rgb, frame.depth);
    
    cout << "join_point_cloud: number of points is " << cloud->points.size() << endl;
    return cloud;
}

Point_cloud::Ptr My_point_cloud::get_cloud() const
{
    if(cloud != 0)
        return cloud;
    else
        throw exception();
}

void My_point_cloud::save_point_cloud(const string &file_path) const
{
    pcl::io::savePCDFile(file_path, *cloud);
    cout << "Point cloud saved in " << file_path << endl;
}
