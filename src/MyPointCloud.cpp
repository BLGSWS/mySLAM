#include <iostream>
using namespace std;

#include "MyPointCloud.h"

Point_cloud::Ptr My_point_cloud::create_point_cloud_by_disp(const Mat &rgb, const Mat &disp)
{
    if(cloud_mount->points.size() != 0)
    {
        cout << "warning: cloud_mount is not null" << endl;
        cloud_mount->points.clear();
    }
    else
    {}

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
    if(cloud_mount->points.size() != 0)
    {
        cout << "warning: cloud_mount is not null" << endl;
        cloud_mount->points.clear();
    }
    else
    {}

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

void My_point_cloud::save_point_cloud(const string &file_path, Point_cloud::Ptr pc)
{
    pcl::io::savePCDFile(file_path, *pc);
    pc->points.clear();
    cout << "Point cloud saved in " << file_path << endl;
}
