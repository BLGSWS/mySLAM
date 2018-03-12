#include <iostream>
using namespace std;

#include "Map.h"

Point_cloud_map::Point_cloud_map(MyCamera *mycamera, Cloud_viewer* viewer, const double &voxel_grid_size): 
camera(mycamera), cloud_viewer(viewer), grid_size(voxel_grid_size),
cloud(Point_cloud::Ptr(new Point_cloud()))
{
    /*入参检测*/
    if(mycamera == 0)
    {
        cout << "Point_cloud_map: null camera ptr" << endl;
        throw exception();
    }
    else {}
}

Point_cloud::Ptr Point_cloud_map::create_point_cloud(const DImage &dimage)
{
    Point_cloud::Ptr new_cloud(new Point_cloud());

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

                new_cloud->points.push_back(np);
            }
        }
    }
    new_cloud->height = 1;
    new_cloud->width = new_cloud->points.size();
    new_cloud->is_dense = false;
    return new_cloud;
}

Point_cloud::Ptr Point_cloud_map::join_point_cloud(const DImage &dimage, const Eigen::Isometry3d &trans_mat)
{
    /*设置滤波器*/
    static pcl::VoxelGrid<PointT> voxel;
    static pcl::PassThrough<PointT> pass;
    voxel.setLeafSize(grid_size, grid_size, grid_size);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 4.0);

    Point_cloud::Ptr new_cloud(new Point_cloud());
    Point_cloud::Ptr temp(new Point_cloud());

    /*建立点云*/
    new_cloud = create_point_cloud(dimage);

    /*旋转点云*/
    Point_cloud::Ptr transformed_cloud(new Point_cloud());
    pcl::transformPointCloud(*new_cloud, *transformed_cloud, trans_mat.matrix());

    /*拼接点云*/
    *cloud += *transformed_cloud;
    cout << "join_point_cloud: number of points is " << cloud->points.size() << endl;

    /*滤波*/
    voxel.setInputCloud(cloud);
    voxel.filter(*temp);
    pass.setInputCloud(temp);
    pass.filter(*cloud);

    /*显示点云*/
    if (cloud_viewer != 0)
    {
        cloud_viewer->showCloud(cloud); 
    }
    else {}

    /*释放资源ps:还要老夫亲手释放？*/
    new_cloud->clear();
    temp->clear();

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
