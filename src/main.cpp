#define VO

#include <iostream>
#include <string>
#include <queue>
using namespace std;

#include "MyPointCloud.h"
#include "Disparity.h"
#include "Features.h"

int main()
{
#ifdef VO
    /*定义相机对象*/
    DepthCamera depth_camera = DepthCamera(Factor, Cx, Cy, Fx, Fy, Tx);
    MyCamera *camera = &depth_camera;//栈内对象，不需要智能指针
    /*定义特征处理对象*/
    Feature_detector detector = Feature_detector(camera, DETECT_TYPE);
    /*定义点云对象*/
    My_point_cloud pcloud(camera, GRID_SIZE);
    /*定义pclviewer对象*/
    pcl::visualization::CloudViewer viewer("viewer");

    Frame current_frame, last_frame;
    bool first_frame_flag = true;

    for(unsigned int i = 1u; i <= PIC_NUM; i++)
    {
        /*字符串转个数字还搞这么麻烦*/
        stringstream ss;
        ss << i;
        cout << "processing " << i << "th image..." << endl;
        /*读取图像*/
        string rgb_filename = "./data/rgb_png/" + ss.str() + ".png";
        string depth_filename = "./data/depth_png/" + ss.str() + ".png";
        Mat rgb = imread(rgb_filename);
        Mat depth = imread(depth_filename, -1);
        if(rgb.empty() || depth.empty())
        {
            cout << rgb_filename << " " << depth_filename << endl;
            cout << "main: cannt find images" << endl;
            return -1;
        }

        /*建立帧*/
        current_frame = detector.detect_features(rgb, depth);

        /*判断是否为第一帧*/
        if(first_frame_flag == true)
        {
            pcloud.create_first_point_cloud(current_frame);
            first_frame_flag = false;
            last_frame = current_frame;
        }
        else
        {
            vector<DMatch> matches = detector.match_features(last_frame, current_frame);
            
            /*pnp求解*/
            Transform_mat trans_mat = detector.estimate_motion(last_frame, current_frame, matches);

            /*inliers太小放弃该帧*/
            if(trans_mat.inliers <= MIN_INLIERS || trans_mat.norm >= MAX_NORM)
            {
                cout << "inliers = " << trans_mat.inliers << endl;
                cout << "norm = " << trans_mat.norm << endl;
                cout << "main: inliers is too small, give up this frame" << endl << endl;
                continue;
            }
            else
            {}
            
            /*打印位移信息*/
            cout << "T = " << trans_mat.T.matrix() << endl;

            /*生成并合并点云*/
            Point_cloud::Ptr cloud = pcloud.join_point_cloud(current_frame, trans_mat);

            /*实时显示点云*/
            if(VISUAL == true && i % FREQUE == 0)
                viewer.showCloud(cloud);
            else
            {}

            /*处理完成，旧帧出队列*/
            last_frame = current_frame;
        }
        cout << endl;
    }

    /*保存点云*/
    pcloud.save_point_cloud("./data/result.pcd");
#endif

    cv::waitKey();
    return 0;
}