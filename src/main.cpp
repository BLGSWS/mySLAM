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
    MyCamera camera = MyCamera(Factor, Cx, Cy, Fx, Fy, Tx);
    /*定义特征处理对象*/
    Feature_detector detector = Feature_detector(camera, "ORB");
    /*定义点云对象*/
    My_point_cloud pcloud(camera);
    pcl::visualization::CloudViewer viewer("viewer");
    /*用队列管理帧，为多线程挖坑*/
    queue<Frame> frame_que;
    bool flag = true;


    for(unsigned int i = 1u; i <= PIC_NUM; i++)
    {
        /*字符串转个数字还搞这么麻烦*/
        stringstream ss;
        ss << i;
        cout << "processing " << i << "th image" << endl;
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

        /*建立帧并入队*/
        Frame frame = detector.detect_features(rgb, depth);
        frame_que.push(frame);

        /*判断是否为第一帧*/
        if(frame_que.size() == 1)
        {
            if(flag == true)
            {
                /*生成点云*/
                pcloud.create_first_point_cloud(frame);
                flag == false;
            }
            else continue; //非多线程应该不会进入else分支
        }
        else
        {
            Frame frame_before = frame_que.front(); 
            vector<DMatch> matches = detector.match_features(frame_before, frame);
            
            /*pnp求解*/
            Result_of_PNP trans_mat = detector.estimate_motion(frame_before, frame, matches);

            /*inliers太小放弃该帧*/
            if(trans_mat.inliers <= MIN_INLIERS)
            {
                cout << "inliers = " << trans_mat.inliers << endl;
                cout << "main: inliers is too small, give up this frame" << endl;
                continue;
            }
            else
            {}
            
            /*打印位移信息*/
            cout << "T = " << trans_mat.T.matrix() << endl;

            /*生成并合并点云*/
            Point_cloud::Ptr cloud = pcloud.join_point_cloud(frame, trans_mat);

            /*实时显示点云*/
            if(VISUAL == true && i % FREQUE == 0)
                viewer.showCloud(cloud);
            else
            {}

            /*处理完成，旧帧出队列*/
            frame_que.pop();
        }
        cout << endl;
    }

    /*保存点云*/
    pcloud.save_point_cloud("./data/result.pcd");
#endif

    cv::waitKey();
    return 0;
}