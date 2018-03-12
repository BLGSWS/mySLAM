//#define VOTEST
//#define KITTI
#define VOCLASS

#include <iostream>
#include <string>
#include <queue>
using namespace std;

#include "Map.h"
#include "Pretreat.h"
//#include "Features.h"
#include "VisualOdometry.h"

int main()
{
#ifdef VOTEST
    /*定义相机对象*/
    DepthCamera depth_camera = DepthCamera(Factor, Cx, Cy, Fx, Fy);
    MyCamera *camera = &depth_camera;//栈内对象，不需要智能指针
    /*定义特征处理对象*/
    PNP_motion detector = PNP_motion(camera, DETECT_TYPE);
    /*定义点云对象*/
    Point_cloud_map pcloud(camera, GRID_SIZE);
    /*定义pclviewer对象*/
    pcl::visualization::CloudViewer viewer("viewer");
    /*定义优化器对象*/
    MyOptimizer optimizer;

    Frame current_frame, last_frame;
    bool first_frame_flag = true;

    for(unsigned int i = START_INDEX; i <= END_INDEX; i++)
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
            Transform_mat trans_mat = Transform_mat::Empty_Transform_mat();
            optimizer.add_edge(i, trans_mat);
        }
        else
        {
            vector<DMatch> matches = detector.match_features(last_frame, current_frame);
            
            /*pnp求解*/
            Transform_mat trans_mat = detector.estimate_motion(last_frame, current_frame, matches);

            /*inliers太小放弃该帧*/
            if(trans_mat.get_inliers() <= MIN_INLIERS || trans_mat.get_norm() >= MAX_NORM)
            {
                cout << "inliers = " << trans_mat.get_inliers() << endl;
                cout << "norm = " << trans_mat.get_norm() << endl;
                cout << "main: inliers is too small, give up this frame" << endl << endl;
                continue;
            }
            else
            {}
            
            /*打印位移信息*/
            cout << "T = " << trans_mat.eigen_T().matrix() << endl;

            /*生成并合并点云*/
            Point_cloud::Ptr cloud = pcloud.join_point_cloud(current_frame, trans_mat);

            /*实时显示点云*/
            if(VISUAL == true && i % FREQUE == 0)
                viewer.showCloud(cloud);
            else
            {}
            
            /*加入新点， 建立新边*/
            optimizer.add_edge(i, trans_mat);

            /*处理完成，旧帧出队列*/
            last_frame = current_frame;
        }
        cout << endl;
    }

    /*优化并保存结果*/
    optimizer.optimize();
    optimizer.save_result("./data/result_after.g2o");
    
    /*保存点云*/
    pcloud.save_point_cloud("./data/result.pcd");
#endif

#ifdef KITTI
    DispCamera disp_camera = DispCamera(Factor, Cx, Cy, Fx, Fy, Tx, DOFFS);
    MyCamera *camera = &disp_camera;
    Mat left_rgb = imread("./kitti_test/left/0000000000.png");
    Mat right_rgb = imread("./kitti_test/right/0000000000.png");
    Mat disp = BM_get_disparity(left_rgb, right_rgb);
    //imwrite("./kitti_test/test/disp.png", disp);
    //disp = imread("./kitti_test/test/disp.png", -1);
    DImage dimage(left_rgb, disp);
    //Frame frame(DImage);
    Point_cloud_map pcloud(camera, GRID_SIZE);
    pcloud.create_first_point_cloud(dimage);
    pcloud.save_point_cloud("./kitti_test/result.pcd");
#endif

#ifdef VOCLASS
    /*定义相机对象*/
    DepthCamera depth_camera = DepthCamera(Factor, Cx, Cy, Fx, Fy);
    MyCamera *camera = &depth_camera;//栈内对象，不需要智能指针

    /**/
    Depth_pretreat bm_pret;
    Depth_pretreat* pret = &bm_pret;

    /*定义特征处理对象*/
    PNP_motion detector = PNP_motion(camera);
    Feature_motion *p_detector = &detector;

    /*定义pclviewer对象*/
    Cloud_viewer viewer("viewer");
    Cloud_viewer *p_viewer = &viewer;

    /*定义点云对象*/
    Point_cloud_map pcloud(camera, p_viewer);
    Point_cloud_map *p_pcloud = &pcloud;

    /*定义优化器对象*/
    MyOptimizer optimizer;
    MyOptimizer* p_optimizer = &optimizer;

    /*定义回环检测对象*/
    Easy_Loop_check loop_ckeck(p_detector, p_optimizer);
    Easy_Loop_check* p_loop_check = &loop_ckeck;

    /*定义视觉里程计对象*/
    VO vo = VO(p_detector, p_loop_check);

    for (uint32 i = START_INDEX; i <= END_INDEX; i++)
    {
        /*读取文件*/
        stringstream ss;
        ss << i;
        cout << "processing " << i << "th image..." << endl;
        /*读取图像*/
        string rgb_filename = "./data/rgb_png/" + ss.str() + ".png";
        string depth_filename = "./data/depth_png/" + ss.str() + ".png";
        Mat rgb = imread(rgb_filename);
        Mat depth = imread(depth_filename, -1);
        if (rgb.empty() || depth.empty())
        {
            cout << rgb_filename << " " << depth_filename << endl;
            cout << "main: cannt find images" << endl;
            return -1;
        }
        else
        {}
        DImage dimage = pret->pretreat(rgb, depth);

        vo.process(dimage);
    }

    /*优化并保存结果*/
    optimizer.optimize();
    optimizer.save_result("./data/result_after.g2o");
    
    /*建图*/
    vector<Frame> key_frames = vo.get_key_frames();
    for (size_t i = 0; i < key_frames.size(); i++)
    {
        Frame frame = key_frames[i];
        Eigen::Isometry3d pose = optimizer.get_pose(key_frames[i].get_id());
        cout << pose.matrix() << endl;
        pcloud.join_point_cloud(key_frames[i].get_dimage(), pose);
    }
    /*保存点云*/
    //pcloud.save_point_cloud("./data/result.pcd");

#endif
    cv::waitKey();
    return 0;
}