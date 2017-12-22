#include "VisualOdometry.h"

VO::VO(Feature_motion *pdetector, Point_cloud_map *pcloud, 
int min_inliers, int min_matches, double max_norm):
vo_min_inliers(min_inliers), vo_max_norm(max_norm),
vo_min_matches(min_matches), first_frame_flag(true)
{
    if (pdetector == 0 || pcloud == 0)
    {
        cout << "VO::VO: parament is null ptr" << endl;
        throw exception();
    }
    else
    {
        vo_detector = pdetector;
        vo_point_cloud = pcloud;
    }
}

Transform_mat VO::process_frame(const DImage &dimage)
{

    current_frame = vo_detector->detect_features(dimage);

    /*判断是否为第一帧*/
    if(first_frame_flag == true)
    {
        //Point_cloud::Ptr cloud = vo_point_cloud->join_point_cloud(current_frame.get_dimage(), Transform_mat::Empty_Transform_mat());
        first_frame_flag = false;
        last_frame = current_frame;
        cout << endl;
        return Transform_mat::Empty_Transform_mat();
    }
    else
    {
        vector<DMatch> matches = vo_detector->match_features(last_frame, current_frame);
        if(matches.size() < vo_min_matches)
        {
            cout << "VO::process_frame: matched feature is too less, give up this frame" << endl << endl;
            return Transform_mat::Empty_Transform_mat();
        }
        else
        {}

        /*pnp求解*/
        Transform_mat trans_mat = vo_detector->estimate_motion(last_frame, current_frame, matches);
        
        /*inliers太小放弃该帧*/
        if(trans_mat.get_inliers() <= MIN_INLIERS || trans_mat.get_norm() >= MAX_NORM)
        {
            cout << "inliers = " << trans_mat.get_inliers() << endl;
            cout << "norm = " << trans_mat.get_norm() << endl;
            cout << "VO::process_frame: inliers is too small, give up this frame" << endl << endl;
            return Transform_mat::Empty_Transform_mat();
        }
        else
        {}

        /*打印位移信息*/
        cout << "T = " << trans_mat.eigen_T().matrix() << endl;
        cout << endl;

        /*生成并合并点云*/
        //Point_cloud::Ptr cloud = vo_point_cloud->join_point_cloud(current_frame.get_dimage(), trans_mat);
        
        /*处理完成，新旧帧交换*/
        last_frame = current_frame;

        /*返回位置变化信息，供后端优化器求解*/
        return trans_mat;
    }
}