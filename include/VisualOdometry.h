#pragma once

#include <iostream>
#include <string>
using namespace std;

#include "Motion.h"
#include "Map.h"

class VO
{
public:
    VO(Feature_motion *pdetector, Point_cloud_map *pcloud, 
    int min_inliers = MIN_INLIERS, int min_matches = MIN_MATCHES, 
    double max_norm = MAX_NORM);
    Transform_mat process_frame(const DImage &dimage);
private:
    Feature_motion* vo_detector;
    Point_cloud_map* vo_point_cloud;
    Cloud_viewer* vo_viewer;
    /*参数*/
    int vo_min_inliers;
    double vo_max_norm;
    int vo_min_matches;
    /*管理帧*/
    Frame current_frame, last_frame;
    /*是否为第一帧*/
    bool first_frame_flag;
};