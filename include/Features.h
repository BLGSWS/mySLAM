# pragma once 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*特征检测模块*/
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
using namespace cv;

#include "Camera.h"
#include "Tools.h"

class Feature_detector
{
public:
    Feature_detector(const MyCamera &camera, const string &t);
    Frame detect_features(const Mat &rgb, const Mat &depth);
    vector<DMatch> match_features(const Frame &frame_before, const Frame &frame_after);
    Result_of_PNP estimate_motion(const Frame &frame_before, const Frame &frame_after, const vector<DMatch> &matches);
private:
    Mat camera_mat;
    MyCamera camera;
    string type;
};

