# pragma once 

/*特征检测模块*/
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
using namespace cv;

#include "Camera.h"
#include "Tools.h"

class Feature_motion
{
public:
    virtual Frame detect_features(const DImage &dimage) const = 0;
    virtual vector<DMatch> match_features(const Frame &frame_before, const Frame &frame_after) const = 0;
    virtual Transform_mat estimate_motion(const Frame &frame_before, const Frame &frame_after, const vector<DMatch> &matches) const = 0;
    virtual ~Feature_motion() = 0;
};

class PNP_motion: public Feature_motion
{
public:
    PNP_motion(MyCamera *mycamera, const string &detect_type = DETECT_TYPE);
    Frame detect_features(const DImage &dimage) const;
    vector<DMatch> match_features(const Frame &frame_before, const Frame &frame_after) const;
    Transform_mat estimate_motion(const Frame &frame_before, const Frame &frame_after, const vector<DMatch> &matches) const;
private:
    MyCamera *camera;
    string type;
};

