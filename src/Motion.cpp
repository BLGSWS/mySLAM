#include <iostream>
#include <vector>
using namespace std;

#include "Motion.h"

Feature_motion::~Feature_motion()
{}

PNP_motion::PNP_motion(MyCamera *mycamera, const string &detect_type):
Feature_motion(), type(detect_type)
{
    /*入参检测*/
    if(mycamera != 0)
        camera = mycamera;
    else
    {
        cout << "PNP_motion: null camera ptr" << endl;
        throw exception();
    }
}

Frame PNP_motion::detect_features(const DImage &dimage) const
{
    /*特征提取器和描述子提取器*/
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;

    /*设定提取方法, 有ORB\SURF\SIFT三种*/
    if (type == "ORB")
    {}
    else if (type == "SURF" || type == "SIFT")
    {
        initModule_nonfree();
    }
    else
    {
        throw exception();
    }
    detector = FeatureDetector::create(type);
    descriptor = DescriptorExtractor::create(type);

    /*提取关键点*/
    vector<KeyPoint> key_points;
    detector->detect(dimage.rgb_info(), key_points);

    /*显示特征*/
    /*cv::Mat img_show;
    cv::drawKeypoints(dimage.rgb_info(), key_points, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("keypoints", img_show);
    cv::imwrite("./data/keypoints.png", img_show);
    cv::waitKey(0);*/

    /*计算特征描述子*/
    Mat desp;
    descriptor->compute(dimage.rgb_info(), key_points, desp);

    /*返回Frame结构*/
    Frame frame(dimage, desp, key_points);
    return frame;
}

vector<DMatch> PNP_motion::match_features(const Frame &frame_before, const Frame &frame_after) const
{
    /*匹配特征描述子*/
    vector<DMatch> matches;
    BFMatcher matcher;
    matcher.match(frame_before.desp_info(), frame_after.desp_info(), matches);

    /*求特征距离的最小值*/
    double min_dis = 9999.0;
    for (size_t i = 0u; i < matches.size(); i++)
    {
        if (matches[i].distance < min_dis && matches[i].distance != 0)
        {
            min_dis = matches[i].distance;
        }
        else
        {}
    }
    cout << "match_features: min dis is " << min_dis << endl;

    if(min_dis < 10) min_dis = 10;
    else
    {}

    /*选择相对距离较小的特征*/
    vector<DMatch>::iterator it = matches.begin();
    while (it != matches.end())
    {
        if ((*it).distance > 10 * min_dis)
        {
            it = matches.erase(it);
        }
        else
        {
            it++;
        }
    }
    cout << "match_features: number of reserved features is " << matches.size() << endl;

    /*显示匹配特征*/
    /*Mat img_matches;
    drawMatches(frame_before.rgb_info(), frame_before.key_points, frame_after.rgb_info(), frame_after.key_points, matches, img_matches);
    imshow( "good matches", img_matches );
    waitKey(0);*/

    return matches;
}

Transform_mat PNP_motion::estimate_motion(const Frame &frame_before, const Frame &frame_after, const vector<DMatch> &matches) const
{
    vector<Point3f> pts_obj;
    vector<Point2f> pts_img;

    /*创建求解PNP需要的入参*/
    for (size_t i = 0u; i < matches.size(); i++)
    {
        Point2f p1 = frame_before.key_points()[matches[i].queryIdx].pt;
        ushort d = frame_before.depth_info().ptr<ushort>(int(p1.y))[int(p1.x)];
        if(d == 0) continue;
        else
        {
            Point2f p2 = frame_after.key_points()[matches[i].trainIdx].pt;
            pts_img.push_back(p2);
            Point3f p1_3d(p1.x, p1.y, d);
            Point3f p1_3d_transed = camera->point2dto3d(p1_3d);
            pts_obj.push_back(p1_3d_transed);
        }
    }
    Mat rvec, tvec, inliers;
    /*构建相机矩阵*/
    double camera_matrix_data[3][3] = {
        {camera->Fx, 0, camera->Cx},
        {0, camera->Fy, camera->Cy},
        {0, 0, 1}
    };
    Mat camera_mat = Mat(3, 3, CV_64F, camera_matrix_data);//为什么构造函数里赋值就不行呢TAT

    /*求解PNP*/
    solvePnPRansac(pts_obj, pts_img, camera_mat, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

    /*构造变换矩阵结构体*/
    Transform_mat result(rvec, tvec, inliers.rows);
    return result;
}