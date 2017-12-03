#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

class Frame
{
public:
    Frame(const Mat &rgb_img, const Mat &depth_img):
    rgb(rgb_img), depth(depth_img)
    {}
    Mat rgb;
    Mat depth;
    Mat desp; //特征描述子
    vector<KeyPoint> key_points; //关键点集
private:
    Frame();
};

class Result_of_PNP
{
public:
    Result_of_PNP(const Mat &r, const Mat &t, const int &i):
    rvec(r), tvec(t), inliers(i)
    {}
    Mat rvec, tvec;
    int inliers;
private:
    Result_of_PNP();
};

class Tools
{
public:
    static Eigen::Isometry3d cvMat2Eigen(const Mat& rvec, const Mat& tvec)
    {
        /*rvec to eigen_R*/
        Mat cv_R;
        Rodrigues(rvec, cv_R);
        Matrix3d eigen_R;
        cv2eigen(cv_R, eigen_R);

        /*eigen_R and tvec to T*/
        Isometry3d T = Eigen::Isometry3d::Identity();
        AngleAxisd angle(eigen_R);
        Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));//???
        T = angle;
        T(0,3) = tvec.at<double>(0,0);
        T(1,3) = tvec.at<double>(0,1);
        T(2,3) = tvec.at<double>(0,2);
        return T;
    }
};