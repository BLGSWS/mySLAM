#include "Tools.h"

Eigen::Isometry3d Tools::cvmat_to_eigen(const Mat& rvec, const Mat& tvec)
{
    /*rvec to eigen_R*/
    Mat cv_R;
    Rodrigues(rvec, cv_R);
    Eigen::Matrix3d eigen_R;
    cv2eigen(cv_R, eigen_R);

    /*eigen_R and tvec to T*/
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(eigen_R);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));//???
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);
    return T;
}