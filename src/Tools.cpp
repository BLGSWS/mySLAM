#include "Tools.h"
#include <sstream>

DImage::DImage(const Mat &rgb_info, const Mat &depth_info):
rgb_mat(rgb_info), depth_mat(depth_info)
{    /*rgb图和深度信息图尺寸必须相等*/
    if(rgb_mat.rows != depth_mat.rows || rgb_mat.cols != depth_mat.cols)
    {
        cout << "size of rgb is " << rgb_mat.rows << ", " << rgb_mat.cols << endl;
        cout << "size of disp is " << depth_mat.rows << ", " << depth_mat.cols << endl;
        throw exception();
    }
    else
    {}
}

Mat DImage::rgb_info() const
{
    return rgb_mat;
}

Mat DImage::depth_info() const
{
    return depth_mat;
}

Frame::Frame(const DImage &dimage, const Mat &description, const vector<KeyPoint> &keypoints): 
d_image(dimage), desp(description), key_points_list(keypoints)
{}

Mat Frame::depth_info() const
{
    return d_image.depth_info();
}

Mat Frame::rgb_info() const
{
    return d_image.rgb_info();
}

Mat Frame::desp_info() const
{
    return desp;
}

DImage Frame::get_dimage() const
{
    return d_image;
}

vector<KeyPoint> Frame::key_points() const
{
    return key_points_list;
}

Param_reader::Param_reader(const string &fp): file_path(fp)
{
    ifstream fin(file_path.c_str());
    if (!fin)
    {
        cout << "Param: parament file does not exist" << endl;
        return;
    }
    else
    {}

    while (!fin.eof())
    {
        string str;
        getline(fin, str);

        if (str[0] == '#' || str[0] == '[') continue;
        else
        {}

        int pos = str.find('=');
        if(pos == -1) continue;
        else
        {}

        string key = str.substr(0, pos);
        string value = str.substr(pos + 1, str.length());
        params[key] = value;

        if (!fin.good()) break;
        else
        {}
    }
}

int Param_reader::get_int(const string &key) const
{
    stringstream ss;
    map<string, string>::const_iterator it = params.find(key);
    if (it == params.end())
    {
        cout << "Params: can not find " << key << " in paraments file" << endl;
        throw exception();
    }
    ss << it->second;
    int i;
    ss >> i;
    return i;
}

double Param_reader::get_double(const string &key) const
{
    stringstream ss;
    map<string, string>::const_iterator it = params.find(key);
    if (it == params.end())
    {
        cout << "Params: can not find " << key << " in paraments file" << endl;
        throw exception();
    }
    ss << it->second;
    double d;
    ss >> d;
    return d;
}

string Param_reader::get_string(const string &key) const
{
    map<string, string>::const_iterator it = params.find(key);
    if (it == params.end())
    {
        cout << "Params: can not find " << key << " in paraments file" << endl;
        throw exception();
    }
    return it->second;
}

bool Param_reader::get_bool(const string &key) const
{
    map<string, string>::const_iterator it = params.find(key);
    if (it == params.end())
    {
        cout << "Params: can not find " << key << " in paraments file" << endl;
        throw exception();
    }
    if(it->second == "true")
        return true;
    else if(it->second == "false")
        return false;
    else
    {
        cout << "Params: " << key << "in paraments file is not a bool value" << endl;
        throw exception();
    }
}

Transform_mat::Transform_mat(): empty(true)
{}

Transform_mat::Transform_mat(const Mat &r, const Mat &t, const int &i):
rvec(r), tvec(t), inliers(i), empty(false)
{
    T = Eigen::Isometry3d::Identity();
    cvmat_to_eigen();
}

Transform_mat Transform_mat::Empty_Transform_mat()
{
    return Transform_mat();
}

Eigen::Isometry3d Transform_mat::eigen_T() const
{
    return T;
}

bool Transform_mat::is_empty() const
{
    return empty;
}

double Transform_mat::get_norm() const
{
    return fabs(min(cv::norm(rvec), 2*M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}

int Transform_mat::get_inliers() const
{
    return inliers;
}

void Transform_mat::cvmat_to_eigen()
{
    /*rvec to eigen_R*/
    Mat cv_R;
    Rodrigues(rvec, cv_R);
    Eigen::Matrix3d eigen_R;
    cv2eigen(cv_R, eigen_R);

    /*eigen_R and tvec to T*/
    Eigen::AngleAxisd angle(eigen_R);
    //Eigen::Translation<double, 3> trans(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2));//???
    T = angle;
    T(0, 3) = tvec.at<double>(0, 0);
    T(1, 3) = tvec.at<double>(0, 1);
    T(2, 3) = tvec.at<double>(0, 2);
}
