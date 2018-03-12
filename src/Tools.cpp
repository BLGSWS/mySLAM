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

Frame::Frame(const DImage &dimage, const Mat &description, const vector<KeyPoint> &keypoints, const uint32 &i): 
d_image(dimage), desp(description), key_points_list(keypoints), id(i)
{}

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

const int Param_reader::get_int(const string &key) const
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

const uint32 Param_reader::get_uint(const string &key) const
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
    if(i < 0)
    {
        cout << "Warning: Param_reader::get_uint: value is less than 0, need unsigned int type" << endl;
    }
    return i;
}

const double Param_reader::get_double(const string &key) const
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

const string Param_reader::get_string(const string &key) const
{
    map<string, string>::const_iterator it = params.find(key);
    if (it == params.end())
    {
        cout << "Params: can not find " << key << " in paraments file" << endl;
        throw exception();
    }
    return it->second;
}

const bool Param_reader::get_bool(const string &key) const
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

Transform_mat::Transform_mat(): empty_flag(true)
{}

Transform_mat::Transform_mat(const Mat &r, const Mat &t, const int &i):
rvec(r), tvec(t), inliers(i), empty_flag(false)
{
    T = Eigen::Isometry3d::Identity();
    cvmat_to_eigen();
}

Transform_mat Transform_mat::Empty_Transform_mat()
{
    return Transform_mat();
}

double Transform_mat::get_norm() const
{
    return fabs(min(cv::norm(rvec), 2*M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
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
