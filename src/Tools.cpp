#include "Tools.h"
#include <sstream>

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

Result_of_PNP::Result_of_PNP(const Mat &r, const Mat &t, const int &i):
rvec(r), tvec(t), inliers(i), norm(0.0)
{
    T = Eigen::Isometry3d::Identity();
    cvmat_to_eigen();
    norm_of_transform();
}

void Result_of_PNP::cvmat_to_eigen()
{
    /*rvec to eigen_R*/
    Mat cv_R;
    Rodrigues(rvec, cv_R);
    Eigen::Matrix3d eigen_R;
    cv2eigen(cv_R, eigen_R);

    /*eigen_R and tvec to T*/
    Eigen::AngleAxisd angle(eigen_R);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));//???
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);
}

void Result_of_PNP::norm_of_transform()
{
    norm = fabs(min(cv::norm(rvec), 2*M_PI - cv::norm(rvec))) + fabs(cv::norm(tvec));
}
