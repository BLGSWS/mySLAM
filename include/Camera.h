#pragma once
#include <string>
using namespace std;

class MyCamera
{
public:
    MyCamera(const double &fa, const double &cx, 
             const double &cy, const double &fx, 
             const double &fy, const double &tx): 
    Factor(fa), Cx(cx), Cy(cy), Fx(fx), Fy(fy), Tx(tx)
    {}
    static void read_by_profile(const string &file_path);
    template<class P>
    P point2dto3d_by_disp(P point)
    {
        P new_point;
        new_point.z = Factor * Tx / point.z;
        new_point.x = (point.x - Cx) * new_point.z / Fx;
        new_point.y = (point.y - Cy) * new_point.z / Fy;
        return new_point;
    }
    template<class P>
    P point2dto3d_by_depth(P point)
    {
        P new_point;
        new_point.z = double(point.z) / Factor;
        new_point.x = (point.x - Cx) * new_point.z / Fx;
        new_point.y = (point.y - Cy) * new_point.z / Fy;
        return new_point;
    }
    /*焦距*/
    double Factor;
    /*平移*/
    double Cx, Cy;
    /*缩放系数*/
    double Fx, Fy;
    /*目距*/
    double Tx;
private:
    MyCamera();
};