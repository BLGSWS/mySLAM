#pragma once
#include <string>
using namespace std;

/*抽象相机类，储存内参，提供2d点到3d点转换方法*/
class MyCamera
{
public:
    MyCamera(const double &fa, const double &cx, 
             const double &cy, const double &fx, 
             const double &fy): 
    Factor(fa), Cx(cx), Cy(cy), Fx(fx), Fy(fy)
    {}
    /*2d点到3d点转换方法，用派生类统一接口*/
    template<class P>
    P point2dto3d(P point)
    {
        P new_point;
        new_point.z = get_z(point.z);
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
protected:
    //模板函数竟然不能声明为虚函数
    //虚函数不光要声明还要给实现
    virtual double get_z(const double &z) = 0;
private:
    MyCamera();
};

/*双目相机类*/
class DispCamera: public MyCamera
{
public:
    DispCamera(const double &fa, const double &cx, 
             const double &cy, const double &fx, 
             const double &fy, const double &tx): 
    MyCamera(fa, cx, cy, fx, fy), Tx(tx)
    {}
    /*双目相机多了一个目距参数*/
    double Tx;
protected:
    double get_z(const double &z)
    {
        if(z == 0)
            return 0.0;
        else
            return Factor * Tx / z;
    }
private:
    DispCamera();
};

/*深度相机类*/
class DepthCamera: public MyCamera
{
public:
    DepthCamera(const double &fa, const double &cx, 
             const double &cy, const double &fx, 
             const double &fy, const double &tx): 
    MyCamera(fa, cx, cy, fx, fy)
    {}
protected:
    double get_z(const double &z)
    {
        return z / Factor;
    }
private:
    DepthCamera();
};