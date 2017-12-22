# pragma once 
/*opencv*/
#include <opencv2/highgui/highgui.hpp> 

/*视差检测模块*/
#include <opencv/cvaux.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Tools.h"

using namespace cv;

class Depth_pretreat
{
public:
    Depth_pretreat()
    {}
    DImage pretreat(const Mat &rgb, const Mat &depth);
};

class Disp_pretreat
{
public:
    //本质上，构造函数只是负责数据成员的
    virtual DImage pretreat(const Mat &left_rgb, const Mat &right_rgb) = 0;
    virtual ~Disp_pretreat() = 0;
};

class BM_Disp_pretreat: public Disp_pretreat
{
public:
    BM_Disp_pretreat(int sadwin_size = SADWIN_SIZE, int broder = BORDER, 
    int max_disp_num = MAX_DISP_NUM, int unique_ratio = UNIQUE_RATIO);
    DImage pretreat(const Mat &left_rgb, const Mat &right_rgb);
private:
    int bm_sadwin_size;
    int bm_broder;
    int bm_max_disp_num;
    int bm_unique_ratio;
};

class GC_Disp_pretreat: public Disp_pretreat
{
public:
    GC_Disp_pretreat();
    DImage pretreat(const Mat &left_rgb, const Mat &right_rgb);
};