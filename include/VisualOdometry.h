#pragma once

#include <iostream>
#include <string>
using namespace std;

#include "Motion.h"
#include "Optimizer.h"
//#include "Map.h"

class VO
{
public:
    VO(Feature_motion* const p_detector, Easy_Loop_check* const p_loop_check);
    /**
     * 处理带深度信息的图片序列，选择关键帧
     * @dimage: 带深度信息的图片
    */
    void process(const DImage &dimage);
    /**
     * 私有数据成员接口
     * */
    const vector<Frame>& get_key_frames() const { return key_frames; }
private:
    VO();
    Feature_motion* vo_detector;
    Easy_Loop_check* vo_loop_check;
    /*管理帧*/
    Frame current_frame, last_frame;
    /*是否为第一帧*/
    bool first_frame_flag;
    vector<Frame> key_frames;
};