#include "VisualOdometry.h"

VO::VO(Feature_motion* const p_detector, Easy_Loop_check* const p_loop_check):
vo_detector(p_detector), vo_loop_check(p_loop_check)
{
    if (vo_detector == 0 || vo_loop_check == 0)
    {
        cout << "VO::VO: input null ptr" << endl;
        throw exception();
    }
    else {}
}

void VO::process(const DImage &dimage)
{
    current_frame = vo_detector->detect_features(dimage);

    /*判断是否为第一帧*/
    if(first_frame_flag == true)
    {
        first_frame_flag = false;
        key_frames.push_back(current_frame);
        last_frame = current_frame;
        cout << endl;
        return;
    }
    else
    {
        bool is_key_frame = vo_loop_check->check_key_frame(current_frame, last_frame);
        
        if(is_key_frame)
        {
            vo_loop_check->check_nearby_loops(current_frame, key_frames);
            vo_loop_check->check_random_loops(current_frame, key_frames);
            key_frames.push_back(current_frame);
        }
        else return;
        
        /*处理完成，新旧帧交换*/
        last_frame = current_frame;
        cout << endl;
    }
}