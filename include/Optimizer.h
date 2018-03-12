# pragma once

#include <vector>
using namespace std;

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "Tools.h"
#include "Motion.h"

typedef g2o::BlockSolver<g2o::BlockSolver_6_3> SlamBlockSolver;//6*3的块求解器
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class MyOptimizer
{
public:
    MyOptimizer();
    /**
     * 增加一条边，增加边的过程中会自动生成一个点，并将新点与旧点连接；会自动固定第一个点
     * @form_vertex_id: 边起点编号，自动生成一个该编号的点
     * @to_vertex_id: 边终点编号
     * @trans_mat: 位姿变换矩阵
     * */
    void add_edge(const uint32 &form_vertex_id, const uint32 &to_vertex_id, const Transform_mat &trans_mat);

    /**
     * 增加一条回环边。由于是连接两个已存在的边，所以不会生成新的点
     * @form_vertex_id: 边起点编号
     * @to_vertex_id: 边终点编号
     * @trans_mat: 位姿变换矩阵
     * */
    void add_loop_edge(const uint32 &form_vertex_id, const uint32 &to_vertex_id, const Transform_mat &trans_mat);

    /**
     * 优化
     * */
    void optimize();

    /**
     * 储存优化结果
     * @file_path: 文件路径，须为.g2o文件
     * */
    void save_result(const string &file_path) const;

    /**
     * 优化后，获取编号为id帧的位姿变换矩阵
     * @id：帧编号
     * @return: eigen储存的变换矩阵
     * */
    Eigen::Isometry3d get_pose(const size_t &id) const;

    ~MyOptimizer()
    {
        globalOptimizer.clear();
    }
private:
    g2o::RobustKernel* robustKernel;
    g2o::SparseOptimizer globalOptimizer;
};

class Easy_Loop_check
{
public:
    Easy_Loop_check(Feature_motion* const p_motion, MyOptimizer* const p_optimizer, 
    const int &nearby_check_num = NEARBY_CHECK_NUM, const int &random_check_num = RANDOM_CHECK_NUM, 
    const int &min_inliers = MIN_INLIERS, const int &max_norm = MAX_NORM, const int &min_norm = MIN_NORM);

    /**
     * 对比两个相邻帧，确定是否保留为关键帧
     * @current_frame: 相邻帧的前一帧
     * @last_frame: 相邻帧的后一帧
     * @return: 是否为关键帧
    */
    bool check_key_frame(const Frame &current_frame, const Frame &last_frame);

    /**
     * 检测附近回环
     * @frame: 待检测帧
     * @key_frames: 储存关键帧的vector
     * */
    void check_nearby_loops(const Frame &frame, const vector<Frame> &key_frames);

    /**
     * 随机检测回环
     * @frame: 待检测帧
     * @key_frames: 储存关键帧的vector
     * */
    void check_random_loops(const Frame &frame, const vector<Frame> &key_frames);
protected:
    /**
     * 输入两个帧，获取相对位姿变化 
     * @return: 变换矩阵
    */
    Transform_mat get_motion(const Frame &frame1, const Frame &frame2);
private:
    MyOptimizer *lc_p_optimizer;
    Feature_motion *lc_p_motion;
    /*参数*/
    int lc_nearby_check_num;//最大检测数
    int lc_random_check_num;
    int lc_min_inliers;
    int lc_max_norm;//最大移动距离
    int lc_min_norm;//最小移动距离
};