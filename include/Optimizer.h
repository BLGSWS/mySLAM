# pragma once

#include <iostream>
#include <memory>
#include <string>
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

typedef g2o::BlockSolver<g2o::BlockSolver_6_3> SlamBlockSolver;//6*3的块求解器
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class MyOptimizer
{
public:
    MyOptimizer();
    void add_edge(const unsigned int &vertex_id, const Transform_mat &trans_mat);
    void add_loop_edge(const unsigned int &current_vertex_id, const unsigned int &last_vertex_id);
    void optimize();
    void save_result(const string &file_path) const;
    ~MyOptimizer()
    {
        globalOptimizer.clear();
    }
private:
    g2o::SparseOptimizer globalOptimizer;
    int last_index;
    int current_index;
};