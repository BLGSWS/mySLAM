#include "Optimizer.h"

MyOptimizer::MyOptimizer()
{
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);//矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(false);
}

void MyOptimizer::add_vertex(const int &vertex_id)
{
    /*建立点*/
    g2o::VertexSE3 *vertex = new g2o::VertexSE3();
    vertex->setId(vertex_id);
    vertex->setEstimate(Eigen::Isometry3d::Identity());
    /*固定第一个点*/
    if(globalOptimizer.vertices().size() == 0)
        vertex->setFixed(true);
    else
    {}
    /*加入点*/
    globalOptimizer.addVertex(vertex);
}

void MyOptimizer::add_edge(const int &current_vertex_id, const int &last_vertex_id, Eigen::Isometry3d &T)
{
    /*建立边*/
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->vertices()[0] = globalOptimizer.vertex(last_vertex_id);
    edge->vertices()[1] = globalOptimizer.vertex(current_vertex_id);
    /*信息矩阵*/
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    edge->setInformation(information);
    /*变换矩阵*/
    edge->setMeasurement(T);
    /*加入边*/
    globalOptimizer.addEdge(edge);
}

void MyOptimizer::optimize()
{
    cout << "optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);
    cout<< "Optimization done." <<endl;
}

void MyOptimizer::save_result(const string &file_path) const
{
    globalOptimizer.save(file_path.c_str());
    cout << "result has been saved in " << file_path << endl;
}