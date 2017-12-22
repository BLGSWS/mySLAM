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

void MyOptimizer::add_edge(const unsigned int &vertex_id, const Transform_mat &trans_mat)
{
    /*建立点*/
    g2o::VertexSE3 *vertex = new g2o::VertexSE3();
    vertex->setId(vertex_id);
    vertex->setEstimate(Eigen::Isometry3d::Identity());
    current_index = vertex_id;
    if(globalOptimizer.vertices().size() == 0)
    {
        /*固定第一个点*/
        vertex->setFixed(true);
        /*加入点*/
        globalOptimizer.addVertex(vertex);
        /*处理完成*/
        last_index = current_index;
    }
    else
    {
        /*如果未解出位姿变化，放弃此节点*/
        if(trans_mat.is_empty()) return;
        else
        {
            /*加入节点*/
            globalOptimizer.addVertex(vertex);
            /*建立边*/
            g2o::EdgeSE3* edge = new g2o::EdgeSE3();
            edge->vertices()[0] = globalOptimizer.vertex(last_index);
            edge->vertices()[1] = globalOptimizer.vertex(current_index);
            /*信息矩阵*/
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6, 6>::Identity();
            information = information * 100;
            edge->setInformation(information);
            /*变换矩阵*/
            edge->setMeasurement(trans_mat.eigen_T());
            /*加入边*/
            globalOptimizer.addEdge(edge);
            /*处理完成*/
            last_index = current_index;
        }
    }
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