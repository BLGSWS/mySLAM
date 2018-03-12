#include "Optimizer.h"
#include <algorithm>

MyOptimizer::MyOptimizer()
{
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);//矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(false);
    robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
}

void MyOptimizer::add_edge(const uint32 &form_vertex_id, const uint32 &to_vertex_id, const Transform_mat &trans_mat)
{
    if (trans_mat.is_empty()) return;
    else {}
    /*判断是否为第一条边*/
    if(globalOptimizer.vertices().size() == 0)
    {
        /*如果是第一条边， 先加第一个点*/
        g2o::VertexSE3 *first_vertex = new g2o::VertexSE3();
        first_vertex->setId(to_vertex_id);
        first_vertex->setEstimate(Eigen::Isometry3d::Identity());
        /*固定第一个点*/
        first_vertex->setFixed(true);
        globalOptimizer.addVertex(first_vertex);
    }
    else {}
    /*建立点*/
    g2o::VertexSE3 *vertex = new g2o::VertexSE3();
    vertex->setId(form_vertex_id);
    vertex->setEstimate(Eigen::Isometry3d::Identity());
    globalOptimizer.addVertex(vertex);
    /*建立边*/
    g2o::EdgeSE3* edge = new g2o::EdgeSE3;
    //edge->vertices()[0] = globalOptimizer.vertex(to_vertex_id);
    //edge->vertices()[1] = globalOptimizer.vertex(form_vertex_id);
    edge->setVertex(0, globalOptimizer.vertex(to_vertex_id));
    edge->setVertex(1, globalOptimizer.vertex(form_vertex_id));
    edge->setRobustKernel(robustKernel);
    /*信息矩阵*/
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6, 6>::Identity();
    information = information * 100;
    edge->setInformation(information);
    /*变换矩阵*/
    edge->setMeasurement(trans_mat.eigen_T().inverse());
    /*加入边*/
    globalOptimizer.addEdge(edge);
}

void MyOptimizer::add_loop_edge(const uint32 &form_vertex_id, const uint32 &to_vertex_id, const Transform_mat &trans_mat)
{
    if (trans_mat.is_empty()) return;
    else {}
    /*建立边*/
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->vertices()[0] = globalOptimizer.vertex(to_vertex_id);
    edge->vertices()[1] = globalOptimizer.vertex(form_vertex_id);
    /*信息矩阵*/
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6, 6>::Identity();
    information = information * 100;
    edge->setInformation(information);
    /*变换矩阵*/
    edge->setMeasurement(trans_mat.eigen_T().inverse());
    /*加入边*/
    globalOptimizer.addEdge(edge);
}

void MyOptimizer::optimize()
{
    cout << "MyOptimizer::optimize: optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);
    cout<< "MyOptimizer::optimize: Optimization done" <<endl;
}

void MyOptimizer::save_result(const string &file_path) const
{
    globalOptimizer.save(file_path.c_str());
    cout << "MyOptimizer::save_result: optimization result has been saved in " << file_path << endl;
}

Eigen::Isometry3d MyOptimizer::get_pose(const size_t &id) const
{
    g2o::HyperGraph::VertexIDMap::const_iterator it = globalOptimizer.vertices().find(id);
    if (it == globalOptimizer.vertices().end())
    {
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(it->second);
    Eigen::Isometry3d pose = vertex->estimate();
    return pose;
}

Easy_Loop_check::Easy_Loop_check(Feature_motion* const p_motion, 
MyOptimizer* const p_optimizer, const int &nearby_check_num, const int &random_check_num, 
const int &min_inliers, const int &max_norm, const int &min_norm):
lc_p_motion(p_motion), lc_p_optimizer(p_optimizer), lc_nearby_check_num(nearby_check_num), 
lc_random_check_num(random_check_num), lc_min_inliers(min_inliers), lc_max_norm(max_norm), 
lc_min_norm(min_norm)
{
    if (lc_p_motion == 0 || lc_p_motion == 0)
    {
        cout << "Easy_Loop_check::Easy_Loop_check: parament is null ptr" << endl;
        throw exception();
    }
    else {}
}

Transform_mat Easy_Loop_check::get_motion(const Frame &frame1, const Frame &frame2)
{
    vector<DMatch> matches = lc_p_motion->match_features(frame2, frame1);
    /*if(matches.size() < vo_min_matches)
    {
        cout << "VO::process_frame: matched feature is too less, give up this frame" << endl << endl;
        return false;
    }
    else {}*/

    /*pnp求解*/
    Transform_mat trans_mat = lc_p_motion->estimate_motion(frame2, frame1, matches);

    /*判断inliers是否太小*/
    if (trans_mat.get_inliers() <= lc_min_inliers)
    {
        cout << "inliers = " << trans_mat.get_inliers() << endl;
        cout << "Easy_Loop_check::process_frame: inliers is too small, give up this frame" << endl << endl;
        return Transform_mat::Empty_Transform_mat();
    }
    else {}

    if (trans_mat.get_norm() > lc_max_norm || trans_mat.get_norm() < lc_min_norm)
    {
        cout << "norm = " << trans_mat.get_norm() << endl;
        cout << "Easy_Loop_check::check_key_frame: norm is out of range, give up this frame" << endl << endl;
        return Transform_mat::Empty_Transform_mat();
    }
    else {}

    /*返回变换矩阵*/
    return trans_mat;
}

bool Easy_Loop_check::check_key_frame(const Frame &current_frame, const Frame &last_frame)
{
    Transform_mat trans_mat = get_motion(current_frame, last_frame);
    if (!trans_mat.is_empty())
    {
        /*打印位置信息*/
        cout << "T = " << trans_mat.eigen_T().matrix() << endl;
        cout << "add vertex: " << current_frame.get_id() << endl;
        cout << "add edge: (" << current_frame.get_id() << "," << last_frame.get_id() << ")" << endl;
        lc_p_optimizer->add_edge(current_frame.get_id(), last_frame.get_id(), trans_mat);
        return true;
    }
    else { return false; }
}

void Easy_Loop_check::check_nearby_loops(const Frame &frame, const vector<Frame> &key_frames)
{
    /*检测数量*/
    size_t num = lc_nearby_check_num > key_frames.size() ? key_frames.size(): lc_nearby_check_num;
    for (size_t i = 0; i < num; i++)
    {
        /*获取变换矩阵*/
        Transform_mat trans_mat = get_motion(frame, key_frames[i]);
        /*如果检测到与某帧存在回环*/
        if(!trans_mat.is_empty())
        {
            lc_p_optimizer->add_loop_edge(frame.get_id(), key_frames[i].get_id(), trans_mat);
            cout << "add loop edge: (" << frame.get_id() << "," << key_frames[i].get_id() << ")" << endl;
        }
        else {}
    }
}

void Easy_Loop_check::check_random_loops(const Frame &frame, const vector<Frame> &key_frames)
{
    srand((uint32)time(NULL));
    if(key_frames.size() < lc_random_check_num)
    {
        for (size_t i = 0; i < key_frames.size(); i++)
        {
            /*获取变换矩阵*/
            Transform_mat trans_mat = get_motion(frame, key_frames[i]);
            /*如果检测到与某帧存在回环*/
            if(!trans_mat.is_empty())
            {
                lc_p_optimizer->add_loop_edge(frame.get_id(), key_frames[i].get_id(), trans_mat);
                cout << "add loop edge: (" << frame.get_id() << "," << key_frames[i].get_id() << ")" << endl;
            }
            else {}
        }
    }
    else
    {
        set<int> checked;
        for (size_t i = 0; i < lc_random_check_num; i++)
        {
            int index = rand() % key_frames.size();
            if (checked.count(index) != 0) { continue; }
            else {}
            /*获取变换矩阵*/
            Transform_mat trans_mat = get_motion(frame, key_frames[index]);
            /*如果检测到与某帧存在回环*/
            if (!trans_mat.is_empty())
            {
                lc_p_optimizer->add_loop_edge(frame.get_id(), key_frames[index].get_id(), trans_mat);
                cout << "add loop edge: (" << frame.get_id() << "," << key_frames[index].get_id() << ")" << endl;
            }
            else {}
            checked.insert(index);
        }
    }

}