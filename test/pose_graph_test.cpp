
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//Sophus
#include "sophus/se3.h"

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//G2O
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

//Std
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>

#include <iostream>

using namespace std;

int main(){

	//Construct the optimizer.
    //Construct the optimizer.
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> DirectBlock;  // 求解的向量是6＊1的
    typedef g2o::BlockSolverX DirectBlock;
    std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ());
    // std::unique_ptr<DirectBlock::LinearSolverType> pLinearSolver (new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());
    std::unique_ptr<DirectBlock> pSolverPtr (new DirectBlock ( std::move(pLinearSolver) ));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* pSolver = new g2o::OptimizationAlgorithmLevenberg ( std::move(pSolverPtr) ); // L-M
    // g2o::OptimizationAlgorithmGaussNewton* pSolver = new g2o::OptimizationAlgorithmGaussNewton ( std::move(pSolverPtr) ); // L-M
	//TODO: These configuration should be adjusted in the future.    
	g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm ( pSolver );
    iOptimizer.setVerbose( true );


    int nVertexId = 3;
    int nVertexSize = 2;

    vector<g2o::VertexSE3 *> gAllVertices;
    vector<Sophus::SE3> gPoses;

    for (int i=0;i<nVertexSize;i++){
    	Eigen::Matrix<double, 6, 1> mPoseMatrix;
    	mPoseMatrix << 1.0 , 1.0 , -1.0, -1.0 , 1.0 , 1.0;
    	mPoseMatrix = mPoseMatrix * (i-0.5)/10;
    	Sophus::SE3 mPose = Sophus::SE3::exp(mPoseMatrix);

    	// cv::RNG iRNG(12345);
    	// Eigen::Matrix<double, 6, 1> mDisturbance;
    	// mDisturbance << iRNG.gaussian(0.1), iRNG.gaussian(0.1), iRNG.gaussian(0.1), iRNG.gaussian(0.1), iRNG.gaussian(0.1), iRNG.gaussian(0.1); 
    	// Sophus::SE3 mDisturbancePose = Sophus::SE3::exp(mDisturbance);
    	// cout << "mDisturbance Pose is: " << endl << mDisturbancePose.matrix() << endl;



	    g2o::VertexSE3 * pPoseVertex = new g2o::VertexSE3();
	    pPoseVertex->setId(nVertexId);
	    if (i==0){
	    	pPoseVertex->setFixed(true);
	    }



	    nVertexId++;
	    pPoseVertex->setEstimate( g2o::SE3Quat((mPose).rotation_matrix(),(mPose ).translation()));

	    iOptimizer.addVertex(pPoseVertex);

	    gAllVertices.push_back(pPoseVertex);
	    gPoses.push_back(mPose);
    }

    vector<g2o::EdgeSE3 *> gEdges;


	const Eigen::Matrix<double,6,6> mInformation = Eigen::Matrix<double,6,6>::Identity();


    int nEdgeIndex = 200;
    for (int i=0;i<nVertexSize;i++){
    	for (int j=i+1;j<nVertexSize;j++){



    		Sophus::SE3 mPose1 = gPoses[i];
    		Sophus::SE3 mPose2 = gPoses[j];

    		// from->estimate().inverse() * to->estimate()
    		Sophus::SE3 mEstimate = ( mPose2 * mPose1.inverse());

    		g2o::EdgeSE3 * pPoseGraphEdge = new g2o::EdgeSE3();
		    pPoseGraphEdge->setMeasurement(g2o::SE3Quat(
		        mEstimate.rotation_matrix(),
		        mEstimate.translation()));

		    pPoseGraphEdge->setId( nEdgeIndex++ );
            //Linked with the position of 3d points.
            pPoseGraphEdge->setVertex(0 , gAllVertices[i]);
            pPoseGraphEdge->setVertex(1 , gAllVertices[j]);
            

            // Eigen::Matrix<double, 6, 6> mInformation = Eigen::Matrix<double, 6, 6>::Identity()*2;

            pPoseGraphEdge->setInformation(mInformation);
            cout << "First: " << endl << pPoseGraphEdge->information() << endl;
            gEdges.push_back(pPoseGraphEdge);
    		iOptimizer.addEdge(pPoseGraphEdge);
    	}
    }
  





    // iOptimizer.initializeOptimization();

    for (auto item : gEdges){
    	cout << "Information1" << endl;
    	// item->LogInformation();
    	cout << "Second: " << endl << item->information() << endl;
    	item->computeError();
    	cout << "Information2" << endl;
    	// item->LogInformation();
    	cout << "Third: " << endl << item->information() << endl;
    }


    // iOptimizer.optimize(100);





	return 0;
}