#include "../../include/optimizer/optimizer.h"
#include <algorithm>
using namespace std;


SingleMapOptimizer::SingleMapOptimizer(Map * pMap){
	this->m_pMap = pMap;
}









void SingleMapOptimizer::Optimize(int nIterations){
	//Generate the optimizer.
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton * solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm(solver);
    //If output the debug info.
    iOptimizer.setVerbose(true);




    //Generate All MapPoints

    vector<MapPoint *> gMapPoints = this->m_pMap->GetMapPoints();
    map<MapPoint *, g2o::VertexSBAPointXYZ *> dMapPointsAndVertices;
    int nMapPointIndex = 3;




    for (MapPoint * pMapPoint : gMapPoints){
    	nMapPointIndex++;
    	cv::Point3d iPoint3D = pMapPoint->GetPosition();
    	g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
    	pPointVertex->setId(nMapPointIndex);
    	dMapPointsAndVertices[pMapPoint] = pPointVertex;
    	// pPoint->setFixed(true);
   	 	pPointVertex->setEstimate(Eigen::Vector3d(iPoint3D.x ,
                                            iPoint3D.y ,
                                          	iPoint3D.z));
   	 	pPointVertex->setMarginalized(true);
   	 	iOptimizer.addVertex(pPointVertex);
    }

    int nKeyFrameIndex = nMapPointIndex + 10;
    vector<KeyFrame *> gKeyFrames = this->m_pMap->GetKeyFrames();


    map<KeyFrame *, g2o::VertexSE3Expmap *> dKeyFramesAndVertices;
    for (KeyFrame * pKeyFrame : gKeyFrames){
    	nKeyFrameIndex ++;

    	//Add the pose vertex of this keyframe

    	Eigen::MatrixXd mPose = pKeyFrame->GetPose();
    	Eigen::Matrix3d mRotation = mPose.block(0 , 0 , 3 , 3);
    	Eigen::Vector3d mTranslation = mPose.block(0 , 3 , 3 , 1);
 		g2o::VertexSE3Expmap * pPoseVertex = new g2o::VertexSE3Expmap();
 		dKeyFramesAndVertices[pKeyFrame] = pPoseVertex;
 		pPoseVertex->setId(nKeyFrameIndex);
    	pPoseVertex->setEstimate( g2o::SE3Quat(
        		mRotation,
        		mTranslation));
    	iOptimizer.addVertex(pPoseVertex);

    	nKeyFrameIndex ++;
    	int nCameraParameterId = nKeyFrameIndex;

    	//Add the camera parameter vertex
    	cv::Mat mK = pKeyFrame->GetCamera()->GetK();
	    g2o::CameraParameters * pCameraParameter = new g2o::CameraParameters(
	        mK.at<double>(0 , 0) , Eigen::Vector2d(mK.at<double>(0 , 2) , mK.at<double>(1 , 2)) , 0
	        );
	    pCameraParameter->setId(nCameraParameterId);
	    //Add parameters to the optimizer.
	    iOptimizer.addParameter(pCameraParameter);


    	//Add reprojection error edges.


		map<MapPoint *, cv::Point2d> dMapPointsAndObservations = pKeyFrame->GetAllObservations();
		for (auto iPair : dMapPointsAndObservations){
			nKeyFrameIndex ++;	
			MapPoint * pMapPoint = iPair.first;
			cv::Point2d iObservation = iPair.second;
	    	g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
	        pEdge->setId( nKeyFrameIndex );
	        //Linked with the position of 3d points.
			pEdge->setVertex(0 , dMapPointsAndVertices[pMapPoint]);
			pEdge->setVertex(1 , pPoseVertex);
			//Set measurement and camera parameters.
			pEdge->setMeasurement(Eigen::Vector2d(
									iObservation.x, 
									iObservation.y));
			pEdge->setParameterId(0 , nCameraParameterId);
			pEdge->setInformation(Eigen::Matrix2d::Identity());
			
			iOptimizer.addEdge(pEdge);

		}
		// break;
    }


    //Initialize optimization
    iOptimizer.initializeOptimization();
    iOptimizer.optimize(nIterations);

	//Load results;

	for (MapPoint * pMapPoint : gMapPoints){
		Eigen::Vector3d mPosition = dMapPointsAndVertices[pMapPoint]->estimate();
		pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
	}

	for (KeyFrame * pKeyFrame : gKeyFrames){
		Eigen::Matrix4d mUpdatedPose =  Eigen::Isometry3d(dKeyFramesAndVertices[pKeyFrame]->estimate()).matrix();
		pKeyFrame->SetPose(mUpdatedPose);
	}


}






void SingleMapOptimizer::OptimizeCommonRegion(int nIterations){
    //Generate the optimizer.
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton * solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm(solver);
    //If output the debug info.
    iOptimizer.setVerbose(true);


    set<MapPoint *> sCommonMapPoints = this->m_pMap->GetCommonMapPoints();
    set<KeyFrame *> sCommonKeyFrames = this->m_pMap->GetCommonKeyFrames();
    cout << "Common MapPoints size: " << sCommonMapPoints.size() << endl;
    cout << "Common KeyFrames size: " << sCommonKeyFrames.size() << endl;



    //Generate All MapPoints

    vector<MapPoint *> gMapPoints = this->m_pMap->GetMapPoints();
    map<MapPoint *, g2o::VertexSBAPointXYZ *> dMapPointsAndVertices;
    int nMapPointIndex = 3;




    for (MapPoint * pMapPoint : gMapPoints){
        nMapPointIndex++;
        cv::Point3d iPoint3D = pMapPoint->GetPosition();
        g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
        pPointVertex->setId(nMapPointIndex);
        dMapPointsAndVertices[pMapPoint] = pPointVertex;
        // pPoint->setFixed(true);
        pPointVertex->setEstimate(Eigen::Vector3d(iPoint3D.x ,
                                            iPoint3D.y ,
                                            iPoint3D.z));
        pPointVertex->setMarginalized(true);

        if (sCommonMapPoints.find(pMapPoint)!= sCommonMapPoints.end()){
            pPointVertex->setFixed(true);
        }

        iOptimizer.addVertex(pPointVertex);


    }

    int nKeyFrameIndex = nMapPointIndex + 10;
    vector<KeyFrame *> gKeyFrames = this->m_pMap->GetKeyFrames();


    map<KeyFrame *, g2o::VertexSE3Expmap *> dKeyFramesAndVertices;
    for (KeyFrame * pKeyFrame : gKeyFrames){
        nKeyFrameIndex ++;

        //Add the pose vertex of this keyframe

        Eigen::MatrixXd mPose = pKeyFrame->GetPose();
        Eigen::Matrix3d mRotation = mPose.block(0 , 0 , 3 , 3);
        Eigen::Vector3d mTranslation = mPose.block(0 , 3 , 3 , 1);
        g2o::VertexSE3Expmap * pPoseVertex = new g2o::VertexSE3Expmap();
        dKeyFramesAndVertices[pKeyFrame] = pPoseVertex;
        pPoseVertex->setId(nKeyFrameIndex);
        pPoseVertex->setEstimate( g2o::SE3Quat(
                mRotation,
                mTranslation));


        if (sCommonKeyFrames.find(pKeyFrame)!= sCommonKeyFrames.end()){
            pPoseVertex->setFixed(true);
        }


        iOptimizer.addVertex(pPoseVertex);

        nKeyFrameIndex ++;
        int nCameraParameterId = nKeyFrameIndex;

        //Add the camera parameter vertex
        cv::Mat mK = pKeyFrame->GetCamera()->GetK();
        g2o::CameraParameters * pCameraParameter = new g2o::CameraParameters(
            mK.at<double>(0 , 0) , Eigen::Vector2d(mK.at<double>(0 , 2) , mK.at<double>(1 , 2)) , 0
            );
        pCameraParameter->setId(nCameraParameterId);
        //Add parameters to the optimizer.
        iOptimizer.addParameter(pCameraParameter);


        //Add reprojection error edges.


        map<MapPoint *, cv::Point2d> dMapPointsAndObservations = pKeyFrame->GetAllObservations();
        for (auto iPair : dMapPointsAndObservations){
            nKeyFrameIndex ++;  
            MapPoint * pMapPoint = iPair.first;
            cv::Point2d iObservation = iPair.second;
            g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
            pEdge->setId( nKeyFrameIndex );
            //Linked with the position of 3d points.
            pEdge->setVertex(0 , dMapPointsAndVertices[pMapPoint]);
            pEdge->setVertex(1 , pPoseVertex);
            //Set measurement and camera parameters.
            pEdge->setMeasurement(Eigen::Vector2d(
                                    iObservation.x, 
                                    iObservation.y));
            pEdge->setParameterId(0 , nCameraParameterId);
            pEdge->setInformation(Eigen::Matrix2d::Identity());
            
            iOptimizer.addEdge(pEdge);

        }
        // break;
    }


    //Initialize optimization
    iOptimizer.initializeOptimization();
    iOptimizer.optimize(nIterations);

    //Load results;

    for (MapPoint * pMapPoint : gMapPoints){
        Eigen::Vector3d mPosition = dMapPointsAndVertices[pMapPoint]->estimate();
        pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
    }

    for (KeyFrame * pKeyFrame : gKeyFrames){
        Eigen::Matrix4d mUpdatedPose =  Eigen::Isometry3d(dKeyFramesAndVertices[pKeyFrame]->estimate()).matrix();
        pKeyFrame->SetPose(mUpdatedPose);
    }


}




void SingleMapOptimizer::OptimizeCascade(int nIterations){
    //Generate the optimizer.
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton * solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm(solver);
    //If output the debug info.
    iOptimizer.setVerbose(true);

    set<MapPoint *> sAllMapPoints = this->m_pMap->GetMapPointsSet();
    set<KeyFrame *> sAllKeyFrames = this->m_pMap->GetKeyFramesSet();

    int nLastKeyFrameSize = 0;





    set<MapPoint *> sCommonMapPoints = this->m_pMap->GetCommonMapPoints();
    set<KeyFrame *> sCommonKeyFrames = this->m_pMap->GetCommonKeyFrames();
    cout << "Common MapPoints size: " << sCommonMapPoints.size() << endl;
    cout << "Common KeyFrames size: " << sCommonKeyFrames.size() << endl;




    //Generate All MapPoints

    vector<MapPoint *> gMapPoints = this->m_pMap->GetMapPoints();
    map<MapPoint *, g2o::VertexSBAPointXYZ *> dMapPointsAndVertices;
    int nMapPointIndex = 3;




    for (MapPoint * pMapPoint : gMapPoints){
        nMapPointIndex++;
        cv::Point3d iPoint3D = pMapPoint->GetPosition();
        g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
        pPointVertex->setId(nMapPointIndex);
        dMapPointsAndVertices[pMapPoint] = pPointVertex;
        // pPoint->setFixed(true);
        pPointVertex->setEstimate(Eigen::Vector3d(iPoint3D.x ,
                                            iPoint3D.y ,
                                            iPoint3D.z));
        pPointVertex->setMarginalized(true);
        pPointVertex->setFixed(true);

        iOptimizer.addVertex(pPointVertex);


    }

    int nKeyFrameIndex = nMapPointIndex + 10;
    vector<KeyFrame *> gKeyFrames = this->m_pMap->GetKeyFrames();


    map<KeyFrame *, g2o::VertexSE3Expmap *> dKeyFramesAndVertices;
    for (KeyFrame * pKeyFrame : gKeyFrames){
        nKeyFrameIndex ++;

        //Add the pose vertex of this keyframe

        Eigen::MatrixXd mPose = pKeyFrame->GetPose();
        Eigen::Matrix3d mRotation = mPose.block(0 , 0 , 3 , 3);
        Eigen::Vector3d mTranslation = mPose.block(0 , 3 , 3 , 1);
        g2o::VertexSE3Expmap * pPoseVertex = new g2o::VertexSE3Expmap();
        dKeyFramesAndVertices[pKeyFrame] = pPoseVertex;
        pPoseVertex->setId(nKeyFrameIndex);
        pPoseVertex->setEstimate( g2o::SE3Quat(
                mRotation,
                mTranslation));
        pPoseVertex->setFixed(true);

        iOptimizer.addVertex(pPoseVertex);

        nKeyFrameIndex ++;
        int nCameraParameterId = nKeyFrameIndex;

        //Add the camera parameter vertex
        cv::Mat mK = pKeyFrame->GetCamera()->GetK();
        g2o::CameraParameters * pCameraParameter = new g2o::CameraParameters(
            mK.at<double>(0 , 0) , Eigen::Vector2d(mK.at<double>(0 , 2) , mK.at<double>(1 , 2)) , 0
            );
        pCameraParameter->setId(nCameraParameterId);
        //Add parameters to the optimizer.
        iOptimizer.addParameter(pCameraParameter);


        //Add reprojection error edges.


        map<MapPoint *, cv::Point2d> dMapPointsAndObservations = pKeyFrame->GetAllObservations();
        for (auto iPair : dMapPointsAndObservations){
            nKeyFrameIndex ++;  
            MapPoint * pMapPoint = iPair.first;
            cv::Point2d iObservation = iPair.second;
            g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
            pEdge->setId( nKeyFrameIndex );
            //Linked with the position of 3d points.
            pEdge->setVertex(0 , dMapPointsAndVertices[pMapPoint]);
            pEdge->setVertex(1 , pPoseVertex);
            //Set measurement and camera parameters.
            pEdge->setMeasurement(Eigen::Vector2d(
                                    iObservation.x, 
                                    iObservation.y));
            pEdge->setParameterId(0 , nCameraParameterId);
            pEdge->setInformation(Eigen::Matrix2d::Identity());
            
            iOptimizer.addEdge(pEdge);

        }
        // break;
    }





    set<KeyFrame *> sOptimizedKeyFrames;
    set<MapPoint *> sOptimizedMapPoints;

    sOptimizedKeyFrames.clear();
    sOptimizedMapPoints.clear();
    iOptimizer.setVerbose(false);

    while (nLastKeyFrameSize < sCommonKeyFrames.size() && sCommonKeyFrames.size() > sOptimizedKeyFrames.size()){
        for (KeyFrame * pKeyFrame : sCommonKeyFrames){
            dKeyFramesAndVertices[pKeyFrame]->setFixed(false);
        
        }

        for (MapPoint * pMapPoint : sCommonMapPoints){
            dMapPointsAndVertices[pMapPoint]->setFixed(false);
        }

        for (KeyFrame * pKeyFrame : sOptimizedKeyFrames){
            dKeyFramesAndVertices[pKeyFrame]->setFixed(true);
        
        }

        for (MapPoint * pMapPoint : sOptimizedMapPoints){
            dMapPointsAndVertices[pMapPoint]->setFixed(true);
        }


        //Initialize optimization
        iOptimizer.initializeOptimization();

        iOptimizer.optimize(nIterations);

        for (KeyFrame * pKeyFrame : sCommonKeyFrames){
            sOptimizedKeyFrames.insert(pKeyFrame);
        }

        for (MapPoint * pMapPoint : sCommonMapPoints){
            sOptimizedMapPoints.insert(pMapPoint);
        }


        for (KeyFrame * pKeyFrame : sOptimizedKeyFrames){
            vector<KeyFrame *> gCovisibleKeyFrames = pKeyFrame->GetCovisibleKeyFrames();
            for (KeyFrame * pKeyFrame : gCovisibleKeyFrames){
                sCommonKeyFrames.insert(pKeyFrame);
            }
        }

        for (KeyFrame * pKeyFrame : sCommonKeyFrames){
            vector<MapPoint *> gObservedMapPoints = pKeyFrame->GetMapPoints();
            for (MapPoint * pMapPoint : gObservedMapPoints){
                sCommonMapPoints.insert(pMapPoint);
            }
        }

    }

    

    //Load results;

    for (MapPoint * pMapPoint : gMapPoints){
        Eigen::Vector3d mPosition = dMapPointsAndVertices[pMapPoint]->estimate();
        pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
    }

    for (KeyFrame * pKeyFrame : gKeyFrames){
        Eigen::Matrix4d mUpdatedPose =  Eigen::Isometry3d(dKeyFramesAndVertices[pKeyFrame]->estimate()).matrix();
        pKeyFrame->SetPose(mUpdatedPose);
    }


}







void SingleMapOptimizer::OptimizeCascade2(int nIterations){
    //Generate the optimizer.
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton * solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm(solver);
    //If output the debug info.
    iOptimizer.setVerbose(true);

    set<MapPoint *> sAllMapPoints = this->m_pMap->GetMapPointsSet();
    set<KeyFrame *> sAllKeyFrames = this->m_pMap->GetKeyFramesSet();

    int nLastKeyFrameSize = 0;

    set<MapPoint *> sCommonMapPoints;
    set<KeyFrame *> sCommonKeyFrames;
    sCommonMapPoints.clear();
    sCommonKeyFrames.clear();
    // cout << "Common MapPoints size: " << sCommonMapPoints.size() << endl;
    // cout << "Common KeyFrames size: " << sCommonKeyFrames.size() << endl;



    set<KeyFrame *> sNewKeyFrames = this->m_pMap->GetCommonKeyFrames();
    set<MapPoint *> sNewMapPoints = this->m_pMap->GetCommonMapPoints();
    map<KeyFrame *, g2o::VertexSE3Expmap *> dKeyFramesAndVertices;
    map<MapPoint *, g2o::VertexSBAPointXYZ *> dMapPointsAndVertices;
    int nVertexIndex = 3;

    //Add camera vertex
    int nCameraParameterId = 0;

    //Add the camera parameter vertex
    if (this->m_pMap->GetKeyFrames().size() == 0){
        cerr << "There are no keyframes in the map during optimization!" << endl;
        return;
    }

    cv::Mat mK = this->m_pMap->GetKeyFrames()[0]->GetCamera()->GetK();
    g2o::CameraParameters * pCameraParameter = new g2o::CameraParameters(
        mK.at<double>(0 , 0), 
        Eigen::Vector2d(mK.at<double>(0 , 2) , mK.at<double>(1 , 2)), 
        0);
    pCameraParameter->setId(nCameraParameterId);
    //Add parameters to the optimizer.
    iOptimizer.addParameter(pCameraParameter);



    while (sNewKeyFrames.size() > 0){
        //Add vertices.
        //Add new mappoints in this iteration.
        for (MapPoint * pMapPoint : sNewMapPoints){
            nVertexIndex++;
            cv::Point3d iPoint3D = pMapPoint->GetPosition();
            g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
            pPointVertex->setId(nVertexIndex);
            dMapPointsAndVertices[pMapPoint] = pPointVertex;
            pPointVertex->setEstimate(Eigen::Vector3d(iPoint3D.x,
                                                iPoint3D.y,
                                                iPoint3D.z));
            pPointVertex->setMarginalized(true);
            pPointVertex->setFixed(true);
            iOptimizer.addVertex(pPointVertex);

            //Add relative reprojection errors.
            vector<KeyFrame *> gObservedKeyFrames = pMapPoint->GetObservedKeyFrames();
            for (KeyFrame * pKeyFrame : gObservedKeyFrames){
                //Internal keyframes
                if (sCommonKeyFrames.find(pKeyFrame) != sCommonKeyFrames.end()){
                    nVertexIndex ++;  
                    cv::Point2d iObservation = pKeyFrame->GetObservation(pMapPoint);
                     
                    g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
                    pEdge->setId( nVertexIndex );
                    //Linked with the position of 3d points.
                    pEdge->setVertex(0 , pPointVertex);
                    pEdge->setVertex(1 , dKeyFramesAndVertices[pKeyFrame]);
                    //Set measurement and camera parameters.
                    pEdge->setMeasurement(Eigen::Vector2d(iObservation.x, iObservation.y));
                    pEdge->setParameterId(0 , nCameraParameterId);
                    pEdge->setInformation(Eigen::Matrix2d::Identity());
            
                    iOptimizer.addEdge(pEdge);
                }
            }
        }

        //Add new mappoints to the common mappoints set.
        for (MapPoint * pMapPoint : sNewMapPoints){
            sCommonMapPoints.insert(pMapPoint);
        }

        //Add new keyframes in this iteration.
        for (KeyFrame * pKeyFrame : sNewKeyFrames){
            nVertexIndex ++;
            
            //Add the pose vertex of this keyframe
            Eigen::MatrixXd mPose = pKeyFrame->GetPose();
            Eigen::Matrix3d mRotation = mPose.block(0 , 0 , 3 , 3);
            Eigen::Vector3d mTranslation = mPose.block(0 , 3 , 3 , 1);
            g2o::VertexSE3Expmap * pPoseVertex = new g2o::VertexSE3Expmap();
            dKeyFramesAndVertices[pKeyFrame] = pPoseVertex;
            
            pPoseVertex->setId(nVertexIndex);
            pPoseVertex->setEstimate( g2o::SE3Quat(
                    mRotation,
                    mTranslation));

            iOptimizer.addVertex(pPoseVertex);


            //Add relative reprojection errors.
            vector<MapPoint *> gObservedMapPoints = pKeyFrame->GetMapPoints();
            for (MapPoint * pMapPoint : gObservedMapPoints){
                if (sCommonMapPoints.find(pMapPoint) != sCommonMapPoints.end()){
                    nVertexIndex ++;  
                    cv::Point2d iObservation = pKeyFrame->GetObservation(pMapPoint);
                     
                    g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
                    pEdge->setId( nVertexIndex );
                    //Linked with the position of 3d points.
                    pEdge->setVertex(0 , dMapPointsAndVertices[pMapPoint]);
                    pEdge->setVertex(1 , pPoseVertex);
                    //Set measurement and camera parameters.
                    pEdge->setMeasurement(Eigen::Vector2d(iObservation.x, iObservation.y));
                    pEdge->setParameterId(0 , nCameraParameterId);
                    pEdge->setInformation(Eigen::Matrix2d::Identity());
            
                    iOptimizer.addEdge(pEdge);
                }
            }
        }

        //Add new keyframes to the common keyframes set.
        for (KeyFrame * pKeyFrame : sNewKeyFrames){
            sCommonKeyFrames.insert(pKeyFrame);
        }

        //Initialize optimization
        iOptimizer.initializeOptimization();

        iOptimizer.optimize(nIterations);

        //Obtain new mappoints and keyframes used in the optimization of the next iteration.
        sNewKeyFrames.clear();
        sNewMapPoints.clear();

        for (KeyFrame * pKeyFrame : sCommonKeyFrames){
            vector<KeyFrame *> gCovisibleKeyFrames = pKeyFrame->GetCovisibleKeyFrames();
            for (KeyFrame * pKeyFrame : gCovisibleKeyFrames){
                if (sCommonKeyFrames.find(pKeyFrame) == sCommonKeyFrames.end()){
                    sNewKeyFrames.insert(pKeyFrame);    
                }
            }
        }

        for (KeyFrame * pKeyFrame : sNewKeyFrames){
            vector<MapPoint *> gObservedMapPoints = pKeyFrame->GetMapPoints();
            for (MapPoint * pMapPoint : gObservedMapPoints){
                if (sCommonMapPoints.find(pMapPoint) == sCommonMapPoints.end()){
                    sNewMapPoints.insert(pMapPoint);    
                }
            }
        }


        for (KeyFrame * pKeyFrame : sCommonKeyFrames){
            vector<MapPoint *> gObservedMapPoints = pKeyFrame->GetMapPoints();
            for (MapPoint * pMapPoint : gObservedMapPoints){
                if (sCommonMapPoints.find(pMapPoint) == sCommonMapPoints.end()){
                    sNewMapPoints.insert(pMapPoint);    
                }
            }
        }

    }

    for (MapPoint * pMapPoint : sAllMapPoints){
        bool bFound = false;
        for (KeyFrame * pKeyFrame : sAllKeyFrames){
            if (pKeyFrame->HasBeenObserved(pMapPoint)){
                bFound = true;
            }
        }
        if (!bFound){
            cout << "Not found!" << endl;
        }
    }
    cout << "All found!" << endl;


    //Load results;

    for (MapPoint * pMapPoint : sAllMapPoints){
        Eigen::Vector3d mPosition = dMapPointsAndVertices[pMapPoint]->estimate();
        pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
    }

    for (KeyFrame * pKeyFrame : sAllKeyFrames){
        Eigen::Matrix4d mUpdatedPose =  Eigen::Isometry3d(dKeyFramesAndVertices[pKeyFrame]->estimate()).matrix();
        pKeyFrame->SetPose(mUpdatedPose);
    }


}

void SingleMapOptimizer::OptimizeWithPoseGraph(int nIterations){
    //Construct the optimizer.
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 每个误差项优化变量维度为3，误差值维度为1s
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    //Now we use Levenberg solver. GN,LM,Dogleg is also avaliable.
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton * solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    g2o::SparseOptimizer iOptimizer;
    iOptimizer.setAlgorithm(solver);
    //If output the debug info.
    iOptimizer.setVerbose(true);

    set<MapPoint *> sAllMapPoints = this->m_pMap->GetMapPointsSet();
    set<KeyFrame *> sAllKeyFrames = this->m_pMap->GetKeyFramesSet();



    set<MapPoint *> sCommonMapPoints = this->m_pMap->GetCommonMapPoints();
    set<KeyFrame *> sCommonKeyFrames = this->m_pMap->GetCommonKeyFrames();




    map<KeyFrame *, g2o::VertexSE3Expmap *> dKeyFramesAndVertices;
    map<MapPoint *, g2o::VertexSBAPointXYZ *> dMapPointsAndVertices;

    //Add all keyframes.
    int nVertexIndex = 3;

        //Add camera vertex
    int nCameraParameterId = 0;

    //Add the camera parameter vertex
    if (this->m_pMap->GetKeyFrames().size() == 0){
        cerr << "There are no keyframes in the map during optimization!" << endl;
        return;
    }

    cv::Mat mK = this->m_pMap->GetKeyFrames()[0]->GetCamera()->GetK();
    g2o::CameraParameters * pCameraParameter = new g2o::CameraParameters(
        mK.at<double>(0 , 0), 
        Eigen::Vector2d(mK.at<double>(0 , 2) , mK.at<double>(1 , 2)), 
        0);
    pCameraParameter->setId(nCameraParameterId);
    //Add parameters to the optimizer.
    iOptimizer.addParameter(pCameraParameter);


    bool bFirstKeyFrame = true;
    for (KeyFrame * pKeyFrame : sAllKeyFrames){
        nVertexIndex ++;
            
        //Add the pose vertex of this keyframe
        Eigen::MatrixXd mPose = pKeyFrame->GetPose();
        Eigen::Matrix3d mRotation = mPose.block(0 , 0 , 3 , 3);
        Eigen::Vector3d mTranslation = mPose.block(0 , 3 , 3 , 1);
        g2o::VertexSE3Expmap * pPoseVertex = new g2o::VertexSE3Expmap();
        dKeyFramesAndVertices[pKeyFrame] = pPoseVertex;
            
        pPoseVertex->setId(nVertexIndex);
        pPoseVertex->setEstimate( g2o::SE3Quat(
                mRotation,
                mTranslation));


        iOptimizer.addVertex(pPoseVertex);
    }

    //Add mappoints in the common region
    for (MapPoint * pMapPoint : sCommonMapPoints){
        nVertexIndex++;
        cv::Point3d iPoint3D = pMapPoint->GetPosition();
        g2o::VertexSBAPointXYZ * pPointVertex = new g2o::VertexSBAPointXYZ();
        pPointVertex->setId(nVertexIndex);
        dMapPointsAndVertices[pMapPoint] = pPointVertex;
        pPointVertex->setEstimate(Eigen::Vector3d(iPoint3D.x,
                                            iPoint3D.y,
                                            iPoint3D.z));
        // pPointVertex->setMarginalized(true);
        // pPointVertex->setFixed(true);
        iOptimizer.addVertex(pPointVertex);
    }

    //Add reprojection error edges.
    
    for (MapPoint * pMapPoint : sCommonMapPoints){
        cout << "Observed keyframe size: " << pMapPoint->GetObservedKeyFrames().size() << endl;
        for (KeyFrame * pKeyFrame : pMapPoint->GetObservedKeyFrames()){
            nVertexIndex++;
            cv::Point2d iObservation = pKeyFrame->GetObservation(pMapPoint);    
            g2o::EdgeProjectXYZ2UV * pEdge = new g2o::EdgeProjectXYZ2UV();
            pEdge->setId( nVertexIndex );
            //Linked with the position of 3d points.
            pEdge->setVertex(0 , dMapPointsAndVertices[pMapPoint]);
            pEdge->setVertex(1 , dKeyFramesAndVertices[pKeyFrame]);
            //Set measurement and camera parameters.
            pEdge->setMeasurement(Eigen::Vector2d(iObservation.x, iObservation.y));
            pEdge->setParameterId(0 , nCameraParameterId);
            pEdge->setInformation(Eigen::Matrix2d::Identity());
            iOptimizer.addEdge(pEdge);
        }        
    }

    //Add pose graph edges.

    set<KeyFrame *> sNotCommonKeyFrames;
    for (KeyFrame * pKeyFrame : sAllKeyFrames){
        if (sCommonKeyFrames.find(pKeyFrame) != sCommonKeyFrames.end()){
            sNotCommonKeyFrames.insert(pKeyFrame);
        }
    }



    const Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity();


    for (KeyFrame * pKeyFrame1 : sNotCommonKeyFrames){
        for (KeyFrame * pKeyFrame2 : sAllKeyFrames){
            if (!pKeyFrame1->CheckCovisibility(pKeyFrame2)){
                continue;
            }

            nVertexIndex++;
            //Add the edge of pose graph
            g2o::EdgeSE3Expmap * pPoseGraphEdge = new g2o::EdgeSE3Expmap();
            pPoseGraphEdge->setId( nVertexIndex );
            //Linked with the position of 3d points.
            pPoseGraphEdge->setVertex(0 , dKeyFramesAndVertices[pKeyFrame1]);
            pPoseGraphEdge->setVertex(1 , dKeyFramesAndVertices[pKeyFrame2]);

            //Estimate = Pose2*Pose1.inv()
            Eigen::MatrixXd mPose1Matrix = pKeyFrame1->GetPose();
            Eigen::MatrixXd mPose2Matrix = pKeyFrame2->GetPose();

            Eigen::Matrix3d mRotation1 = mPose1Matrix.block(0 , 0 , 3 , 3);
            Eigen::Vector3d mTranslation1 = mPose1Matrix.block(0 , 3 , 3 , 1);
            Eigen::Matrix3d mRotation2 = mPose2Matrix.block(0 , 0 , 3 , 3);
            Eigen::Vector3d mTranslation2 = mPose2Matrix.block(0 , 3 , 3 , 1);

            Sophus::SE3 mPose1 = Sophus::SE3(mRotation1, mTranslation1);
            Sophus::SE3 mPose2 = Sophus::SE3(mRotation2, mTranslation2);

            Sophus::SE3 mEstimate = (mPose2*mPose1.inverse());

            g2o::VertexSE3Expmap * pVertex1  = dKeyFramesAndVertices[pKeyFrame1];
            g2o::VertexSE3Expmap * pVertex2  = dKeyFramesAndVertices[pKeyFrame2];

            


            pPoseGraphEdge->setMeasurement(g2o::SE3Quat(
                mEstimate.rotation_matrix(),
                mEstimate.translation()));

            pPoseGraphEdge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());

            iOptimizer.addEdge(pPoseGraphEdge);
        }
    }

    //Initialize optimization
    iOptimizer.initializeOptimization();
    iOptimizer.optimize(100);

    //Load results;

    for (MapPoint * pMapPoint : sCommonMapPoints){
        Eigen::Vector3d mPosition = dMapPointsAndVertices[pMapPoint]->estimate();
        pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
    }

    for (KeyFrame * pKeyFrame : sAllKeyFrames){
        Eigen::Matrix4d mUpdatedPose =  Eigen::Isometry3d(dKeyFramesAndVertices[pKeyFrame]->estimate()).matrix();
        pKeyFrame->SetPose(mUpdatedPose);
    }


}