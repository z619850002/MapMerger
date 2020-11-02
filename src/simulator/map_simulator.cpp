#include "../../include/keyframe/keyframe.h"
#include "../../include/mappoint/mappoint.h"
#include "../../include/simulator/map_simulator.h"

using namespace std;

MapSimulator::MapSimulator(){
	this->m_iMapSimulatorRange = MapPointsRange(-10 , -10 , -10 , 10 , 10 , 10);
}


bool MapSimulator::SwitchMap(int nMapId){
	if (m_dMapAndId.count(nMapId) != 0){
		Map * pSwitchedMap = this->m_dMapAndId[nMapId];
		this->m_pCurrentMap = pSwitchedMap;
		return true;	
	}
	return false;
}


vector<MapPoint *> MapSimulator::SimulateMapPoints(MapPointsRange iMapPointRange){
	double nSigma = 0.1;
	cv::RNG iRNG(12345);
	vector<MapPoint *> gMapPoints;
	gMapPoints.reserve(this->m_sRealPoints.size());
	for (RealPoint * pRealPoint : this->m_sRealPoints){
		cv::Point3d iPosition = pRealPoint->GetPosition();
		double nX = iPosition.x;
		double nY = iPosition.y;
		double nZ = iPosition.z;
		if (nX >= this->m_iMapSimulatorRange.m_nMinX && 
			nX <= this->m_iMapSimulatorRange.m_nMaxX &&
			nY >= this->m_iMapSimulatorRange.m_nMinY &&
			nY <= this->m_iMapSimulatorRange.m_nMaxY &&
			nZ >= this->m_iMapSimulatorRange.m_nMinY &&
			nZ <= this->m_iMapSimulatorRange.m_nMaxY){

			MapPoint * pNewMapPoint = new MapPoint(pRealPoint);
			cv::Point3d iNewPosition(	nX + iRNG.gaussian(nSigma),
										nY + iRNG.gaussian(nSigma),
										nZ + iRNG.gaussian(nSigma));
			pNewMapPoint->SetPosition(iNewPosition);
			gMapPoints.push_back(pNewMapPoint);
		}

	}

	return gMapPoints;
}


vector<RealPoint *> MapSimulator::SimulateRealPoints(int nNumber){
	vector<RealPoint *> gRealPoints;
	//Random number generator.
	cv::RNG iRNG(12345);
	for (int i = 0; i < nNumber; i++) 
	{
        double nX = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinX, 
        							this->m_iMapSimulatorRange.m_nMaxX);

        double nY = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinY, 
        							this->m_iMapSimulatorRange.m_nMaxY);

        double nZ = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinZ, 
        							this->m_iMapSimulatorRange.m_nMaxZ);

        RealPoint * pNewRealPoint = new RealPoint(cv::Point3d(nX, nY, nZ));
        
        gRealPoints.push_back(pNewRealPoint);
	}
	this->m_sRealPoints = set<RealPoint *>(gRealPoints.begin(), gRealPoints.end()); 

	return gRealPoints;
}



vector<Sophus::SE3> MapSimulator::SimulateTrajectory(Sophus::SE3 iStartPose, Sophus::SE3 iEndPose, int nNumber){
	//Random number generator
	cv::RNG iRNG(12345);
	double nSigma = 0.0000000000001;

	vector<Sophus::SE3> gPoses;
	gPoses.reserve(nNumber+2);
	//Sampled all poses
	gPoses.push_back(iStartPose);
	for (int i=1;i<nNumber;i++){
		double nRatio = 1.0/(double)nNumber * (double)i;
		Sophus::SE3 iSampledPose = TrajectoryInterpolate(
			iStartPose, 
			iEndPose, 
			nRatio);
		
		//Add noise to the smooth trajectory.
		double nNoiseX = iRNG.gaussian(nSigma);
		double nNoiseY = iRNG.gaussian(nSigma);
		double nNoiseZ = iRNG.gaussian(nSigma);
		double nNoiseR1 = iRNG.gaussian(nSigma);
		double nNoiseR2 = iRNG.gaussian(nSigma);
		double nNoiseR3 = iRNG.gaussian(nSigma);

		Eigen::Matrix<double, 6 , 1> mNoiseVec(6);
		mNoiseVec << nNoiseX, nNoiseY, nNoiseZ, nNoiseR1, nNoiseR2, nNoiseR3;

		iStartPose = Sophus::SE3::exp(iStartPose.log() + mNoiseVec);

		gPoses.push_back(iSampledPose);


	}
	// gPoses.push_back(iEndPose);

	
	return gPoses;
}



void initializePose(Sophus::SE3& T_FG,Sophus::SE3& T_LG,
					Sophus::SE3& T_BG,Sophus::SE3& T_RG)
{
	double nScale = 1;


	// Initialize T_FG
	Eigen::Matrix3d R_FG = Eigen::Matrix3d();
	R_FG<< 9.99277118e-01,  3.82390286e-04, -3.80143958e-02,
		  -2.30748265e-02, -7.88582447e-01, -6.14495953e-01,
	      -3.02124625e-02,  6.14928921e-01, -7.88003572e-01;




	Eigen::Vector3d t_FG;
	t_FG<< 6.75437418e-01,  2.50896883e+01,  3.17779305e+00;
	T_FG = Sophus::SE3(R_FG,t_FG/nScale);



	
	// Initialize T_LG
	Eigen::Matrix3d R_LG = Eigen::Matrix3d();
	R_LG<<-1.21898860e-02,  9.99924056e-01, -1.81349393e-03,
		   8.02363600e-01,  8.69913885e-03, -5.96772133e-01,
	      -5.96711036e-01, -8.72966581e-03, -8.02408707e-01;
	Eigen::Vector3d t_LG;
	t_LG<< 1.36392943e+00,  1.60942881e+01,  1.04105913e+01;
	T_LG = Sophus::SE3(R_LG,t_LG/nScale);
	
	// Initialize T_BG
	Eigen::Matrix3d R_BG = Eigen::Matrix3d();
	R_BG<<-9.99615699e-01,  1.56439861e-02, -2.28849354e-02,
		   2.59906371e-02,  8.16008735e-01, -5.77454960e-01,
	       9.64060983e-03, -5.77827838e-01, -8.16101739e-01;
	Eigen::Vector3d t_BG;
	t_BG<< 1.09266953e+00,  2.46308124e+01,  6.60957845e+00;
	T_BG = Sophus::SE3(R_BG,t_BG/nScale);

	// Initialize T_RG
	Eigen::Matrix3d R_RG = Eigen::Matrix3d();
	R_RG<< 4.57647596e-03, -9.99989102e-01,  9.22798184e-04,
		  -6.26343448e-01, -3.58584197e-03, -7.79538984e-01,
	       7.79533797e-01,  2.98955282e-03, -6.26353033e-01;
	Eigen::Vector3d t_RG;
	t_RG<<-1.66115120e-01,  1.76226207e+01, 6.08338205e+00;
	T_RG = Sophus::SE3(R_RG,t_RG/nScale);
	
	return;
}





void MapSimulator::SimulateScene(){
	cv::Mat mK = (cv::Mat_<double>(3 , 3) << 500 , 0  , 640 , 0 , 500 , 360 , 0 , 0 , 1);
	Camera * pCamera = new Camera(mK);



	Sophus::SE3 mTF, mTL, mTB, mTR;
	initializePose(mTF, mTL, mTB, mTR);

	vector<Sophus::SE3> gPoses;

	gPoses = this->SimulateTrajectory(mTF, mTL, 8);

	vector<Sophus::SE3> gPoses2, gPoses3, gPoses4;
	gPoses2 = this->SimulateTrajectory(mTL, mTB, 8);
	gPoses3 = this->SimulateTrajectory(mTB, mTR, 8);
	gPoses4 = this->SimulateTrajectory(mTR, mTF, 8);
	gPoses.insert(gPoses.end(), gPoses2.begin(), gPoses2.end());
	gPoses.insert(gPoses.end(), gPoses3.begin(), gPoses3.end());
	gPoses.insert(gPoses.end(), gPoses4.begin(), gPoses4.end());



	vector<Sophus::SE3> gSecondPoses;



	Sophus::SE3 mT_transform =  mTB.inverse() * mTF;

	Eigen::Vector3d mTransformTranslation = mT_transform.translation();
	Eigen::Matrix3d mTransformRotation = mT_transform.rotation_matrix();
	Eigen::Vector3d mOnlyMoveTranslation = mTransformRotation * mTF.inverse().translation() + mTransformTranslation - mTF.inverse().translation();

	for (Sophus::SE3 iFirstPose : gPoses){

		Sophus::SE3 mRelativePose2 = Sophus::SE3(Eigen::Matrix3d::Identity(), mOnlyMoveTranslation * 1.1).inverse();
		Sophus::SE3 iSecondPose = iFirstPose * mRelativePose2;
		
		// Sophus::SE3 iSecondPose = iFirstPose *  Sophus::SE3(Eigen::Matrix3d::Identity(), mMove);
		gSecondPoses.push_back(iSecondPose);
	}



	for (int i=0;i<gPoses.size();i++){
		gPoses[i] = gPoses[i] * mTF.inverse();
		gSecondPoses[i] = gSecondPoses[i] * mTF.inverse();
	}

	vector<KeyFrame *> gAllKeyFrames, gFirstKeyFrames, gSecondKeyFrames;

	Map * pFirstMap = new Map();
	Map * pSecondMap = new Map();


	
	for (int i=0;i<gPoses.size();i++){
		Sophus::SE3 iPose = gPoses[i];
		KeyFrame * pKeyFrame = new KeyFrame(pCamera);
		pKeyFrame->SetPose(iPose.matrix());
		gAllKeyFrames.push_back(pKeyFrame);
		m_dKeyFramePoseGroundTruth[pKeyFrame->GetId()] = iPose;
		pFirstMap->AddKeyFrame(pKeyFrame);
		gFirstKeyFrames.push_back(pKeyFrame);
	}


	for (int i=0;i<gSecondPoses.size();i++){
		Sophus::SE3 iPose = gSecondPoses[i];
		KeyFrame * pKeyFrame = new KeyFrame(pCamera);
		pKeyFrame->SetPose(iPose.matrix());
		gAllKeyFrames.push_back(pKeyFrame);
		m_dKeyFramePoseGroundTruth[pKeyFrame->GetId()] = iPose;
		pSecondMap->AddKeyFrame(pKeyFrame);
		gSecondKeyFrames.push_back(pKeyFrame);
	}



	double nMaxDepth = pCamera->GetMaxDepth();

	for (KeyFrame * pKeyFrame : gAllKeyFrames){
		vector<cv::Point3d> gPoints = pKeyFrame->SamplePoint(1280, 720, 0.1, nMaxDepth+2, 30);
		for (cv::Point3d iPoint : gPoints){
			RealPoint * pNewRealPoint = new RealPoint(iPoint);
			// gAllRealPoints->AddRealPoints(pNewRealPoint);
			this->m_sRealPoints.insert(pNewRealPoint);
		}

	}

	set<RealPoint *> sRealPoints1, sRealPoints2;

	//Add map point to those 2 maps.
	for (RealPoint * pRealPoint : this->m_sRealPoints){
		for (KeyFrame * pKeyFrame : gFirstKeyFrames){
			if (pKeyFrame->CanObserve(pRealPoint->GetPosition())){
				sRealPoints1.insert(pRealPoint);
			}
		}

		for (KeyFrame * pKeyFrame : gSecondKeyFrames){
			if (pKeyFrame->CanObserve(pRealPoint->GetPosition())){
				sRealPoints2.insert(pRealPoint);
			}
		}
	}


	//Add observation
	for (RealPoint * pRealPoint : sRealPoints1){
		MapPoint * pNewMapPoint = new MapPoint(pRealPoint);
		pFirstMap->AddMapPoint(pNewMapPoint);
		for (KeyFrame * pKeyFrame : gFirstKeyFrames){
			if (pKeyFrame->CanObserve(pRealPoint->GetPosition())){
				pKeyFrame->AddMapPoint(pNewMapPoint);
			}
		}
	}

	for (RealPoint * pRealPoint : sRealPoints2){
		MapPoint * pNewMapPoint = new MapPoint(pRealPoint);
		pSecondMap->AddMapPoint(pNewMapPoint);
		for (KeyFrame * pKeyFrame : gSecondKeyFrames){
			if (pKeyFrame->CanObserve(pRealPoint->GetPosition())){
				pKeyFrame->AddMapPoint(pNewMapPoint);
			}
		}
	}


	this->AddMap(pFirstMap);
	this->AddMap(pSecondMap);

	pFirstMap->UpdateCovisibleGraph();
	pSecondMap->UpdateCovisibleGraph();

}




vector<Sophus::SE3> MapSimulator::GetGroundTruth(vector<KeyFrame *> gKeyFrames){
	vector<Sophus::SE3> gGroundTruth;
	gGroundTruth.reserve(gKeyFrames.size());

	for (KeyFrame * pKeyFrame : gKeyFrames){
		gGroundTruth.push_back(this->m_dKeyFramePoseGroundTruth[pKeyFrame->GetId()]);
	}

	return gGroundTruth;

}