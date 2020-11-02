#include "../../include/map/map.h"
#include "../../include/merger/map_merger.h"
#include "../../include/optimizer/sim3_solver.h"
using namespace std;


unsigned int Map::m_nCountID = 0;

Map::Map(){
	this->m_nId = Map::m_nCountID++;
	this->m_nScale = 1;
	this->m_sCommonKeyFrames.clear();
	this->m_sFusedMapPoints.clear();
}


void Map::UpdateCovisibleGraph(){
	set<KeyFrame *>::iterator pIter, pIter2;
	for (	pIter = m_sKeyFrames.begin();
			pIter != m_sKeyFrames.end();
			pIter++){
		for (	pIter2 = pIter;
			pIter2 != m_sKeyFrames.end();
			pIter2++){
			if (pIter2 == pIter){
				continue;
			}
			int nCovisibilities = (*pIter)->CheckCovisibility(*pIter2);
			if (nCovisibilities > 0){
				(*pIter)->AddCovisibleEdge(*pIter2, nCovisibilities);
				(*pIter2)->AddCovisibleEdge(*pIter, nCovisibilities);	
			}
		}	
	}
}



void Map::Localize(){
	KeyFrame * pFirstKeyFrame = this->m_gKeyFrames[0];
	this->m_mLocalPose = pFirstKeyFrame->GetPose();
	for (KeyFrame * pKeyFrame : this->m_gKeyFrames){
		//Rescale.

		Eigen::MatrixXd mNewPose =  pKeyFrame->GetPose() * this->m_mLocalPose.inverse();
		Eigen::Matrix3d mRotation = mNewPose.block(0, 0 , 3 , 3);
		Eigen::Vector3d mTranslation = mNewPose.block(0 , 3 , 3 , 1);


		Sophus::SE3 mScaledPose = Sophus::SE3(mRotation,  mTranslation * this->m_nScale);
		pKeyFrame->SetPose(mScaledPose.matrix());
	}

	for (MapPoint * pMapPoint : this->m_sMapPoints){
		cv::Point3d iPosition = pMapPoint->GetPosition();
		Eigen::Vector4d mPosition;
		mPosition << iPosition.x,iPosition.y, iPosition.z, 1.0;
		mPosition = this->m_mLocalPose * mPosition * this->m_nScale;
		pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
	}
}
 

void Map::AddNoise(double nSigmaMapPoint, double nSigma){
	
	cv::RNG iRNG(10000);
	//The noise should be added to both the pose and the map point
	for (MapPoint * pMapPoint : this->m_sMapPoints){
		cv::Point3d iPosition = pMapPoint->GetPosition();
		double nNoiseX = iRNG.gaussian(nSigmaMapPoint);
		double nNoiseY = iRNG.gaussian(nSigmaMapPoint);
		double nNoiseZ = iRNG.gaussian(nSigmaMapPoint);

		iPosition.x = iPosition.x + nNoiseX * this->m_nScale;
		iPosition.y = iPosition.y + nNoiseX * this->m_nScale;
		iPosition.z = iPosition.z + nNoiseX * this->m_nScale;
		pMapPoint->SetPosition(iPosition);
	}

	for (KeyFrame * pKeyFrame : this->m_sKeyFrames){
		Eigen::MatrixXd mPose = pKeyFrame->GetPose();
		Eigen::Matrix3d mRotation = mPose.block(0, 0 , 3 , 3);
		Eigen::Vector3d mTranslation = mPose.block(0 , 3 , 3 , 1);
		Sophus::SE3 mSophusPose = Sophus::SE3(mRotation,  mTranslation);
		Eigen::Matrix<double, 6 , 1> mVec = Sophus::SE3::log(mSophusPose);
		
		for (int i=0;i<6;i++){
			mVec(i , 0) += iRNG.gaussian(nSigma) * this->m_nScale;
		}

		Sophus::SE3 mNewPose = Sophus::SE3::exp(mVec);
		pKeyFrame->SetPose(mNewPose.matrix());
	}
}



void Map::Transform(Eigen::MatrixXd mTransform){

	Eigen::Matrix3d mScaledRotation = mTransform.block(0 , 0 , 3 , 3);
	double nScale = mScaledRotation.determinant();
	Eigen::Matrix3d mRotation = mScaledRotation;
	Eigen::Vector3d mTranslation = mTransform.block(0 , 3 , 3 , 1);

	// Eigen::MatrixXd
	
	for (MapPoint * pMapPoint : this->GetMapPoints()){
		cv::Point3d iPosition = pMapPoint->GetPosition();

		Eigen::Vector4d mPosition(iPosition.x, iPosition.y, iPosition.z , 1.0);

		mPosition = mTransform * mPosition;
		
		pMapPoint->SetPosition(cv::Point3d(	mPosition(0), 
											mPosition(1), 
											mPosition(2)));
	}

	for (KeyFrame * pKeyFrame : this->m_sKeyFrames){
		// Sophus::SE3 mSophusPose = Sophus::SE3(mRotation, mTranslation/nScale);

		Eigen::MatrixXd mPose = pKeyFrame->GetPose() * mTransform.inverse();
		Eigen::Matrix3d mScaledRotation2 = mPose.block(0 , 0 , 3 , 3);
		
		
		double nScale2 = mScaledRotation2.determinant();
		mPose /= pow(nScale2, 1.0/3.0);
		mPose(3 , 3) = 1;

		
		
		pKeyFrame->SetPose(mPose);
	}
}



void Map::MergeMap(Map * pMergedMap){
	vector<KeyFrame *> gMergedKeyFrames = pMergedMap->GetKeyFrames();
	vector<MapPoint *> gMergedMapPoints = pMergedMap->GetMapPoints();
	

	//Fused matched mappoints.
	MapMerger * pMerger = new MapMerger();
	vector<MapPoint *> gMatchedMapPoints1, gMatchedMapPoints2;
	pMerger->MatchMap(this, pMergedMap, gMatchedMapPoints1, gMatchedMapPoints2);

	this->m_sFusedMapPoints.clear();
	this->m_sCommonKeyFrames.clear();

	for (int i=0;i<gMatchedMapPoints1.size();i++){
		MapPoint * pMapPoint1 = gMatchedMapPoints1[i];
		MapPoint * pMapPoint2 = gMatchedMapPoints2[i];
		cv::Point3d iPosition1 = pMapPoint1->GetPosition();
		cv::Point3d iPosition2 = pMapPoint2->GetPosition();

		iPosition1.x = (iPosition1.x + iPosition2.x)/2;
		iPosition1.y = (iPosition1.y + iPosition2.y)/2;
		iPosition1.z = (iPosition1.z + iPosition2.z)/2;

		pMapPoint1->SetPosition(iPosition1);

		for (KeyFrame * pNewKeyFrame : gMergedKeyFrames){
			pNewKeyFrame->ReplaceMapPoint(pMapPoint2, pMapPoint1);
			this->m_sCommonKeyFrames.insert(pNewKeyFrame);
		}

		this->m_sFusedMapPoints.insert(pMapPoint1);


		for (KeyFrame * pKeyFrame : this->m_gKeyFrames){
			if (pKeyFrame->HasBeenObserved(pMapPoint1)){
				this->m_sCommonKeyFrames.insert(pKeyFrame);
			}
		}

	}


	//Merge keyframes and mappoints.
	for (KeyFrame * pKeyFrame : gMergedKeyFrames){
		this->AddKeyFrame(pKeyFrame);
	}
	for (MapPoint * pMapPoint : gMergedMapPoints){
		this->AddMapPoint(pMapPoint);
	}

	//Remove duplicate mappoints.
	for (MapPoint * pDeletedMapPoint : gMatchedMapPoints2){
		this->DeleteMapPoint(pDeletedMapPoint);
	}

	//Update observation relationship
	this->UpdateCovisibleGraph();



}

double Map::ComputeATE(vector<Sophus::SE3> gGroundTruth, vector<double> & gErrors){
	
	//Find the sim3 transformation.
	vector<MapPoint *> gCurrentMapPoints = this->GetMapPoints();
	vector<MapPoint *> gGroundTruthMapPoints;
	gGroundTruthMapPoints.reserve(gCurrentMapPoints.size());

	for (MapPoint * pMapPoint : gCurrentMapPoints){
		RealPoint * pRealPoint = pMapPoint->GetRealPoint();
		gGroundTruthMapPoints.push_back(new MapPoint(pRealPoint));
	}

	Sim3Solver * pSolver = new Sim3Solver(gCurrentMapPoints, gGroundTruthMapPoints, true);
	pSolver->ComputeSim3();
	Eigen::MatrixXd mSim3Transform = pSolver->GetEigenSim3Matrix();



	vector<Eigen::Vector3d> gGroundTruthTrajectory, gCurrentTrajectory;
	


	for (Sophus::SE3 mPose : gGroundTruth){
		Eigen::Vector3d mTrajectory = mPose.inverse().translation();
		mTrajectory = mSim3Transform * mTrajectory;
		gGroundTruthTrajectory.push_back(mTrajectory);
	}

	vector<KeyFrame *> gKeyFrames = this->GetKeyFrames();
	for (KeyFrame * pKeyFrame : gKeyFrames){
		Eigen::Vector3d mTrajectory = pKeyFrame->GetPose().inverse().block(0 , 3 , 3 , 1);
		gCurrentTrajectory.push_back(mTrajectory);
	}

	double nTranslations = 0.0;
	gErrors.reserve(gKeyFrames.size());

	for (int i=0; i<gKeyFrames.size(); i++){
		double nLocalError = 0.0;
		Eigen::Vector3d mTranslation = gGroundTruthTrajectory[i] - gCurrentTrajectory[i];
		nLocalError += mTranslation(0) * mTranslation(0);
		nLocalError += mTranslation(1) * mTranslation(1);
		nLocalError += mTranslation(2) * mTranslation(2);
		nTranslations += nLocalError;
		gErrors.push_back(sqrt(nLocalError));
	}

	nTranslations /= (double)(gKeyFrames.size());
	nTranslations = sqrt(nTranslations);

	return nTranslations;

}