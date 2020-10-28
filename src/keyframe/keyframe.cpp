#include "../../include/keyframe/keyframe.h"
#include "../../include/mappoint/mappoint.h"
#include <algorithm>
#include <iterator>
#include <map>

using namespace std;


unsigned int KeyFrame::m_nCountID = 0;



KeyFrame::KeyFrame(){
	this->m_nId = KeyFrame::m_nCountID++;
}

KeyFrame::KeyFrame(Camera * pCamera){
	this->m_pCamera = pCamera;
}

int KeyFrame::CheckCovisibility(KeyFrame * pKeyFrame){

	set<MapPoint *> sCurrentMapPoints, sReferenceMapPoints;
	map<MapPoint *, cv::Point2d>::iterator pIter;
	
	for (	pIter = this->m_dMapPointsAndObservations.begin();
			pIter != this->m_dMapPointsAndObservations.end();
			pIter++){
		sCurrentMapPoints.insert(pIter->first);
	}

	for (	pIter = pKeyFrame->m_dMapPointsAndObservations.begin();
			pIter != pKeyFrame->m_dMapPointsAndObservations.end();
			pIter++){
		sReferenceMapPoints.insert(pIter->first);
	}

	
    set<MapPoint *> sUnion;
    set_intersection(	sCurrentMapPoints.begin(),sCurrentMapPoints.end(),
    					sReferenceMapPoints.begin(),sReferenceMapPoints.end(),
    					inserter(sUnion,sUnion.begin()));
    return sUnion.size();
}


void KeyFrame::AddMapPoint(MapPoint * pMapPoint){
	cv::Point3d iPosition3d = pMapPoint->GetPosition();
	cv::Point2d iObservation = this->m_pCamera->ProjectPoint(iPosition3d);
	this->AddMapPoint(pMapPoint, iObservation);
}




vector<cv::Point3d> KeyFrame::SamplePoint(double nWidth, double nHeight, double nMinDepth, double nMaxDepth, int nNumber)
{
		vector<cv::Point3d> gPoints;
		gPoints.reserve(nNumber);
		cv::RNG iRNG(12345);
		for (int i=0;i<nNumber;i++){		
			double nImageX = iRNG.uniform(0.0, nWidth);
			double nImageY = iRNG.uniform(0.0, nHeight);
			double nDepth = iRNG.uniform(nMinDepth, nMaxDepth);

			cv::Mat mK = this->m_pCamera->GetK();
			cv::Mat mImagePoint = (cv::Mat_<double>(3 , 1) << nImageX, nImageY, 1.0);
			
			cv::Mat mCameraPoint = mK.inv() * mImagePoint * nDepth;
			Eigen::Vector4d mHomogeneousCameraPoint;
			mHomogeneousCameraPoint <<  mCameraPoint.at<double>(0 , 0),
										mCameraPoint.at<double>(1 , 0),
										mCameraPoint.at<double>(2 , 0),
										1.0;


			Eigen::Vector4d mWorldPoint = this->m_mPose.inverse() * mHomogeneousCameraPoint;

			gPoints.push_back(cv::Point3d(	mWorldPoint(0 , 0),
											mWorldPoint(1 , 0),
											mWorldPoint(2 , 0)));
		}
		return gPoints;
	}