#ifndef KEYFRAME_H_
#define KEYFRAME_H_


//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//Std
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include "../mappoint/mappoint.h"
#include "../camera/camera.h"

using namespace std;

class MapPoint;
class Camera;

class KeyFrame
{
public:
	KeyFrame();

	KeyFrame(Camera * pCamera);

	void AddMapPoint(MapPoint * pMapPoint, cv::Point2d iObservation);

	void AddMapPoint(MapPoint * pMapPoint);

	bool CanObserve(cv::Point3d iPointWorld);


	cv::Point2d GetObservation(MapPoint * pMapPoint);

	void SetPose(Eigen::MatrixXd mPose);

	Eigen::MatrixXd GetPose();

	void SetCamera(Camera * pCamera);

	Camera * GetCamera();

	void SetPreviousKeyFrame(KeyFrame * pKeyFrame);

	KeyFrame * GetPreviousKeyFrame();


	bool HasBeenObserved(MapPoint * pMapPoint);

	void AddCovisibleEdge(KeyFrame * pKeyFrame, int nCovisiblePoints);

	bool RemoveCovisibleEdge(KeyFrame * pKeyFrame);

	int CheckCovisibility(KeyFrame * pKeyFrame);

	int GetId();

	vector<KeyFrame *> GetCovisibleKeyFrames();

	vector<cv::Point3d> SamplePoint(double nWidth, double nHeight, double nMinDepth, double nMaxDepth, int nNumber);

	vector<MapPoint *> GetMapPoints();

	map<MapPoint *, cv::Point2d> GetAllObservations();


private:
	unsigned int m_nId;

	KeyFrame * m_pPreviousKeyFrame;

	Camera * m_pCamera;

	//The pose matrix
	Eigen::MatrixXd m_mPose;

	map<MapPoint *, cv::Point2d> m_dMapPointsAndObservations;

	map<KeyFrame *, int>	m_dCovisibleEdges;
	
	static unsigned int m_nCountID;

};


inline void KeyFrame::AddMapPoint(MapPoint * pMapPoint, cv::Point2d iObservation){
	this->m_dMapPointsAndObservations[pMapPoint] = iObservation;
}



inline cv::Point2d KeyFrame::GetObservation(MapPoint * pMapPoint){
	if (this->m_dMapPointsAndObservations.count(pMapPoint)){
		return this->m_dMapPointsAndObservations[pMapPoint];	
	}
	cv::Point2d iWrongPoint(-1 , -1);
	cerr << "No observation in this keyframe" << endl;
	return iWrongPoint;
}


inline void KeyFrame::SetPose(Eigen::MatrixXd mPose){
	this->m_mPose = mPose;
}


inline Eigen::MatrixXd KeyFrame::GetPose(){
	return this->m_mPose;
}


inline void KeyFrame::SetCamera(Camera * pCamera){
	this->m_pCamera = pCamera;
}


inline Camera * KeyFrame::GetCamera(){
	return this->m_pCamera;
}


inline bool KeyFrame::HasBeenObserved(MapPoint * pMapPoint){
	return (this->m_dMapPointsAndObservations.count(pMapPoint)!=0);
}

inline void KeyFrame::AddCovisibleEdge( KeyFrame * pKeyFrame, 
										int nCovisiblePoints){
	this->m_dCovisibleEdges[pKeyFrame] = nCovisiblePoints;
}

inline bool KeyFrame::RemoveCovisibleEdge(KeyFrame * pKeyFrame){
	if (this->m_dCovisibleEdges.count(pKeyFrame)==0){
		return false;
	}
	m_dCovisibleEdges.erase(pKeyFrame);
	return true;
}

inline int KeyFrame::GetId(){
	return this->m_nId;
}


inline void KeyFrame::SetPreviousKeyFrame(KeyFrame * pKeyFrame){
	this->m_pPreviousKeyFrame = pKeyFrame;
}

inline KeyFrame * KeyFrame::GetPreviousKeyFrame(){
	return this->m_pPreviousKeyFrame;
}

inline vector<KeyFrame *> KeyFrame::GetCovisibleKeyFrames(){
	vector<KeyFrame *> gCovisibleKeyFrames;
	gCovisibleKeyFrames.reserve(this->m_dCovisibleEdges.size());
	for (auto pPair : this->m_dCovisibleEdges){
		gCovisibleKeyFrames.push_back(pPair.first);
	}
	return gCovisibleKeyFrames;
}


inline vector<MapPoint *> KeyFrame::GetMapPoints(){
	vector<MapPoint *> gMapPoints;
	gMapPoints.reserve(this->m_dMapPointsAndObservations.size());
	for (auto pPair : this->m_dMapPointsAndObservations){
		gMapPoints.push_back(pPair.first);
	}
	return gMapPoints;
}


inline map<MapPoint *, cv::Point2d> KeyFrame::GetAllObservations(){
	return this->m_dMapPointsAndObservations;
}

#endif