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


class KeyFrame
{
public:
	KeyFrame();

	KeyFrame(Camera * pCamera);

	void AddMapPoint(MapPoint * pMapPoint, cv::Point2d iObservation);

	void AddMapPoint(MapPoint * pMapPoint);

	cv::Point2d GetObservation(MapPoint * pMapPoint);

	void SetPose(Eigen::MatrixXd mPose);

	Eigen::MatrixXd GetPose();

	void SetCamera(Camera * pCamera);

	Camera * GetCamera();

	bool HasBeenObserved(MapPoint * pMapPoint);

	void AddCovisibleEdge(KeyFrame * pKeyFrame, int nCovisiblePoints);

	bool RemoveCovisibleEdge(KeyFrame * pKeyFrame);

	int CheckCovisibility(KeyFrame * pKeyFrame);

	int GetId();

private:
	unsigned int m_nId;

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

#endif