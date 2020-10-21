#ifndef MAP_POINT_H_
#define MAP_POINT_H_


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
#include "../keyframe/keyframe.h"



using namespace std;

class KeyFrame;

class MapPoint
{
public:
	MapPoint();

	MapPoint(cv::Point3d iPosition);

	void AddKeyFrame(KeyFrame * pKeyFrame, cv::Point2d iObservation);

	cv::Point2d GetObservation(KeyFrame * pKeyFrame);

	void SetPosition(cv::Point3d iPosition);

	cv::Point3d GetPosition();

	int GetId();

private:
	unsigned int m_nId;

	map<KeyFrame *, cv::Point2d> m_dKeyFramesAndObservations;
	
	cv::Point3d m_iPosition;


	static unsigned int m_nCountID;
	
};





inline void MapPoint::AddKeyFrame(KeyFrame * pKeyFrame, cv::Point2d iObservation){
	this->m_dKeyFramesAndObservations[pKeyFrame] = iObservation;
}




inline cv::Point2d MapPoint::GetObservation(KeyFrame * pKeyFrame){
	if (this->m_dKeyFramesAndObservations.count(pKeyFrame)!=0){
		return this->m_dKeyFramesAndObservations[pKeyFrame];
	}
	cv::Point2d iWrongPoint(-1 , -1);
	cerr << "No observation in this keyframe" << endl;
	return iWrongPoint;
}

inline void MapPoint::SetPosition(cv::Point3d iPosition){
	this->m_iPosition = iPosition;
}

inline cv::Point3d MapPoint::GetPosition(){
	return this->m_iPosition;
}


inline int MapPoint::GetId(){
	return this->m_nId;
}



#endif