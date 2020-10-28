#ifndef REAL_POINT_H_
#define REAL_POINT_H_


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
#include "../camera/camera.h"
#include "../mappoint/mappoint.h"
#include "../map/map.h"


using namespace std;

class MapPoint;
class Map;

class RealPoint
{
public:
	RealPoint();

	RealPoint(cv::Point3d iPosition);
	
	void AddMapPoint(Map * pMap, MapPoint * pMapPoint);

	cv::Point3d GetPosition();
	void SetPosition(cv::Point3d iPosition);



private:

	unsigned int m_nId;




	map<Map *, MapPoint *> m_dMapAndMapPoints;
	
	cv::Point3d m_iPosition;


	static unsigned int m_nCountID;

	
};


inline void RealPoint::AddMapPoint(Map * pMap, MapPoint * pMapPoint){
	this->m_dMapAndMapPoints[pMap] = pMapPoint;
}


inline void RealPoint::SetPosition(cv::Point3d iPosition){
	this->m_iPosition = iPosition;
}

inline cv::Point3d RealPoint::GetPosition(){
	return this->m_iPosition;
}


#endif