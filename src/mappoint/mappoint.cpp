#include "../../include/keyframe/keyframe.h"
#include "../../include/mappoint/mappoint.h"


using namespace std;



unsigned int MapPoint::m_nCountID = 0;



MapPoint::MapPoint(){
	this->m_nId = MapPoint::m_nCountID ++;
}


MapPoint::MapPoint(cv::Point3d iPosition){
	this->m_nId = MapPoint::m_nCountID ++;
	this->m_iPosition = iPosition;	
}



MapPoint::MapPoint(RealPoint * pRealPoint){
	this->m_nId = MapPoint::m_nCountID ++;
	this->m_pRealPoint = pRealPoint;
}




