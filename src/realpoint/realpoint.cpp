#include "../../include/realpoint/realpoint.h"


using namespace std;



unsigned int RealPoint::m_nCountID = 0;


RealPoint::RealPoint(){
	this->m_nId = RealPoint::m_nCountID ++;
}

RealPoint::RealPoint(cv::Point3d iPosition){
	this->m_nId = RealPoint::m_nCountID ++;
	this->m_iPosition = iPosition;
}


