#include "../../include/camera/camera.h"
#include <math.h>
using namespace std;


Camera::Camera(){
	this->m_nObservationCovariance = 1.5;	
	this->m_nFov = 100;
}


Camera::Camera(cv::Mat mK)
	: m_mK(mK)
{
	this->m_nObservationCovariance = 1.5;	
	this->m_nFov = 100;
	this->m_nMaxDepth = 3;
}




cv::Point2d Camera::ProjectPoint(MapPoint * pMapPoint){
	return this->ProjectPoint(pMapPoint->GetPosition());
}


cv::Point2d Camera::ProjectPoint(cv::Point3d iPointCamera){

	
	double nHomogeneousX = iPointCamera.x/iPointCamera.z; 
	double nHomogeneousY = iPointCamera.y/iPointCamera.z;
	
	double nFx = this->m_mK.at<double>(0 , 0);
	double nFy = this->m_mK.at<double>(1 , 1);
	double nCx = this->m_mK.at<double>(0 , 2);
	double nCy = this->m_mK.at<double>(1 , 2);

	double nImageX = nHomogeneousX * nFx + nCx;
	double nImageY = nHomogeneousY * nFy + nCy;

	return cv::Point2d(nImageX, nImageY);
}


cv::Point2d Camera::ObservePoint(MapPoint * pMapPoint){
	return this->ObservePoint(pMapPoint->GetPosition());
}

cv::Point2d Camera::ObservePoint(cv::Point3d iPointCamera){
	cv::RNG iRNG(12345);
	cv::Point2d iAccuratePoint = this->ProjectPoint(iPointCamera);
	iAccuratePoint.x = iAccuratePoint.x + iRNG.gaussian(this->m_nObservationCovariance);
	iAccuratePoint.y = iAccuratePoint.y + iRNG.gaussian(this->m_nObservationCovariance);
	return iAccuratePoint;
}



bool Camera::CanBeObserved(MapPoint * pMapPoint){
	return this->CanBeObserved(pMapPoint->GetPosition());
}



bool Camera::CanBeObserved(cv::Point3d iPointCamera){
	// cout << "Point Camera is: " << iPointCamera << endl;
	if (iPointCamera.z<=0.01 || iPointCamera.z >= this->m_nMaxDepth){
		return false;
	}
	double nDistance = sqrt((iPointCamera.x * iPointCamera.x) + (iPointCamera.y * iPointCamera.y));
	double nDepth = iPointCamera.z;
	double nFovRad = (this->m_nFov/2)/(45.0 / atan(1.0));
	
	if (nDistance/nDepth > tan(nFovRad)){
		return false;
	}
	return true;
}

