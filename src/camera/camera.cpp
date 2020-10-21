#include "../../include/camera/camera.h"

using namespace std;


Camera::Camera(){
	
}


Camera::Camera(cv::Mat mK)
	: m_mK(mK)
{

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
