#ifndef CAMERA_H_
#define CAMERA_H_


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

#include "../mappoint/mappoint.h"

class MapPoint;
class KeyFrame;

class Camera
{
public:
	Camera();
	Camera(cv::Mat mK);

	bool CanBeObserved(cv::Point3d iPointCamera);
	bool CanBeObserved(MapPoint * pMapPoint);

	cv::Point2d ObservePoint(cv::Point3d iPointCamera);
	cv::Point2d ObservePoint(MapPoint * pMapPoint);
	
	cv::Point2d ProjectPoint(cv::Point3d iPointCamera);
	cv::Point2d ProjectPoint(MapPoint * pMapPoint);

	void SetK(cv::Mat mK);
	cv::Mat GetK();

	double GetMaxDepth();
	void SetMaxDepth(double nMaxDepth);


private:
	cv::Mat m_mK;

	double m_nObservationCovariance;

	double m_nFov;

	double m_nMaxDepth;
	
};


inline void Camera::SetK(cv::Mat mK){
	this->m_mK = mK;
}

inline cv::Mat Camera::GetK(){
	return this->m_mK;
}


inline double Camera::GetMaxDepth(){
	return this->m_nMaxDepth;
}

inline void Camera::SetMaxDepth(double nMaxDepth){
	this->m_nMaxDepth = nMaxDepth;
}


#endif