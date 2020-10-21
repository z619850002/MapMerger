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



class Camera
{
public:
	Camera();
	Camera(cv::Mat mK);

	cv::Point2d ProjectPoint(cv::Point3d iPointCamera);

	void SetK(cv::Mat mK);
	cv::Mat GetK();


private:
	cv::Mat m_mK;

	
};


inline void Camera::SetK(cv::Mat mK){
	this->m_mK = mK;
}

inline cv::Mat Camera::GetK(){
	return this->m_mK;
}





#endif