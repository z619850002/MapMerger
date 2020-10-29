#ifndef SIM3_SOLVER_H_
#define SIM3_SOLVER_H_


//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>



#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <vector>
#include <cmath>

#include "../keyframe/keyframe.h"
#include "../mappoint/mappoint.h"
#include "../map/map.h"


using namespace std;


class Sim3Solver
{
public:
	Sim3Solver(vector<MapPoint *> gMapPoints1, vector<MapPoint *> gMapPoints2, bool bFixScale = false);
	void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);
	void ComputeSim3();

	Eigen::MatrixXd GetEigenSim3Matrix();
	cv::Mat GetCVSim3Matrix();


private:
	vector<MapPoint *> m_gMapPoints1, m_gMapPoints2;
	int m_nIterations;
	bool m_bFixScale;


	cv::Mat m_mRotation;
	cv::Mat m_mTranslation;
	double m_nScale;


};



#endif