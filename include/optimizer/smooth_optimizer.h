#ifndef SMOOTH_OPTIMIZER_H_
#define SMOOTH_OPTIMIZER_H_

//Sophus
#include "sophus/se3.h"

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
#include <list>
#include <map>
#include <queue>
#include <set>
#include <iterator>

using namespace std;

class SmoothOptimizer
{
public:
	SmoothOptimizer(int nNodeSize){
		this->m_gConnections.reserve(nNodeSize);
		//Construct the connections.
		for (int i=0;i<nNodeSize;i++){
			this->m_gConnections.push_back(map<int,Sophus::SE3>());
		}
		this->m_nMatchSize = 3;
	}

	inline void AddKeyFramePose(Sophus::SE3 mPose){
		this->m_gKeyFramePoses.push_back(mPose);
	}

	inline void AddConnections(int nRefIndex, int nConnectedIndex, Sophus::SE3 mPose_rc){
		this->m_gConnections[nRefIndex][nConnectedIndex] = mPose_rc;
	}

	void Optimize(int nStartNode = -1){
		if (nStartNode == -1){
			nStartNode = this->m_gKeyFramePoses.size()-1;
		}
		cout << "Start to optimize" << endl;
		for (int nIteration = 0; nIteration < 20; nIteration++){
			for (int i=nStartNode;i>1;i--){
				vector<Sophus::SE3> gPoses;
				map<int, Sophus::SE3> dConnections = this->m_gConnections[i];
				gPoses.reserve(dConnections.size());
				for (map<int, Sophus::SE3>::iterator pIter = dConnections.begin(); pIter !=dConnections.end(); pIter++){
					Sophus::SE3 mRelativePose = pIter->second;
					int nConnectedIndex = pIter->first;
					Sophus::SE3 mEstimatePose = mRelativePose * this->m_gKeyFramePoses[nConnectedIndex];
					mEstimatePose = this->TrajectoryInterpolate(mEstimatePose, this->m_gKeyFramePoses[i], 0.5);
					gPoses.push_back(mEstimatePose);
				}
				if (gPoses.size()==0){
					continue;
				}
				Sophus::SE3 mUpdatedPose = this->TrajectoryInterpolateMulti(gPoses);
				this->m_gKeyFramePoses[i] = mUpdatedPose;
			}
		}
		
	}

	vector<Sophus::SE3> LoadPoses(){
		return this->m_gKeyFramePoses;
	}

private:
	int m_nMatchSize;
	vector<Sophus::SE3> m_gKeyFramePoses;

	vector<map<int, Sophus::SE3>> m_gConnections;


	inline Sophus::SE3 TrajectoryInterpolate(
		Sophus::SE3 iStartPose, 
		Sophus::SE3 iEndPose, 
		double nRatio){
		Sophus::SE3 iRelativePose = iStartPose.inverse() * iEndPose;
		Eigen::Matrix<double,6,1> mRelativePose = iRelativePose.log();

		Eigen::Matrix<double,6,1> mInterpolation = mRelativePose * nRatio;

		return iStartPose * Sophus::SE3::exp(mInterpolation);
	}


	inline Sophus::SE3 TrajectoryInterpolateMulti(
		vector<Sophus::SE3> gAllPoses){
		double nRatio = 1.0/gAllPoses.size();
		Sophus::SE3 iStartPose = gAllPoses[0];
		for (int i=1;i<gAllPoses.size();i++){
			Sophus::SE3 iEndPose = gAllPoses[i];
			Sophus::SE3 iRelativePose = iStartPose.inverse() * iEndPose;
			Eigen::Matrix<double,6,1> mRelativePose = iRelativePose.log();

			Eigen::Matrix<double,6,1> mInterpolation = mRelativePose * nRatio;
			iStartPose *= Sophus::SE3::exp(mInterpolation);
		}

		return iStartPose;
	}

};


#endif