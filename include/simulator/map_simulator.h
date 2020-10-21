#ifndef MAP_SIMULATOR_H_
#define MAP_SIMULATOR_H_

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

#include "../map/map.h"
#include "../mappoint/mappoint.h"
#include "../camera/camera.h"
#include "../keyframe/keyframe.h"


struct MapPointsRange
{
	double m_nMinX, m_nMaxX;
	double m_nMinY, m_nMaxY;
	double m_nMinZ, m_nMaxZ;

	MapPointsRange(){}

	MapPointsRange(	double nMinX, double nMinY, double nMinZ,
					double nMaxX, double nMaxY, double nMaxZ) 
		:m_nMinX(nMinX), m_nMaxX(nMaxX), 
		m_nMinY(nMinY), m_nMaxY(nMaxY), 
		m_nMinZ(nMinZ), m_nMaxZ(nMaxZ)
	{}
};


class MapSimulator
{
public:
	MapSimulator();
	
	void GenerateNewMap();
	void AddMap(Map * pMap);

	bool SwitchMap(Map * pMap);
	bool SwitchMap(int nMapId);


	vector<MapPoint *> SimulateMapPoints(int nNumber){
		vector<MapPoint *> gMapPoints;
		//Random number generator.
		cv::RNG iRNG(12345);
		for (int i = 0; i < nNumber; i++) 
    	{
            double nX = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinX, 
            							this->m_iMapSimulatorRange.m_nMaxX);

            double nY = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinY, 
            							this->m_iMapSimulatorRange.m_nMaxY);

            double nZ = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinZ, 
            							this->m_iMapSimulatorRange.m_nMaxZ);

            MapPoint * pNewMapPoint = new MapPoint(cv::Point3d(nX, nY, nZ));
            this->m_sMapPoints.insert(pNewMapPoint);
            gMapPoints.push_back(pNewMapPoint);
    	}
    	return gMapPoints;
	}

	vector<Sophus::SE3> SimulateTrajectory(Sophus::SE3 iStartPose, Sophus::SE3 iEndPose, int nNumber){
		//Random number generator
		cv::RNG iRNG(12345);
		double nSigma = 0.000001;

		vector<Sophus::SE3> gPoses;
		gPoses.reserve(nNumber+2);
		//Sampled all poses
		gPoses.push_back(iStartPose);
		for (int i=1;i<=nNumber;i++){
			double nRatio = 1.0/(double)nNumber * (double)i;
			Sophus::SE3 iSampledPose = TrajectoryInterpolate(
				iStartPose, 
				iEndPose, 
				nRatio);
			
			//Add noise to the smooth trajectory.
			double nNoiseX = iRNG.gaussian(nSigma);
			double nNoiseY = iRNG.gaussian(nSigma);
			double nNoiseZ = iRNG.gaussian(nSigma);
			double nNoiseR1 = iRNG.gaussian(nSigma);
			double nNoiseR2 = iRNG.gaussian(nSigma);
			double nNoiseR3 = iRNG.gaussian(nSigma);

			Eigen::Matrix<double, 6 , 1> mNoiseVec(6);
			mNoiseVec << nNoiseX, nNoiseY, nNoiseZ, nNoiseR1, nNoiseR2, nNoiseR3;

			iStartPose = Sophus::SE3::exp(iStartPose.log() + mNoiseVec);

			gPoses.push_back(iSampledPose);


		}
		gPoses.push_back(iEndPose);

		
		return gPoses;


	}

private:

	Sophus::SE3 TrajectoryInterpolate(
		Sophus::SE3 iStartPose, 
		Sophus::SE3 iEndPose, 
		double nRatio){
		Sophus::SE3 iRelativePose = iStartPose.inverse() * iEndPose;
		Eigen::Matrix<double,6,1> mRelativePose = iRelativePose.log();

		Eigen::Matrix<double,6,1> mInterpolation = mRelativePose * nRatio;

		return iStartPose * Sophus::SE3::exp(mInterpolation);
	}

	set<Map *> m_sMaps;

	set<MapPoint *> m_sMapPoints;

	map<int, Map *> m_dMapAndId;


	Map * m_pCurrentMap;
	
	MapPointsRange m_iMapSimulatorRange;
};

inline void MapSimulator::GenerateNewMap(){
	Map * pNewMap = new Map();
	this->m_sMaps.insert(pNewMap);
	this->m_dMapAndId[pNewMap->GetId()] = pNewMap;
}


inline void MapSimulator::AddMap(Map * pMap){
	this->m_sMaps.insert(pMap);
	this->m_dMapAndId[pMap->GetId()] = pMap;
}

inline bool MapSimulator::SwitchMap(Map * pMap){
	if (this->m_sMaps.find(pMap) != this->m_sMaps.end()){		
		this->m_pCurrentMap = pMap;
		return true;	
	}
	return false;
}



#endif