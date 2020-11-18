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
	vector<Map *> GetMaps();

	bool SwitchMap(Map * pMap);
	bool SwitchMap(int nMapId);

	void SimulateScene();

	map<int, Sophus::SE3> GetGroundTruth();

	
	vector<MapPoint *> SimulateMapPoints(MapPointsRange iMapPointRange);

	vector<RealPoint *> SimulateRealPoints(int nNumber);

	vector<Sophus::SE3> SimulateTrajectory(Sophus::SE3 iStartPose, Sophus::SE3 iEndPose, int nNumber);

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

	set<RealPoint *> m_sRealPoints;
	

	map<int, Map *> m_dMapAndId;

	map<int, Sophus::SE3> m_dKeyFramePoseGroundTruth;


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


inline vector<Map *> MapSimulator::GetMaps(){
	vector<Map * > gMaps;
	gMaps.clear();
	gMaps.insert(gMaps.begin(), this->m_sMaps.begin(), this->m_sMaps.end());
	return gMaps;
}


#endif