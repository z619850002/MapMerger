#ifndef MAP_H_
#define MAP_H_

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//Sophus
#include "sophus/se3.h"

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
#include <set>
#include "../mappoint/mappoint.h"
#include "../camera/camera.h"
#include "../keyframe/keyframe.h"

using namespace std;


class Map
{
public:
	Map();

	void AddKeyFrame(KeyFrame * pKeyFrame);
	void AddMapPoint(MapPoint * pMapPoint);
	void DeleteMapPoint(MapPoint * pMapPoint);

	void UpdateCovisibleGraph();

	vector<KeyFrame *> GetKeyFrames();
	vector<MapPoint *> GetMapPoints();

	set<KeyFrame *> GetKeyFramesSet();
	set<MapPoint *> GetMapPointsSet();

	void Localize();

	int GetId();

	void AddNoise(double nSigmaMapPoint = 0.02, double nSigma = 0.02);

	void SetScale(double nScale);
	double GetScale();

	void Transform(Eigen::MatrixXd mTransform);

	void MergeMap(Map * pMergedMap);

	double ComputeATE(vector<Sophus::SE3> gGroundTruth, vector<double> & gErrors);


	set<KeyFrame *> GetCommonKeyFrames();
	set<MapPoint *> GetCommonMapPoints();

private:
	unsigned int m_nId;
	vector<KeyFrame *> m_gKeyFrames;
	set<KeyFrame *> m_sKeyFrames;
	set<MapPoint *> m_sMapPoints;

	set<MapPoint *> m_sFusedMapPoints;
	set<KeyFrame *> m_sCommonKeyFrames;

	Eigen::MatrixXd m_mLocalPose;

	static unsigned int m_nCountID;

	double m_nScale;

};


inline void Map::AddKeyFrame(KeyFrame * pKeyFrame){
	this->m_sKeyFrames.insert(pKeyFrame);
	this->m_gKeyFrames.push_back(pKeyFrame);
}

inline void Map::AddMapPoint(MapPoint * pMapPoint){
	this->m_sMapPoints.insert(pMapPoint);
}

inline void Map::DeleteMapPoint(MapPoint * pMapPoint){
	this->m_sMapPoints.erase(pMapPoint);
}



inline int Map::GetId(){
	return this->m_nId;
}

inline vector<KeyFrame *> Map::GetKeyFrames(){
	return this->m_gKeyFrames;
}


inline vector<MapPoint *> Map::GetMapPoints(){
	vector<MapPoint *> gMapPoints;
	gMapPoints.clear();
	gMapPoints.insert(gMapPoints.end(), this->m_sMapPoints.begin(), this->m_sMapPoints.end());
	return gMapPoints;
}

inline void Map::SetScale(double nScale){
	this->m_nScale = nScale;
}

inline double Map::GetScale(){
	return this->m_nScale;
}


inline set<KeyFrame *> Map::GetCommonKeyFrames(){
	return this->m_sCommonKeyFrames;
}

inline set<MapPoint *> Map::GetCommonMapPoints(){
	return this->m_sFusedMapPoints;
}


inline set<KeyFrame *> Map::GetKeyFramesSet(){
	return this->m_sKeyFrames;
}

inline set<MapPoint *> Map::GetMapPointsSet(){
	return this->m_sMapPoints;
}



#endif