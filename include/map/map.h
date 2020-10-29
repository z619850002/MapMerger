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

	void UpdateCovisibleGraph();

	vector<KeyFrame *> GetKeyFrames();
	vector<MapPoint *> GetMapPoints();

	void Localize();

	int GetId();

	void AddNoise();

	void SetScale(double nScale);
	double GetScale();

private:
	unsigned int m_nId;
	vector<KeyFrame *> m_gKeyFrames;
	set<KeyFrame *> m_sKeyFrames;
	set<MapPoint *> m_sMapPoints;

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




#endif