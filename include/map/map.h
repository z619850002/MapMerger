#ifndef MAP_H_
#define MAP_H_

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
#include <set>
#include "../mappoint/mappoint.h"
#include "../camera/camera.h"
#include "../keyframe/keyframe.h"



class Map
{
public:
	Map();

	void AddKeyFrame(KeyFrame * pKeyFrame);
	void AddMapPoint(MapPoint * pMapPoint);

	void UpdateCovisibleGraph();

	int GetId();

private:
	unsigned int m_nId;
	set<KeyFrame *> m_sKeyFrames;
	set<MapPoint *> m_sMapPoints;
	

	static unsigned int m_nCountID;

};


inline void Map::AddKeyFrame(KeyFrame * pKeyFrame){
	this->m_sKeyFrames.insert(pKeyFrame);
}

inline void Map::AddMapPoint(MapPoint * pMapPoint){
	this->m_sMapPoints.insert(pMapPoint);
}


inline int Map::GetId(){
	return this->m_nId;
}




#endif