#include "../../include/keyframe/keyframe.h"
#include "../../include/mappoint/mappoint.h"
#include <algorithm>
#include <iterator>
#include <map>

using namespace std;


unsigned int KeyFrame::m_nCountID = 0;



KeyFrame::KeyFrame(){
	this->m_nId = KeyFrame::m_nCountID++;
}

KeyFrame::KeyFrame(Camera * pCamera){
	this->m_pCamera = pCamera;
}

int KeyFrame::CheckCovisibility(KeyFrame * pKeyFrame){

	set<MapPoint *> sCurrentMapPoints, sReferenceMapPoints;
	map<MapPoint *, cv::Point2d>::iterator pIter;
	
	for (	pIter = this->m_dMapPointsAndObservations.begin();
			pIter != this->m_dMapPointsAndObservations.end();
			pIter++){
		sCurrentMapPoints.insert(pIter->first);
	}

	for (	pIter = pKeyFrame->m_dMapPointsAndObservations.begin();
			pIter != pKeyFrame->m_dMapPointsAndObservations.end();
			pIter++){
		sReferenceMapPoints.insert(pIter->first);
	}

	
    set<MapPoint *> sUnion;
    set_intersection(	sCurrentMapPoints.begin(),sCurrentMapPoints.end(),
    					sReferenceMapPoints.begin(),sReferenceMapPoints.end(),
    					inserter(sUnion,sUnion.begin()));
    return sUnion.size();
}


void KeyFrame::AddMapPoint(MapPoint * pMapPoint){
	cv::Point3d iPosition3d = pMapPoint->GetPosition();
	cv::Point2d iObservation = this->m_pCamera->ProjectPoint(iPosition3d);
	this->AddMapPoint(pMapPoint, iObservation);
}
