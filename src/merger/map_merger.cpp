#include "../../include/merger/map_merger.h"

using namespace std;

//Default Constructor.
MapMerger::MapMerger(){

}
	
Map * MapMerger::MergeMap(Map * pMap1, Map * pMap2){
	//Firstly find matched mappoints between 2 maps;
	

	vector<MapPoint *> gMatchedPoints1, gMatchedPoints2;
	this->MatchMap(pMap1, pMap2, gMatchedPoints1, gMatchedPoints2);	


	//Find a sim3 transformation firstly

	//Then conduct the bundle adjustment approach.

}



bool MapMerger::MatchMap(		Map * pMap1, Map * pMap2,
								vector<MapPoint *> & gMapPoints1,
								vector<MapPoint *> & gMapPoints2){
	//Firstly find matched mappoints between 2 maps;
	vector<MapPoint *> gMap1MapPoints, 	gMap2MapPoints;
	gMap1MapPoints = pMap1->GetMapPoints();
	gMap2MapPoints = pMap2->GetMapPoints();



	gMapPoints1.reserve(gMap1MapPoints.size());
	gMapPoints2.reserve(gMap2MapPoints.size());
	for (MapPoint * pMapPoint1 : gMap1MapPoints){
		for (MapPoint * pMapPoint2 : gMap2MapPoints){
			if (this->MatchMapPoint(pMapPoint1, pMapPoint2)){
				gMapPoints1.push_back(pMapPoint1);
				gMapPoints2.push_back(pMapPoint2);
				break;
			}
		}
	}
	return gMapPoints2.size() > 6;
}



bool MapMerger::MatchKeyFrame(	KeyFrame * pKeyFrame1, KeyFrame * pKeyFrame2,
								vector<MapPoint *> & gMapPoints1,
								vector<MapPoint *> & gMapPoints2){

	gMapPoints1.clear();
	gMapPoints2.clear();
	gMapPoints1.reserve(pKeyFrame1->GetMapPoints().size());
	gMapPoints2.reserve(pKeyFrame2->GetMapPoints().size());

	for (MapPoint * pMapPoint1 : pKeyFrame1->GetMapPoints()){
		for (MapPoint * pMapPoint2 : pKeyFrame2->GetMapPoints()){
			if (this->MatchMapPoint(pMapPoint1, pMapPoint2)){
				gMapPoints1.push_back(pMapPoint1);
				gMapPoints2.push_back(pMapPoint2);
				break;
			}
		}
	}

	return gMapPoints1.size() >=6;
}