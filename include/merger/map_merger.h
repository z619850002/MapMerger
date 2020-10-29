#ifndef MAP_MERGER_H_
#define MAP_MERGER_H_

#include "../map/map.h"
#include "../keyframe/keyframe.h"
#include "../mappoint/mappoint.h"
#include "../optimizer/sim3_solver.h"
using namespace std;



class MapMerger
{
public:
	MapMerger();
	
	Map * MergeMap(Map * pMap1, Map * pMap2);

	bool MatchMapPoint(MapPoint * pMapPoint1, MapPoint * pMapPoint2);

	bool MatchKeyFrame(	KeyFrame * pKeyFrame1, KeyFrame * pKeyFrame2,
						vector<MapPoint *> & gMapPoints1,
						vector<MapPoint *> & gMapPoints2);


	bool MatchMap(		Map * pMap1, Map * pMap2,
						vector<MapPoint *> & gMapPoints1,
						vector<MapPoint *> & gMapPoints2);

};


inline bool MapMerger::MatchMapPoint(MapPoint * pMapPoint1, MapPoint * pMapPoint2){
	return pMapPoint1->GetRealPoint() == pMapPoint2->GetRealPoint();
}


#endif