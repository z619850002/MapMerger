#include "../../include/keyframe/keyframe.h"
#include "../../include/mappoint/mappoint.h"
#include "../../include/simulator/map_simulator.h"

using namespace std;

MapSimulator::MapSimulator(){
	this->m_iMapSimulatorRange = MapPointsRange(-10 , -10 , -10 , 10 , 10 , 10);
}


bool MapSimulator::SwitchMap(int nMapId){
	if (m_dMapAndId.count(nMapId) != 0){
		Map * pSwitchedMap = this->m_dMapAndId[nMapId];
		this->m_pCurrentMap = pSwitchedMap;
		return true;	
	}
	return false;
}

