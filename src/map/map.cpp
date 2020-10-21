#include "../../include/map/map.h"

using namespace std;


unsigned int Map::m_nCountID = 0;

Map::Map(){
	this->m_nId = Map::m_nCountID++;
}


void Map::UpdateCovisibleGraph(){
	set<KeyFrame *>::iterator pIter, pIter2;
	for (	pIter = m_sKeyFrames.begin();
			pIter != m_sKeyFrames.end();
			pIter++){
		for (	pIter2 = pIter;
			pIter2 != m_sKeyFrames.end();
			pIter2++){
			if (pIter2 == pIter){
				continue;
			}
			int nCovisibilities = (*pIter)->CheckCovisibility(*pIter2);
			(*pIter)->AddCovisibleEdge(*pIter2, nCovisibilities);
			(*pIter2)->AddCovisibleEdge(*pIter, nCovisibilities);
		}	
	}
}