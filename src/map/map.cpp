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
			if (nCovisibilities > 0){
				(*pIter)->AddCovisibleEdge(*pIter2, nCovisibilities);
				(*pIter2)->AddCovisibleEdge(*pIter, nCovisibilities);	
			}
		}	
	}
}



void Map::Localize(){
	KeyFrame * pFirstKeyFrame = this->m_gKeyFrames[0];
	this->m_mLocalPose = pFirstKeyFrame->GetPose();
	for (KeyFrame * pKeyFrame : this->m_gKeyFrames){
		pKeyFrame->SetPose(pKeyFrame->GetPose() * this->m_mLocalPose.inverse());
	}

	for (MapPoint * pMapPoint : this->m_sMapPoints){
		cv::Point3d iPosition = pMapPoint->GetPosition();
		Eigen::Vector4d mPosition;
		mPosition << iPosition.x,iPosition.y, iPosition.z, 1.0;
		mPosition = this->m_mLocalPose * mPosition;
		pMapPoint->SetPosition(cv::Point3d(mPosition[0], mPosition[1], mPosition[2]));
	}
}