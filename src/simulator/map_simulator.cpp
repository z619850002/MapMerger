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


vector<MapPoint *> MapSimulator::SimulateMapPoints(MapPointsRange iMapPointRange){
	double nSigma = 0.1;
	cv::RNG iRNG(12345);
	vector<MapPoint *> gMapPoints;
	gMapPoints.reserve(this->m_sRealPoints.size());
	for (RealPoint * pRealPoint : this->m_sRealPoints){
		cv::Point3d iPosition = pRealPoint->GetPosition();
		double nX = iPosition.x;
		double nY = iPosition.y;
		double nZ = iPosition.z;
		if (nX >= this->m_iMapSimulatorRange.m_nMinX && 
			nX <= this->m_iMapSimulatorRange.m_nMaxX &&
			nY >= this->m_iMapSimulatorRange.m_nMinY &&
			nY <= this->m_iMapSimulatorRange.m_nMaxY &&
			nZ >= this->m_iMapSimulatorRange.m_nMinY &&
			nZ <= this->m_iMapSimulatorRange.m_nMaxY){

			MapPoint * pNewMapPoint = new MapPoint(pRealPoint);
			cv::Point3d iNewPosition(	nX + iRNG.gaussian(nSigma),
										nY + iRNG.gaussian(nSigma),
										nZ + iRNG.gaussian(nSigma));
			pNewMapPoint->SetPosition(iNewPosition);
			gMapPoints.push_back(pNewMapPoint);
		}

	}

	return gMapPoints;
}


vector<RealPoint *> MapSimulator::SimulateRealPoints(int nNumber){
	vector<RealPoint *> gRealPoints;
	//Random number generator.
	cv::RNG iRNG(12345);
	for (int i = 0; i < nNumber; i++) 
	{
        double nX = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinX, 
        							this->m_iMapSimulatorRange.m_nMaxX);

        double nY = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinY, 
        							this->m_iMapSimulatorRange.m_nMaxY);

        double nZ = iRNG.uniform(	this->m_iMapSimulatorRange.m_nMinZ, 
        							this->m_iMapSimulatorRange.m_nMaxZ);

        RealPoint * pNewRealPoint = new RealPoint(cv::Point3d(nX, nY, nZ));
        
        gRealPoints.push_back(pNewRealPoint);
	}
	this->m_sRealPoints = set<RealPoint *>(gRealPoints.begin(), gRealPoints.end()); 

	return gRealPoints;
}



vector<Sophus::SE3> MapSimulator::SimulateTrajectory(Sophus::SE3 iStartPose, Sophus::SE3 iEndPose, int nNumber){
	//Random number generator
	cv::RNG iRNG(12345);
	double nSigma = 0.000001;

	vector<Sophus::SE3> gPoses;
	gPoses.reserve(nNumber+2);
	//Sampled all poses
	gPoses.push_back(iStartPose);
	for (int i=1;i<=nNumber;i++){
		double nRatio = 1.0/(double)nNumber * (double)i;
		Sophus::SE3 iSampledPose = TrajectoryInterpolate(
			iStartPose, 
			iEndPose, 
			nRatio);
		
		//Add noise to the smooth trajectory.
		double nNoiseX = iRNG.gaussian(nSigma);
		double nNoiseY = iRNG.gaussian(nSigma);
		double nNoiseZ = iRNG.gaussian(nSigma);
		double nNoiseR1 = iRNG.gaussian(nSigma);
		double nNoiseR2 = iRNG.gaussian(nSigma);
		double nNoiseR3 = iRNG.gaussian(nSigma);

		Eigen::Matrix<double, 6 , 1> mNoiseVec(6);
		mNoiseVec << nNoiseX, nNoiseY, nNoiseZ, nNoiseR1, nNoiseR2, nNoiseR3;

		iStartPose = Sophus::SE3::exp(iStartPose.log() + mNoiseVec);

		gPoses.push_back(iSampledPose);


	}
	gPoses.push_back(iEndPose);

	
	return gPoses;
}