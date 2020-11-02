#include "../include/simulator/map_simulator.h"
#include "../include/plotter/map_plotter.h"
#include "../include/merger/map_merger.h"
#include "../include/optimizer/sim3_solver.h"
#include "../include/optimizer/optimizer.h"
using namespace std;


int main(){
	MapSimulator * pSimulator = new MapSimulator();

	pSimulator->SimulateScene();

	vector<Map *> gMaps = pSimulator->GetMaps();
	Map * pMap = gMaps[0];
	Map * pMap2 = gMaps[1];






	pMap->AddNoise(0.04, 0.04);
	pMap2->AddNoise(0.04, 0.04);


	pMap2->SetScale(1.5);
	pMap->Localize();
	pMap2->Localize();



	SingleMapOptimizer * pOptimizer = new SingleMapOptimizer(pMap);
    pOptimizer->Optimize(100);

    pOptimizer->SetMap(pMap2);
    pOptimizer->Optimize(100);

	

	MapMerger * pMerger = new MapMerger();

	pMerger->MergeMap(pMap, pMap2);



	vector<Sophus::SE3> gGroundTruth = pSimulator->GetGroundTruth(pMap->GetKeyFrames());

	vector<double> gErrors1, gErrors2;
	double nError1 = pMap->ComputeATE(gGroundTruth, gErrors1);

	
    pOptimizer->SetMap(pMap);
    pOptimizer->OptimizeCascade(100);	

    double nError2 = pMap->ComputeATE(gGroundTruth, gErrors2);

	cout << "Error 1 is: " << nError1 << endl;

	cout << "Error 2 is: " << nError2 << endl;


	for (int i=0;i<gErrors2.size();i++){
		gErrors1[i] = gErrors1[i] - gErrors2[i];
		cout << "Error is: " << gErrors1[i] << endl;
	}
    

	MapPlotter * pPlotter = new MapPlotter(pMap);

	map<int , double> dErrors;
	
	for (int i=0;i<pMap->GetKeyFrames().size();i++){
		KeyFrame * pKeyFrame = pMap->GetKeyFrames()[i];
		dErrors[pKeyFrame->GetId()] = gErrors1[i];
	}

	pPlotter->SetError(dErrors);

    pPlotter->Run("MapMerger2: Map Viewer");

	return 0;
}