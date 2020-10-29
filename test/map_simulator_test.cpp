#include "../include/simulator/map_simulator.h"
#include "../include/plotter/map_plotter.h"
#include "../include/merger/map_merger.h"
#include "../include/optimizer/sim3_solver.h"
using namespace std;



int main(){
	MapSimulator * pSimulator = new MapSimulator();

	pSimulator->SimulateScene();

	vector<Map *> gMaps = pSimulator->GetMaps();
	Map * pMap = gMaps[0];

	Map * pMap2 = gMaps[1];



	pMap->SetScale(3);

	pMap->Localize();


	pMap2->Localize();

	pMap->AddNoise();
	pMap2->AddNoise();

	



	MapMerger * pMerger = new MapMerger();
	vector<MapPoint *> gMapPoints1, gMapPoints2;
	pMerger->MatchMap(pMap, pMap2, gMapPoints1, gMapPoints2);




	Sim3Solver  * pSolver = new Sim3Solver(gMapPoints1, gMapPoints2);
	pSolver->ComputeSim3();
	Eigen::MatrixXd mSim3 = pSolver->GetEigenSim3Matrix();




	pMap2->Transform(mSim3);




	MapPlotter * pPlotter2 = new MapPlotter(pMap2);
	pPlotter2->AddMap(pMap);
	pPlotter2->Run("MapMerger2: Map Viewer");



    



	return 0;
}