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



	pMap2->SetScale(3);
	pMap->Localize();
	pMap2->Localize();

	pMap->AddNoise();
	pMap2->AddNoise();

	





	MapPlotter * pPlotter2 = new MapPlotter(pMap2);
	// pPlotter2->AddMap(pMap);
	


    SingleMapOptimizer * pOptimizer = new SingleMapOptimizer(pMap2);

    // pOptimizer->Optimize(1000);

    pPlotter2->Run("MapMerger2: Map Viewer");


	return 0;
}