#include "../include/simulator/map_simulator.h"
#include "../include/plotter/map_plotter.h"
using namespace std;



int main(){
	MapSimulator * pSimulator = new MapSimulator();

	pSimulator->SimulateScene();

	vector<Map *> gMaps = pSimulator->GetMaps();
	Map * pMap = gMaps[0];

	cout << "KeyFrame size: " << pMap->GetKeyFrames().size() << endl;
	cout << "MapPoints size: " << pMap->GetMapPoints().size() << endl;

	Map * pMap2 = gMaps[1];

	cout << "KeyFrame2 size: " << pMap2->GetKeyFrames().size() << endl;
	cout << "MapPoints2 size: " << pMap2->GetMapPoints().size() << endl;

	pMap->SetScale(0.5);

	pMap->Localize();
	pMap2->Localize();

	// MapPlotter * pPlotter = new MapPlotter(pMap);
	// pPlotter->Run();

	pMap2->AddNoise();


	MapPlotter * pPlotter2 = new MapPlotter(pMap2);
	// pPlotter2->AddMap(pMap);
	pPlotter2->Run("MapMerger2: Map Viewer");


	// for (KeyFrame * pKeyFrame : pMap2->GetKeyFrames()){
	// 	cout << "Covisible size: " << pKeyFrame->GetCovisibleKeyFrames().size() << endl;
	// 	cout << "MapPoints size: " << pKeyFrame->GetMapPoints().size() << endl;
	// }

    



	return 0;
}