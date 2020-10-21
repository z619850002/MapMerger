#include <iostream>
#include "../include/keyframe/keyframe.h"
#include "../include/mappoint/mappoint.h"
#include "../include/map/map.h"
using namespace std;



cv::Mat mK = (cv::Mat_<double>(3 , 3) << 	100.0, 0.0  , 0.0,
											0.0,   100.0, 0.0,
											0.0,   0.0,   1.0);


int main(){
	cout << "Hello, world!" << endl;
	//All map points.
	MapPoint * pMapPoint = new MapPoint();
	MapPoint * pMapPoint2 = new MapPoint();
	MapPoint * pMapPoint3 = new MapPoint();
	//All those keyframes.
	KeyFrame * pKeyFrame = new KeyFrame();
	KeyFrame * pKeyFrame2 = new KeyFrame();
	KeyFrame * pKeyFrame3 = new KeyFrame();
	//Camera
	Camera * pCamera = new Camera(mK);
	pKeyFrame->SetCamera(pCamera);
	pKeyFrame2->SetCamera(pCamera);
	pKeyFrame3->SetCamera(pCamera);


	cv::Point3d iPosition3d(1 , 0 , 1);
	cv::Point3d iPosition3d2(0 , 1 , 1);
	cv::Point3d iPosition3d3(1 , 1 , 1);
	


	pMapPoint->SetPosition(iPosition3d);
	pMapPoint2->SetPosition(iPosition3d2);
	pMapPoint3->SetPosition(iPosition3d3);



	
	pKeyFrame->AddMapPoint(pMapPoint);
	pKeyFrame->AddMapPoint(pMapPoint2);


	pKeyFrame2->AddMapPoint(pMapPoint3);
	pKeyFrame2->AddMapPoint(pMapPoint2);


	pKeyFrame3->AddMapPoint(pMapPoint);
	pKeyFrame3->AddMapPoint(pMapPoint2);
	pKeyFrame3->AddMapPoint(pMapPoint3);
	

	cout << "Covisibility between keyframe 1 and 2: " << endl;
	cout << pKeyFrame->CheckCovisibility(pKeyFrame2) << endl;


	cout << "Covisibility between keyframe 2 and 3: " << endl;
	cout << pKeyFrame2->CheckCovisibility(pKeyFrame3) << endl;


	cout << "Covisibility between keyframe 1 and 3: " << endl;
	cout << pKeyFrame->CheckCovisibility(pKeyFrame3) << endl;


	Map * pMap = new Map();
	pMap->AddKeyFrame(pKeyFrame);
	pMap->AddKeyFrame(pKeyFrame2);
	pMap->AddKeyFrame(pKeyFrame3);

	pMap->AddMapPoint(pMapPoint);
	pMap->AddMapPoint(pMapPoint2);
	pMap->AddMapPoint(pMapPoint3);

	pMap->UpdateCovisibleGraph();

	return 0;
}