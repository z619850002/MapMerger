#include "../include/plotter/map_simulator_plotter.h"

using namespace std;



void initializePose(Sophus::SE3& T_FG,Sophus::SE3& T_LG,
					Sophus::SE3& T_BG,Sophus::SE3& T_RG)
{
	double nScale = 4;


	// Initialize T_FG
	Eigen::Matrix3d R_FG = Eigen::Matrix3d();
	R_FG<< 9.99277118e-01,  3.82390286e-04, -3.80143958e-02,
		  -2.30748265e-02, -7.88582447e-01, -6.14495953e-01,
	      -3.02124625e-02,  6.14928921e-01, -7.88003572e-01;




	Eigen::Vector3d t_FG;
	t_FG<< 6.75437418e-01,  2.50896883e+01,  3.17779305e+00;
	T_FG = Sophus::SE3(R_FG,t_FG/nScale);



	
	// Initialize T_LG
	Eigen::Matrix3d R_LG = Eigen::Matrix3d();
	R_LG<<-1.21898860e-02,  9.99924056e-01, -1.81349393e-03,
		   8.02363600e-01,  8.69913885e-03, -5.96772133e-01,
	      -5.96711036e-01, -8.72966581e-03, -8.02408707e-01;
	Eigen::Vector3d t_LG;
	t_LG<< 1.36392943e+00,  1.60942881e+01,  1.04105913e+01;
	T_LG = Sophus::SE3(R_LG,t_LG/nScale);
	
	// Initialize T_BG
	Eigen::Matrix3d R_BG = Eigen::Matrix3d();
	R_BG<<-9.99615699e-01,  1.56439861e-02, -2.28849354e-02,
		   2.59906371e-02,  8.16008735e-01, -5.77454960e-01,
	       9.64060983e-03, -5.77827838e-01, -8.16101739e-01;
	Eigen::Vector3d t_BG;
	t_BG<< 1.09266953e+00,  2.46308124e+01,  6.60957845e+00;
	T_BG = Sophus::SE3(R_BG,t_BG/nScale);

	// Initialize T_RG
	Eigen::Matrix3d R_RG = Eigen::Matrix3d();
	R_RG<< 4.57647596e-03, -9.99989102e-01,  9.22798184e-04,
		  -6.26343448e-01, -3.58584197e-03, -7.79538984e-01,
	       7.79533797e-01,  2.98955282e-03, -6.26353033e-01;
	Eigen::Vector3d t_RG;
	t_RG<<-1.66115120e-01,  1.76226207e+01, 6.08338205e+00;
	T_RG = Sophus::SE3(R_RG,t_RG/nScale);
	
	return;
}



Sophus::SE3 TrajectoryInterpolate(
	Sophus::SE3 iStartPose, 
	Sophus::SE3 iEndPose, 
	double nRatio){
	Sophus::SE3 iRelativePose = iStartPose.inverse() * iEndPose;
	Eigen::Matrix<double,6,1> mRelativePose = iRelativePose.log();

	Eigen::Matrix<double,6,1> mInterpolation = mRelativePose * nRatio;

	return iStartPose * Sophus::SE3::exp(mInterpolation);
}

int main2(){
	Sophus::SE3 mTF, mTL, mTB, mTR;
	initializePose(mTF, mTL, mTB, mTR);

	KeyFrame * pKeyFrameFront = new KeyFrame();
	KeyFrame * pKeyFrameLeft = new KeyFrame();
	KeyFrame * pKeyFrameBack = new KeyFrame();
	KeyFrame * pKeyFrameRight = new KeyFrame();

	pKeyFrameFront->SetPose(mTF.matrix());
	pKeyFrameLeft->SetPose(mTL.matrix());
	pKeyFrameBack->SetPose(mTB.matrix());
	pKeyFrameRight->SetPose(mTR.matrix());

	Plotter * pPlotter = new Plotter();
	pPlotter->AddKeyFrame(pKeyFrameFront);

	for (int i=0;i<10;i++){
		Sophus::SE3 mTMiddle = TrajectoryInterpolate(mTF, mTL, (double)(1.0/10.0*(double)(i+1)));
		KeyFrame * pKeyFrameMiddle = new KeyFrame();
		pKeyFrameMiddle->SetPose(mTMiddle.matrix());
		pPlotter->AddKeyFrame(pKeyFrameMiddle);
	}

	pPlotter->AddKeyFrame(pKeyFrameLeft);


	for (int i=0;i<10;i++){
		Sophus::SE3 mTMiddle = TrajectoryInterpolate(mTL, mTB, (double)(1.0/10.0*(double)(i+1)));
		KeyFrame * pKeyFrameMiddle = new KeyFrame();
		pKeyFrameMiddle->SetPose(mTMiddle.matrix());
		pPlotter->AddKeyFrame(pKeyFrameMiddle);
	}

	pPlotter->AddKeyFrame(pKeyFrameBack);

	for (int i=0;i<10;i++){
		Sophus::SE3 mTMiddle = TrajectoryInterpolate(mTB, mTR, (double)(1.0/10.0*(double)(i+1)));
		KeyFrame * pKeyFrameMiddle = new KeyFrame();
		pKeyFrameMiddle->SetPose(mTMiddle.matrix());
		pPlotter->AddKeyFrame(pKeyFrameMiddle);
	}

	pPlotter->AddKeyFrame(pKeyFrameRight);


	for (int i=0;i<10;i++){
		Sophus::SE3 mTMiddle = TrajectoryInterpolate(mTR, mTF, (double)(1.0/10.0*(double)(i+1)));
		KeyFrame * pKeyFrameMiddle = new KeyFrame();
		pKeyFrameMiddle->SetPose(mTMiddle.matrix());
		pPlotter->AddKeyFrame(pKeyFrameMiddle);
	}




	pPlotter->Run();

	return 0;
}




int main(){
	Sophus::SE3 mTF, mTL, mTB, mTR;
	initializePose(mTF, mTL, mTB, mTR);

	vector<Sophus::SE3> gPoses;
	vector<MapPoint *> gMapPoints;

	MapSimulator * pSimulator = new MapSimulator();
	gPoses = pSimulator->SimulateTrajectory(mTF, mTB, 10);
	gMapPoints = pSimulator->SimulateMapPoints(1000);



	Plotter * pPlotter = new Plotter();
	
	for (int i=0;i<gPoses.size();i++){
		Sophus::SE3 iPose = gPoses[i];
		KeyFrame * pKeyFrame = new KeyFrame();
		pKeyFrame->SetPose(iPose.matrix());
		pPlotter->AddKeyFrame(pKeyFrame);
	}

	for (auto item : gMapPoints){
		pPlotter->AddMapPoints(item);
	}

	pPlotter->Run();

	return 0;
}