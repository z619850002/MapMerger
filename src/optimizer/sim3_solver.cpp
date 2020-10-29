#include "../../include/optimizer/sim3_solver.h"


using namespace std;



Sim3Solver::Sim3Solver(vector<MapPoint *> gMapPoints1, vector<MapPoint *> gMapPoints2, bool bFixScale):
    m_nIterations(0), m_bFixScale(bFixScale)
{
    
	this->m_gMapPoints1 = gMapPoints1;
	this->m_gMapPoints2 = gMapPoints2;


}

void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;
    }
}

void Sim3Solver::ComputeSim3()
{

	int nPointsNum = this->m_gMapPoints1.size();
	if (this->m_gMapPoints2.size() != nPointsNum){
		cerr << "Map point sizes are not matched when computing sim3" << endl;
	}
	cv::Mat P1(3,nPointsNum,CV_32F);
	cv::Mat P2(3,nPointsNum,CV_32F);
	

	for (int i=0;i<nPointsNum;i++){
		cv::Point3d iPoint1 = this->m_gMapPoints1[i]->GetPosition();
		cv::Point3d iPoint2 = this->m_gMapPoints2[i]->GetPosition();

		cv::Mat mPosition1 =  (cv::Mat_<float>(3 , 1) << iPoint1.x, iPoint1.y, iPoint1.z);
		cv::Mat mPosition2 =  (cv::Mat_<float>(3 , 1) << iPoint2.x, iPoint2.y, iPoint2.z);

		mPosition1.copyTo(P1.col(i));
		mPosition2.copyTo(P2.col(i));
	}




    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

	//Pr is the relative position
	//P1, P2 are all 3*n, Pr1, Pr2 are also 3*n
    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    //O is just the centroid
    //O is 3*1
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix
    // M is 3*3
    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);



    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation





    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)



    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(norm(vec),evec.at<float>(0,0));



    if (norm(vec) != 0.0){
   		vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half 	
    }
    


    this->m_mRotation.create(3,3,P1.type());

    cv::Rodrigues(vec,this->m_mRotation); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2

    cv::Mat P3 = this->m_mRotation*Pr2;

    // Step 6: Scale

    if(!this->m_bFixScale)
    {
        double nom = Pr1.dot(P3);

        cv::Mat aux_P3;
        cv::pow(P3,2,aux_P3);
        // (P3.size(),P3.type());
        // aux_P3=P3;
        // cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        this->m_nScale = nom/den;
        cout << "Scale is: " << m_nScale << endl;
    }
    else
        this->m_nScale = 1.0f;

    // Step 7: Translation

    this->m_mTranslation.create(1,3,P1.type());
    this->m_mTranslation = O1 - m_nScale*m_mRotation*O2;

    // Step 8: Transformation

    // // Step 8.1 T12
    // mT12i = cv::Mat::eye(4,4,P1.type());

    // cv::Mat sR = ms12i*mR12i;

    // sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    // mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    
}


Eigen::MatrixXd Sim3Solver::GetEigenSim3Matrix(){
	Eigen::MatrixXd mSim3Eigen;
	cv::Mat mSim3CV = this->GetCVSim3Matrix();
	cv::cv2eigen(mSim3CV, mSim3Eigen);
	return mSim3Eigen;
}




cv::Mat Sim3Solver::GetCVSim3Matrix(){
  	cv::Mat mSim3 = cv::Mat::eye(4,4,this->m_mRotation.type());

    cv::Mat mScaledRotation = m_nScale*m_mRotation;

    mScaledRotation.copyTo(mSim3.rowRange(0,3).colRange(0,3));
    this->m_mTranslation.copyTo(mSim3.rowRange(0,3).col(3));

    return mSim3;
}