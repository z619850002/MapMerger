#ifndef MAP_SIMULATOR_PLOTTER_H_
#define MAP_SIMULATOR_PLOTTER_H_



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.h>

#include <opencv2/core/core.hpp>

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <iterator>


#include<pangolin/pangolin.h>

#include "../simulator/map_simulator.h"

class Plotter
{
public:
	Plotter(){
    	float fps = 20;
    	this->m_nT = 1e3/fps;


	    this->m_nImageWidth = 640;
	    this->m_nImageHeight = 480;

	    this->m_nViewpointX = 0;
	    this->m_nViewpointY = -0.7;
	    this->m_nViewpointZ = -1.8;
	    this->m_nViewpointF = 500;


	    //Drawer settings
	    this->mKeyFrameSize = 0.5;
	    this->mKeyFrameLineWidth = 1;
	    this->mGraphLineWidth = 0.9;
	    this->mPointSize = 5;
	    this->mCameraSize = 0.08;
	    this->mCameraLineWidth = 3;
	}

	void AddKeyFrame(KeyFrame * pKeyFrame){
		this->m_gKeyFrames.push_back(pKeyFrame);
	}

	void AddMapPoints(MapPoint * pMapPoint){
		this->m_gMapPoints.push_back(pMapPoint);
	}

public:
    

    void Run(){
    	pangolin::CreateWindowAndBind("MapMerger: Map Viewer",1024,768);
	    // 3D Mouse handler requires depth testing to be enabled
	    glEnable(GL_DEPTH_TEST);

	    // Issue specific OpenGl we might need
	    glEnable (GL_BLEND);
	    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	    // pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
	    
    	// Define Camera Render Object (for view / scene browsing)
	    pangolin::OpenGlRenderState s_cam(
	                pangolin::ProjectionMatrix(1024,768,this->m_nViewpointF,this->m_nViewpointF,512,389,0.1,1000),
	                pangolin::ModelViewLookAt(this->m_nViewpointX,this->m_nViewpointY,this->m_nViewpointZ, 0,0,0,0.0,-1.0, 0.0)
	                );

	    // Add named OpenGL viewport to window and provide 3D Handler
	    pangolin::View& d_cam = pangolin::CreateDisplay()
	            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
	            .SetHandler(new pangolin::Handler3D(s_cam));

	    pangolin::OpenGlMatrix Twc;
	    Twc.SetIdentity();

	    // cv::namedWindow("Rookie-SLAM: Current Frame");

	    // bool bFollow = true;
	    // bool bLocalizationMode = false;

	    while(1)
	    {
	        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	        // this->m_pMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

	        //Follow the camera.
	        s_cam.Follow(Twc);
	        
	        d_cam.Activate(s_cam);
	        glClearColor(1.0f,1.0f,1.0f,1.0f);
	        
			       
            // this->m_pMapDrawer->DrawMapPoints();

	        this->DrawMapPoints();
            this->DrawKeyFrames();

	        pangolin::FinishFrame();

	        cv::waitKey(this->m_nT);   
	    }

	}


    void DrawKeyFrames(){
    	const float &w = mKeyFrameSize;
	    const float h = w*0.75;
	    const float z = w*0.6;

    	//Iterate all frames.
    	// cout << "KeyFrame size " << vpKFs.size() << endl;

    	// for (auto item : vpKFs){
    	// 	cv::Mat Twc = ConvertSophusToCV(item->m_sTransform).inv();
    	// 	cout << "Twc is " << endl << Twc << endl;
    	// 	cout << endl << endl ;
    	// }

        for(int i=0; i< this->m_gKeyFrames.size(); i++)
        {

            KeyFrame * pKeyFrame = this->m_gKeyFrames[i];

            Eigen::MatrixXd mTwc = pKeyFrame->GetPose().inverse();

            

            glPushMatrix();
            // cout << "Twc is " << endl << Twc << endl;

	        std::vector<GLfloat > gTwc = {	mTwc(0,0),mTwc(1,0), mTwc(2,0), mTwc(3,0),
				    						mTwc(0,1),mTwc(1,1), mTwc(2,1), mTwc(3,1), 
				    						mTwc(0,2),mTwc(1,2), mTwc(2,2), mTwc(3,2),
				    						mTwc(0,3),mTwc(1,3), mTwc(2,3), mTwc(3,3)};

			
	        glMultMatrixf(gTwc.data());


            glLineWidth(mKeyFrameLineWidth);
            glColor3f(((float)(i%3))/2,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }

    }


    void DrawMapPoints(){
    	//Firstly get all mappoints.
	    glPointSize(mPointSize);
	    glBegin(GL_POINTS);
	    glColor3f(0.0,1.0,0.0);
	    for(size_t i=0, iend=m_gMapPoints.size(); i<iend;i++)
	    {
	        cv::Point3d iPosition = this->m_gMapPoints[i]->GetPosition();
	        glVertex3f(iPosition.x,iPosition.y,iPosition.z);
	    }

	    glEnd();
    }




    // 1/fps in ms
    double m_nT;
    float m_nImageWidth, m_nImageHeight;

    float m_nViewpointX, m_nViewpointY, m_nViewpointZ, m_nViewpointF;

    vector<KeyFrame *> m_gKeyFrames;


    vector<MapPoint *> m_gMapPoints;


    //Drawer setttings.
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

};





#endif