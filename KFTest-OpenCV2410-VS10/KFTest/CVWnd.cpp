// CVWnd.cpp : implementation file
//

#include "stdafx.h"
#include "KFTest.h"
#include "CVWnd.h"

#include "KFTestDoc.h"
#include "KFTestView.h"
extern CKFTestView *pv;

// CVWnd

// plot points
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ),                \
Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ),                \
Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )


struct mouse_info_struct mouse_info = {-1,-1}, last_mouse;
vector<Point> mousev,kalmanv;

//KalmanFilter (int dynamParams, int measureParams, int controlParams=0, int type=CV_32F)
// the full constructor taking the dimensionality of the state, of the measurement and of the control vector
//KalmanFilter KF(4, 2, 0); 
//KalmanFilter KF(6, 2, 0); 

Mat_<float> measurement(2,1); 
//vector<Point> mousev,kalmanv;

IMPLEMENT_DYNAMIC(CVWnd, CWnd)

CVWnd::CVWnd()
{
	bRunPrev = FALSE;

	gcnt = 0;
	vc=0.0f ;
}

CVWnd::~CVWnd()
{
}


BEGIN_MESSAGE_MAP(CVWnd, CWnd)
	ON_WM_ERASEBKGND()
	ON_WM_TIMER()
	ON_WM_DESTROY()
END_MESSAGE_MAP()


void CVWnd::Create( CWnd *p, CRect rc)
{
	CWnd::Create( NULL,"",WS_CHILD|WS_VISIBLE,CRect(rc.TopLeft().x,rc.TopLeft().y,rc.Width(),rc.Height()),p,0);
	
	RedirectIOToConsole();

	Init();

	SetTimer(0,10,NULL);

}

void CVWnd::InitKF(BOOL bForceResetData, float vc_, BOOL bCVModel)
{
	img.setTo(cv::Scalar(0,0,0));

	vc = vc_;

	if(bCVModel) // Constant velocity assumption 
	{
		// Useful to model smooth target motion
		// State prediction:  Linear target motion

		KF = KalmanFilter(4, 2, 0); 
		

		KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,vc,0,   
													 0,1,0,vc,  
													 0,0,1,0,  
													 0,0,0,1);
	}
	else	// Const acc. assumption
	{
		// Useful to model target motion that is smooth in position and velocity changes 
		KF = KalmanFilter(6, 2, 0); 
		KF.transitionMatrix = *(Mat_<float>(6, 6) << 1,0,vc,0,0.5*vc*vc, 0,
													 0,1,0,vc,0,  0.5*vc*vc,
													 0,0,1,0,vc,0,  
													 0,0,0,1,0,vc,
													 0,0,0,0,1,0,
													 0,0,0,0,0,1 );
	}
	//KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);	//Roy

	measurement.setTo(Scalar(0));
	/*
	KF.statePre.at<float>(0) = pt.x;
	KF.statePre.at<float>(1) = pt.y;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	*/
	
	KF.statePost.at<float>(0) = pt.x;
	KF.statePost.at<float>(1) = pt.y;
	KF.statePost.at<float>(2) = 0;
	KF.statePost.at<float>(3) = 0;
	
	if(!bCVModel)
	{
		KF.statePost.at<float>(4) = 0;
		KF.statePost.at<float>(5) = 0;
	}
	/*
	KF.statePre.at(0) = mousePos.x;
	KF.statePre.at(1) = mousePos.y;

	should actually be

	KF.statePost.at(0) = mousePos.x;
	KF.statePost.at(1) = mousePos.y;

	as Kalman.predict() calculates statePre = TransitionMatrix * statePost;

	// See http://opencvexamples.blogspot.com/2014/01/kalman-filter-implementation-tracking.html
*/

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.1));

	// Image to show mouse tracking
	Mat img(480, 640, CV_8UC3);
				
	if(bForceResetData)
	{
		mousev.clear();
		kalmanv.clear();
	}

	gcnt = 0;
}

void CVWnd::Init()
{
	string str;

	img = Mat::zeros(480,640, CV_8UC3);

	if ( img.empty() ) 
    { 
	        cout << "Error loading the image" << endl;

        return;
    }
	else
	{
		printf("loaded an image %dx%d\n", img.rows, img.cols);

	}
}
// CVWnd message handlers


BOOL CVWnd::OnEraseBkgnd(CDC* pDC)
{
	CString buf;
	buf.Format("%d %d", pt.x, pt.y);

	/*
	pDC->Rectangle(CRect(0,0,640,480));
	pDC->TextOutA(10,10,buf);
	*/
	if ( img.empty() )  return 1;

	IplImage copy = img;
	cimg.CopyOf(&copy);
	cimg.DrawToHDC(pDC->GetSafeHdc(), CRect(0,0,640,480));


	return 1;
	//return CWnd::OnEraseBkgnd(pDC);
}


void CVWnd::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	//if(bRun)
	BOOL bRun = pv->m_chk_run.GetCheck();

	if(bRun)
	if(nIDEvent==0)
	{
		if(bRun != bRunPrev)
		{
			pv->m_wnd1.SetFocus();
		}

		GetCursorPos(&pt);
		ScreenToClient(&pt);

		mouse_info.x = pt.x;
		mouse_info.y = pt.y;

		if(pt.x>=0 && pt.x<640 && pt.y >=0 && pt.y<480)
		{
			BOOL bCVModel = pv->m_chk_model_cv.GetCheck();
			float v1 = pv->m_sld_vel.GetPos() * 0.1;
			if(gcnt==0)
			{
				InitKF(TRUE, v1, bCVModel);
				pv->m_btn_clear.EnableWindow(TRUE);
			}
			else
			{
				if(v1 != vc)
				{
					CString buf1; 
					buf1.Format("%.2f", v1);
					pv->m_info1.SetWindowTextA(buf1);
					InitKF(FALSE, v1, bCVModel);
				}

				img.setTo(cv::Scalar(0,0,0));

				// First predict, to update the internal statePre variable
				Mat prediction = KF.predict();
				Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
              
				// Get mouse point
				measurement(0) = pt.x;
				measurement(1) = pt.y; 

				 // The update phase 
				Mat estimated = KF.correct(measurement);

				Point statePt(estimated.at<float>(0),estimated.at<float>(1));
				Point measPt(measurement(0),measurement(1));

				mousev.push_back(measPt);
				kalmanv.push_back(statePt);
				drawCross( statePt, Scalar(255,255,255), 5 );
				drawCross( measPt, Scalar(0,0,255), 5 );

				int i;
				for ( i = 0; i < mousev.size()-1; i++) 
					line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);
     
				for (i = 0; i < kalmanv.size()-1; i++) 
					line(img, kalmanv[i], kalmanv[i+1], Scalar(0,255,255), 1);

				char str1[255];

				// Test 
				//Mat gain = KF.gain;
				//KF.gain.rows = 4;
				//KF.gain.cols = 2;
				//sprintf(str1, "%.3f,%.3f,%.3f,%.3f\n%.3f,%.3f,%.3f,%.3f\n", 
				//			KF.gain.at<float>(0,0), KF.gain.at<float>(0,1), KF.gain.at<float>(1,0), KF.gain.at<float>(1,1),
				//			KF.gain.at<float>(2,0), KF.gain.at<float>(2,1), KF.gain.at<float>(3,0), KF.gain.at<float>(3,1));
					

				// for debugging the observed state
				
				if(bCVModel)
				{
					sprintf(str1, "%.3f\t%.3f\t%.3f\t%.3f\n",	estimated.at<float>(0), estimated.at<float>(1), 
																estimated.at<float>(2), estimated.at<float>(3));
					printf(str1);

					sprintf(str1, "%.3f\t%.3f\n", estimated.at<float>(2), estimated.at<float>(3));
				}
				else
				{
					sprintf(str1, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",	estimated.at<float>(0), estimated.at<float>(1), 
																			estimated.at<float>(2), estimated.at<float>(3),
																			estimated.at<float>(4), estimated.at<float>(5));
					printf(str1);

					sprintf(str1, "%.2f\t%.2f\t%.2f\t%.2f\n", estimated.at<float>(2), estimated.at<float>(3), estimated.at<float>(4), estimated.at<float>(5));
				}
				pv->m_info2.SetWindowTextA(str1);


			}
			gcnt++;
		}
		Invalidate();
	}

	bRunPrev = bRun;
	CWnd::OnTimer(nIDEvent);
}


void CVWnd::OnDestroy()
{
	CWnd::OnDestroy();

	// TODO: Add your message handler code here
}
