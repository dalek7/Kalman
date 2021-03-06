#pragma once

// for testing Kalman Filter implemented in OpenCV
// Seung-Chan Kim
// 2014/7/23

// CVWnd
#include "opencv2/opencv.hpp"
//#pragma comment (lib, "cv.lib")
#pragma comment (lib, "opencv_core2410.lib")
#pragma comment (lib, "opencv_flann2410.lib")
#pragma comment (lib, "opencv_highgui2410.lib")
#pragma comment (lib, "opencv_video2410.lib")

using namespace std;
using namespace cv;

#include "../lib/CvvImage.h"

struct mouse_info_struct { int x,y; };

class CVWnd : public CWnd
{
	DECLARE_DYNAMIC(CVWnd)

public:
	CVWnd();
	virtual ~CVWnd();


public:
	
	virtual void Create( CWnd *, CRect rc=CRect(0,0,640,480));
	BOOL bRunPrev;
	
	CPoint pt;

	KalmanFilter KF;//(6, 2, 0); 

	void InitKF(BOOL bForceResetData=TRUE, float dt_ = 1, BOOL bCVModel=TRUE); // const velocity model
	void Init();
	
	Mat img;
	CvvImage cimg;


	int gcnt;
	float dt;
	//KALMAN
public:
	
	

protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnDestroy();
};


