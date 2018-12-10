//
//  main.cpp
//  KalmanTracking
//
//  Created by Seung-Chan Kim on 10/12/2018.
//  Copyright © 2018 Seung-Chan Kim. All rights reserved.
//  An example using the standard Kalman filter
//  https://docs.opencv.org/3.4/de/d70/samples_2cpp_2kalman_8cpp-example.html

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

static inline Point calcPoint(Point2f center, double R, double angle)
{
    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}

static void DbgMat(Mat m)
{
    for(int i = 0; i <m.rows; i++)
    {
        for(int j = 0; j <m.cols; j++)
            printf("%.2f\t", m.at<float>(i,j));//m.cols*
        
        printf("\n");
    }
}

#define drawCross( center, color, d )                                        \
line( img, Point( center.x - d, center.y - d ),                          \
Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
line( img, Point( center.x + d, center.y - d ),                          \
Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )

static void help()
{
    printf( "\nExample of c calls to OpenCV's Kalman filter.\n"
           "   Tracking of rotating point.\n"
           "   Rotation speed is constant.\n"
           "   Both state and measurements vectors are 1D (a point angle),\n"
           "   Measurement is the real point angle + gaussian noise.\n"
           "   The real and the estimated points are connected with yellow line segment,\n"
           "   the real and the measured points are connected with red line segment.\n"
           "   (if Kalman filter works correctly,\n"
           "    the yellow segment should be shorter than the red one).\n"
           "\n"
           "   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
           "   Pressing ESC will stop the program.\n"
           );
}


int main(int, char**)
{
    help();
    Mat img(500, 500, CV_8UC3);
    KalmanFilter KF(2, 1, 0);
    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
    Mat processNoise(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    char code = (char)-1;
    for(;;)
    {
        randn( state, Scalar::all(0), Scalar::all(0.1) );
        KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
        setIdentity(KF.errorCovPost, Scalar::all(1));
        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
        
        cout << "KF.transitionMatrix" <<endl;
        cout << KF.transitionMatrix <<endl;
        
        cout << "KF.measurementMatrix" <<endl;
        cout << KF.measurementMatrix <<endl;
        
        cout << "KF.processNoiseCov" <<endl;
        cout << KF.processNoiseCov <<endl;
        
        cout << "KF.errorCovPost" <<endl;
        cout << KF.errorCovPost <<endl;
        
        for(;;)
        {
            Point2f center(img.cols*0.5f, img.rows*0.5f);
            float R = img.cols/3.f;
            double stateAngle = state.at<float>(0);
            Point statePt = calcPoint(center, R, stateAngle);
            Mat prediction = KF.predict();
            double predictAngle = prediction.at<float>(0);
            Point predictPt = calcPoint(center, R, predictAngle);
            randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
            // generate measurement
            measurement += KF.measurementMatrix*state;
            double measAngle = measurement.at<float>(0);
            Point measPt = calcPoint(center, R, measAngle);
            
            // plot points
            img = Scalar::all(0);
            drawCross( statePt, Scalar(255,255,255), 5 );
            drawCross( measPt, Scalar(0,0,255), 5 );
            drawCross( predictPt, Scalar(0,255,0), 5 );
            line( img, statePt, measPt, Scalar(0,0,255), 1, LINE_AA, 0 );
            line( img, statePt, predictPt, Scalar(0,255,255), 1, LINE_AA, 0 );
            if(theRNG().uniform(0,4) != 0)
                KF.correct(measurement);
            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
            
            state = KF.transitionMatrix*state + processNoise;
            
            int fontFace = 0;//FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontScale = 0.4;
            int thickness = 1;
            
            putText(img, "statePt", Point(50+30,50), fontFace, fontScale, Scalar(255,255,255), thickness, 1);
            putText(img, "measPt", Point(50+30,50+20*1), fontFace, fontScale, Scalar(0,0,255), thickness, 1);
            putText(img, "predictPt", Point(50+30,50+20*2), fontFace, fontScale, Scalar(0,255,0), thickness, 1);
            
            drawCross( Point(50,50), Scalar(255,255,255), 5 ); // statePt
            drawCross( Point(50,50+20*1), Scalar(0,0,255), 5 ); //measPt
            drawCross( Point(50,50+20*2), Scalar(0,255,0), 5 ); //predictPt
            
            imshow( "Kalman", img );
            code = (char)waitKey(100);
            if( code > 0 )
                break;
        }
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }
    return 0;
}


/*
 KF.transitionMatrix
 [1, 1;
 0, 1]
 
 KF.measurementMatrix
 [1, 0]
 
 KF.processNoiseCov
 [9.9999997e-06, 0;
 0, 9.9999997e-06]
 
 KF.errorCovPost
 [1, 0;
 0, 1]
 */
