//
// Created by 박규열 on 24/09/2019.
//

#ifndef LANEDETECTION_LANEDETECTION_H
#define LANEDETECTION_LANEDETECTION_H

#include <iostream>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>


#define PI 3.1415926


#define LIGHT_BEEP        // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE        // to test line trace sensor
#define DISTANCE_SENSOR   // to test distance sensor

#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

#define VPE_OUTPUT_W        320
#define VPE_OUTPUT_H        180

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02


using namespace std;
using namespace cv;




void laneDetection(Mat &srcRGB, int iw, int ih, Mat &dstRGB, int nw, int nh);











#endif //LANEDETECTION_LANEDETECTION_H
