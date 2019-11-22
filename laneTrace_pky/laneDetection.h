#ifndef LANEDETECTION_H
#define LANEDETECTION_H

// #include <iostream>
// #include <stdio.h>
// #include <string.h>
// //#include <sys/time.h>

// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/imgproc/imgproc_c.h>


// #include <cmath>
// #include <vector>

// #define PI 3.1415926

// using namespace std;
// using namespace cv;





#ifdef __cplusplus
extern "C" {
#endif

double laneDetection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

#ifdef __cplusplus
}
#endif



#endif //LANEDETECTION_H

