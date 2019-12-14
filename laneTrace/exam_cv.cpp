
#include <iostream>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/device/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "stop_when_accident.h"

#define PI 3.1415926

using namespace std;
using namespace cv;

extern "C" {

signed short OpenCV_red_Detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) // detect red stop sign
{

	// printf("reddddddddddddddddddddddddddddddddddddddddddddddddddddddd\n");
	int range_count = 0;
	int stopornot = 1;
	Mat img_input, img_gray, img_hsv, img_result;
	signed short speed = 1; //[TODO]basic speed you can edit this value!
	// Scalar blue(10, 200, 50);
	Scalar red(0, 0, 255); //red definition
	Scalar black(0, 0, 0);
	Mat rgb_color, hsv_color;
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

	Coloring(rgb_color, red); //red coloring

	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0]; //we will use only hue value
	//int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	//int value = (int)hsv_color.at<Vec3b>(0, 0)[2];
	int low_hue = hue;//make limit
	int high_hue = hue + 210;//make limit
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;
	//printf("hue = %d, low_hue = %d\n", hue, low_hue);

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);//make limit of hue value

	img_input = srcRGB;

	cvtColor(img_input, img_hsv, COLOR_BGR2HSV);

	Mat binary_image;
	Mat img_mask1, img_mask2, test;

	//accept red filter for detect red stop sign
	inRange(img_hsv, Scalar(low_hue1, 100, 0), Scalar(10, 255, 255), img_mask1);
	// inRange(img_input, Scalar(low_hue1, 250, 250), Scalar(high_hue1, 255, 255), test);
	if (range_count == 2) {
		inRange(img_hsv, Scalar(170, 100, 0), Scalar(180, 255, 255), img_mask2);
		img_mask1 |= img_mask2;
	}

	//make contour...
	vector<vector<Point> > contours;
	findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	
	vector<Point> approx;
	
	// for disp test
	// Mat test(ih, iw, CV_8UC3, black);
	// srcRGB = test;
	cvtColor(img_mask1, img_result, COLOR_GRAY2BGR);


	for (size_t i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		// approxPolyDP(Mat(contours[i]), approx, 1, true);

		if (fabs(contourArea(Mat(approx))) > 1200)  // edit responsiveness...
		{
			int size = approx.size();

			//drawing contour
			if (size % 2 == 0) {
				line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_result, approx[k], 3, Scalar(0, 0, 255));
			}
			else {
				line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_result, approx[k], 3, Scalar(0, 0, 255));
			}
			//circle!! stop!!
			if (size >= 4){
				cout << "red_circle" << endl;
				speed = 0;
			}
		}
	}
	// cvtColor(test, srcRGB, COLOR_BGR2GRAY);
	// srcRGB = img_result;
	// resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

	return speed;
}

int OpenCV_red_Detection_for_traffic_light(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) // detect red stop sign
{

	int range_count = 0;
	int stopornot = 1;
	Mat img_input, img_gray, img_hsv, img_result;
	signed short speed = 1; //[TODO]basic speed you can edit this value!
	// Scalar blue(10, 200, 50);
	Scalar red(0, 0, 255); //red definition
	Scalar black(0, 0, 0);
	Mat rgb_color, hsv_color;
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
	int is_Traffic_Light_red = 0;

	Coloring(rgb_color, red); //red coloring

	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0]; //we will use only hue value
	int low_hue = hue;//make limit
	int high_hue = hue + 210;//make limit
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;
	// printf("hue = %d, low_hue = %d\n", hue, low_hue);

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);//make limit of hue value

	img_input = srcRGB;

	cvtColor(img_input, img_hsv, COLOR_BGR2HSV);

	Mat binary_image;
	Mat img_mask1, img_mask2, test;

	//accept red filter for detect red stop sign
	inRange(img_hsv, Scalar(0, 100, 0), Scalar(10, 255, 255), img_mask1);
	// inRange(img_input, Scalar(low_hue1, 250, 250), Scalar(high_hue1, 255, 255), test);
	if (range_count == 2) {
		inRange(img_hsv, Scalar(170, 100, 0), Scalar(180, 255, 255), img_mask2);
		img_mask1 |= img_mask2;
	}

	//make contour...
	vector<vector<Point> > contours;
	findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	
	vector<Point> approx;
	
	// for disp test
	// Mat test(ih, iw, CV_8UC3, black);
	// srcRGB = test;
	cvtColor(img_mask1, img_result, COLOR_GRAY2BGR);


	for (size_t i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		// approxPolyDP(Mat(contours[i]), approx, 1, true);

		if (fabs(contourArea(Mat(approx))) > 80)  // edit responsiveness...
		{
			int size = approx.size();

			//drawing contour
			if (size % 2 == 0) {
				line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_result, approx[k], 3, Scalar(0, 0, 255));
			}
			else {
				line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_result, approx[k], 3, Scalar(0, 0, 255));
			}
			//circle!! stop!!
			if (size >= 4){
				// cout << "traffic red_sign" << endl;
				is_Traffic_Light_red = 1;//red!
			}
		}
	}
	// cvtColor(test, srcRGB, COLOR_BGR2GRAY);
	srcRGB = img_result;
	resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

	return is_Traffic_Light_red;
}


int OpenCV_green_Detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) // mechanism is similar with red_detection func...
{

	int range_count = 0;
	Mat img_input, img_gray, img_hsv, img_result;
	Scalar green(0, 200, 50); //define green [TODO] we have to do fine tuning
	Mat rgb_color, hsv_color;
	Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
	Mat dstRGB(nh, nw, CV_8UC3, outBuf);
	int is_Traffic_Light = 0;
	int a, b;
	a = 0;
	b = 0;
	Coloring(rgb_color, green);

	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	//int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	//int value = (int)hsv_color.at<Vec3b>(0, 0)[2];
	int low_hue = hue;//¿¿¿ ¿¿
	int high_hue = hue + 20;
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);

	img_input = srcRGB;

	cvtColor(img_input, img_hsv, COLOR_BGR2HSV);
	// cvtColor(img_input, img_gray, COLOR_BGR2HSV);

	Mat binary_image;
	Mat img_mask1, img_mask2;

	//accept green filter for detect Traffic light
	inRange(img_hsv, Scalar(50, 80, 0), Scalar(130, 255, 150), img_mask1);//70 100 70
	if (range_count == 2) {
		inRange(img_hsv, Scalar(50, 80, 0), Scalar(130, 255, 150), img_mask2);
		img_mask1 |= img_mask2;
	}

	vector<vector<Point> > contours;
	findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	vector<Point> approx;
	cvtColor(img_mask1, img_result, COLOR_GRAY2BGR);
	for (size_t i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		if (fabs(contourArea(Mat(approx))) > 400) //[TODO]we have to do fine tuning 
		{
			int size = approx.size();

			if (size % 2 == 0) {
				line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_result, approx[k], 3, Scalar(0, 0, 255));
			}
			else {
				line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_result, approx[k], 3, Scalar(0, 0, 255));
			}
			for(int i = 0; i < 1000; i++){
				if (size <= 7){//go left!
					// setLabel(img_result, "left!", contours[i]); //¿¿¿
					// is_Traffic_Light = 1; //left signal
					a++; 
				}
				else if (size >= 8){//circle, go right!!
					// setLabel(img_result, "circle!!", contours[i]); //¿
					// is_Traffic_Light = 2; //right signal
					b++;
				}
			}
			if(a >= b) {
				is_Traffic_Light = 1;
				cout << "traffic left" << endl;
			}
			else{
				is_Traffic_Light = 2;
				cout << "traffic circle" << endl;
			}
		}
	}  
	srcRGB = img_result;
	resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

	return is_Traffic_Light;
}


/**
  * @brief  Merge two source images of the same size into the output buffer.
  * @param  src1: pointer to parameter of rgb32 image buffer
             src2: pointer to parameter of bgr32 image buffer
             dst : pointer to parameter of rgb32 output buffer
             w : width of src and dst buffer
             h : height of src and dst buffer
  * @retval none
  */
void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dst, int w, int h)
{
    Mat src1AR32(h, w, CV_8UC4, src1);
    Mat src2AR32(h, w, CV_8UC4, src2);
    Mat dstAR32(h, w, CV_8UC4, dst);

    cvtColor(src2AR32, src2AR32, CV_BGRA2RGBA);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double opacity = ((double)(src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + 3])) / 255.;
            for (int c = 0; opacity > 0 && c < src1AR32.channels(); ++c) {
                unsigned char overlayPx = src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + c];
                unsigned char srcPx = src1AR32.data[y * src1AR32.step + x * src1AR32.channels() + c];
                src1AR32.data[y * src1AR32.step + src1AR32.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }

    memcpy(dst, src1AR32.data, w*h*4);
}

}

