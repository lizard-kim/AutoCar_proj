
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


//-----
#include <vector>
#include "util.h"
#include <string>
const Scalar COLOR_BLUE = Scalar(255, 0, 0);
const Scalar COLOR_RED = Scalar(0, 0, 255);
const Scalar COLOR_GREEN = Scalar(170, 170, 0);


const Vec3b RGB_WHITE_LOWER = Vec3b(100, 100, 180);
const Vec3b RGB_WHITE_UPPER = Vec3b(255, 255, 255);

const Vec3b HSV_YELLOW_LOWER = Vec3b(10, 70, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);

const Vec3b HSV_WHITE_LOWER = Vec3b(80, 0, 180);
const Vec3b HSV_WHITE_UPPER = Vec3b(180, 60, 255);

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);
const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(179, 255, 255);

const Vec3b HSV_GREEN_LOWER = Vec3b(40, 50, 50);
const Vec3b HSV_GREEN_UPPER = Vec3b(100, 255, 255);

const Vec3b HSV_BLACK_LOWER = Vec3b(0, 0, 0);
const Vec3b HSV_BLACK_UPPER = Vec3b(180, 255, 50);

const Vec3b YUV_LOWER = Vec3b(0, 110, 120);
const Vec3b YUV_UPPER = Vec3b(40, 130, 140);

//-----


#define PI 3.1415926

using namespace std;
using namespace cv;

//----
float get_slope(const Point& p1, const Point& p2);



extern "C" {

/**
  * @brief  To load image file to the buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to load
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_load_file(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    srcRGB = imread(file, CV_LOAD_IMAGE_COLOR); // rgb
    //cvtColor(srcRGB, srcRGB, CV_RGB2BGR);

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To convert format from BGR to RGB.
  * @param  inBuf: buffer pointer of BGR image
             w: width value of the buffers
             h : height value of the buffers
             outBuf : buffer pointer of RGB image
  * @retval none
  */
void OpenCV_Bgr2RgbConvert(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
{
    Mat srcRGB(h, w, CV_8UC3, inBuf);
    Mat dstRGB(h, w, CV_8UC3, outBuf);

    cvtColor(srcRGB, dstRGB, CV_BGR2RGB);
}

/**
  * @brief  Detect faces on loaded image and draw circles on the faces of the loaded image.
  * @param  file: pointer for load image file in local path
             outBuf: buffer pointer to draw circles on the detected faces
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_face_detection(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    
    // Load Face cascade (.xml file)
    CascadeClassifier face_cascade;
    face_cascade.load( "haarcascade_frontalface_alt.xml" );
 
    // Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( srcRGB, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
    
    // Draw circles on the detected faces
    for( int i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( srcRGB, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
 
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To bind two images on destination buffer.
  * @param  file1: file path of first image to bind
             file2: file path of second image to bind
             outBuf : destination buffer pointer to bind
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_binding_image(char* file1, char* file2, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file1, CV_LOAD_IMAGE_COLOR);
    Mat srcRGB2 = imread(file2, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cv::resize(srcRGB2, srcRGB2, cv::Size(srcRGB2.cols/1.5, srcRGB2.rows/1.5));
    cv::Point location = cv::Point(280, 220);
    for (int y = std::max(location.y, 0); y < srcRGB.rows; ++y)
    {
        int fY = y - location.y;
        if (fY >= srcRGB2.rows)
            break;
        
        for (int x = std::max(location.x, 0); x < srcRGB.cols; ++x)
        {
            int fX = x - location.x;
            if (fX >= srcRGB2.cols)
            break;
            
            double opacity = ((double)srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + 3]) / 255.;
            for (int c = 0; opacity > 0 && c < srcRGB.channels(); ++c)
            {
                unsigned char overlayPx = srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + c];
                unsigned char srcPx = srcRGB.data[y * srcRGB.step + x * srcRGB.channels() + c];
                srcRGB.data[y * srcRGB.step + srcRGB.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }
 
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  Apply canny edge algorithm and draw it on destination buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to apply canny edge
             nw : width value of destination buffer
             nh : height value of destination buffer
  * @retval none
  */
void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat srcGRAY;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cvtColor(srcRGB, srcGRAY, CV_BGR2GRAY);
    cv::Mat contours;
    cv::Canny(srcGRAY, contours, 125, 350);

    //cv::Mat contoursInv;
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
 
    cvtColor(contours, contours, CV_GRAY2BGR);
    
    cv::resize(contours, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  Detect the hough and draw hough on destination buffer.
  * @param  srcBuf: source pointer to hough transform
             iw: width value of source buffer
             ih : height value of source buffer
             outBuf : destination pointer to hough transform
             nw : width value of destination buffer
             nh : height value of destination buffer
  * @retval none
  */
// signed short OpenCV_red_Detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) // detect red stop sign
// {
//     printf("red_detection running\n");
//     int range_count = 0;
//     int stopornot = 1;
//     Mat img_input, img_gray, img_hsv, img_result;
//     signed short speed_ratio = 1; //[TODO]basic speed you can edit this value!
//     // Scalar blue(10, 200, 50);
//     Scalar red(0, 0, 255); //red definition
//     Scalar black(0, 0, 0);
//     Mat rgb_color, hsv_color;
//     Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
//     Mat dstRGB(nh, nw, CV_8UC3, outBuf);
//
//     Coloring(rgb_color, red); //red coloring
//
//     cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);
//
//     int hue = (int)hsv_color.at<Vec3b>(0, 0)[0]; //we will use only hue value
//     //int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
//     //int value = (int)hsv_color.at<Vec3b>(0, 0)[2];
//     int low_hue = hue;//make limit
//     int high_hue = hue + 210;//make limit
//     int low_hue1 = 0, low_hue2 = 0;
//     int high_hue1 = 0, high_hue2 = 0;
//
//     MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);//make limit of hue value
//
//     img_input = srcRGB;
//
//     cvtColor(img_input, img_hsv, COLOR_BGR2HSV);
//
//     Mat binary_image;
//     Mat img_mask1, img_mask2, test;
//
//     //accept red filter for detect red stop sign
//     inRange(img_hsv, Scalar(low_hue1, 50, 50), Scalar(255, 255, 255), img_mask1);
//     // inRange(img_input, Scalar(low_hue1, 250, 250), Scalar(high_hue1, 255, 255), test);
//     if (range_count == 2) {
//         inRange(img_hsv, Scalar(low_hue2, 50, 50), Scalar(255, 255, 255), img_mask2);
//         img_mask1 |= img_mask2;
//     }
//
//     //make contour...
//     vector<vector<Point> > contours;
//     findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
//
//     vector<Point> approx;
//
//     // for disp test
//     // Mat test(ih, iw, CV_8UC3, black);
//     // srcRGB = test;
//     cvtColor(img_mask1, img_result, COLOR_GRAY2BGR);
//
//
//     for (size_t i = 0; i < contours.size(); i++){
//         approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
//         // approxPolyDP(Mat(contours[i]), approx, 1, true);
//
//         if (fabs(contourArea(Mat(approx))) > 1500)  // edit responsiveness...
//         {
//             int size = approx.size();
//
//             //drawing contour
//             if (size % 2 == 0) {
//                 line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);
//
//                 for (int k = 0; k < size - 1; k++)
//                     line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
//                 for (int k = 0; k < size; k++)
//                     circle(img_result, approx[k], 3, Scalar(0, 0, 255));
//             }
//             else {
//                 line(img_result, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);
//
//                 for (int k = 0; k < size - 1; k++)
//                     line(img_result, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
//                 for (int k = 0; k < size; k++)
//                     circle(img_result, approx[k], 3, Scalar(0, 0, 255));
//             }
//             //circle!! stop!!
//             if (size >= 7){
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 cout << "red_circle" << endl;
//                 speed_ratio = 0;
//             }
//         }
//     }
//     cvtColor(img_result, srcRGB, COLOR_BGR2GRAY);
//     // srcRGB = img_result;
//     resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
//
//     return speed_ratio;
// }


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
	printf("hue = %d, low_hue = %d\n", hue, low_hue);

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);//make limit of hue value

	img_input = srcRGB;

	cvtColor(img_input, img_hsv, COLOR_BGR2HSV);

	Mat binary_image;
	Mat img_mask1, img_mask2, test;

	//accept red filter for detect red stop sign
	inRange(img_hsv, Scalar(low_hue1, 100, 100), Scalar(255, 255, 255), img_mask1);
	// inRange(img_input, Scalar(low_hue1, 250, 250), Scalar(high_hue1, 255, 255), test);
	if (range_count == 2) {
		inRange(img_hsv, Scalar(low_hue2, 100, 100), Scalar(255, 255, 255), img_mask2);
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
			if (size >= 7){
				// cout << "red_circle" << endl;
				speed = 0;
			}
		}
	}
	// cvtColor(test, srcRGB, COLOR_BGR2GRAY);
	srcRGB = img_result;
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
	printf("hue = %d, low_hue = %d\n", hue, low_hue);

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);//make limit of hue value

	img_input = srcRGB;

	cvtColor(img_input, img_hsv, COLOR_BGR2HSV);

	Mat binary_image;
	Mat img_mask1, img_mask2, test;

	//accept red filter for detect red stop sign
	inRange(img_hsv, Scalar(170, 100, 0), Scalar(180, 255, 255), img_mask1);
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

		if (fabs(contourArea(Mat(approx))) > 400)  // edit responsiveness...
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
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				cout << "red_sign" << endl;
				is_Traffic_Light_red = 1;//red!
			}
		}
	}
	// cvtColor(test, srcRGB, COLOR_BGR2GRAY);
	srcRGB = img_result;
	// resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

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
	inRange(img_hsv, Scalar(60, 100, 100), Scalar(120, 255, 255), img_mask1);//70 100 70
	if (range_count == 2) {
		inRange(img_hsv, Scalar(60, 100, 100), Scalar(120, 255, 255), img_mask2);
		img_mask1 |= img_mask2;
	}

	vector<vector<Point> > contours;
	findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	vector<Point> approx;
	cvtColor(img_mask1, img_result, COLOR_GRAY2BGR);

	for (size_t i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		if (fabs(contourArea(Mat(approx))) > 500) //[TODO]we have to do fine tuning 
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

			if (size == 7){//go left!
				// setLabel(img_result, "left!", contours[i]); //¿¿¿
				cout << "left" << endl;
				cout << "left" << endl;
				cout << "left" << endl;
				cout << "left" << endl;
				cout << "left" << endl;
				cout << "left" << endl;
				is_Traffic_Light = 1; //left signal
			}
			else if (size >= 8){//circle, go right!!
				// setLabel(img_result, "circle!!", contours[i]); //¿
				cout << "circle" << endl;
				cout << "circle" << endl;
				cout << "circle" << endl;
				cout << "circle" << endl;
				cout << "circle" << endl;
				cout << "circle" << endl;
				cout << "circle" << endl;
				is_Traffic_Light = 2; //right signal
			}
			// display result
			// resize(img_gray, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
		}
	}  
	srcRGB = img_result;
	resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

	return is_Traffic_Light;
}

// bool pixel_detector(Mat image, char* order){
//     int count = 0;
//     if(order == "first") {
//         for(int j = (2*(image.rows))/3; j < image.rows; j++){
//             uchar* pointer = image.ptr<uchar>(j); //access j row
//             for(int i = 0; i < (image.cols)/3; i++){
//                 //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
//                 if(int(pointer[i]) == 255) count ++;
//                 pointer[i]++;
//             }
//         }
//     }
//     else if(order == "second"){
//         for(int j = (2*(image.rows))/3; j < image.rows; j++){
//             uchar* pointer = image.ptr<uchar>(j); //access j row
//             for(int i = image.cols/3; i < (2*image.cols)/3; i++){
//                 //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
//                 if(int(pointer[i]) == 255) count ++;
//                 pointer[i]++;
//             }
//         }
//     }
//     else if(order == "third"){
//         for(int j = (2*(image.rows))/3; j < image.rows; j++){
//             uchar* pointer = image.ptr<uchar>(j); //access j row
//             for(int i = (2*image.cols)/3; i < image.cols; i++){
//                 //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
//                 if(int(pointer[i]) == 255) count ++;
//                 pointer[i]++;
//             }
//         }
//     }
//
//     cout << "count : " << count << endl;
//     return count > 1000 ? true : false;
// }
//
// Mat pre_histogram_backprojection(){
//     Mat srcImage = imread("overroad2.jpg", IMREAD_COLOR); // standard value
//     //if (srcImage.empty())
//     //	return ;
//     cout << "image size : " << srcImage.cols << " " << srcImage.rows << endl;
//     resize(srcImage, srcImage, Size(srcImage.cols/5, srcImage.rows/5));
//     Mat hsvImage;
//     cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);
//     Rect roi(Point(480, 570), Point(495, 585));
//     rectangle(srcImage, roi, Scalar(0, 0, 255), 2);
//     Mat roiImage = hsvImage(roi);
//
//     int histSize = 256;
//     float hValue[] = { 0, 256 };
//     float sValue[] = { 0, 256 };
//     float vValue[] = { 0, 256 };
//     const float* ranges[] = { hValue, sValue, vValue };
//     int channels[] = {0, 1, 2};
//     int dims = 3; // dimenstion
//
//     Mat hist;
//     calcHist(&roiImage, 1, channels, Mat(), hist, dims, &histSize, ranges); //histogram calculate my area
//
//     return hist;
// }
//
// int histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
// {
//     Mat dstRGB(nh, nw, CV_8UC3, outBuf); // 나중에
//     //Mat srcRGB(ih, iw, CV_8UC3, srcBuf); // 이미지 받아오기
//     Mat srcImage(ih, iw, CV_8UC3, srcBuf);
//     Mat resRGB(ih, iw, CV_8UC3);
//     Mat hsvImage;
//     float hValue[] = { 0, 256 };
//     float sValue[] = { 0, 256 };
//     float vValue[] = { 0, 256 };
//     const float* ranges[] = { hValue, sValue, vValue };
//     int channels[] = {0, 1, 2};
//
//     if (srcImage.empty())
//         cout << "Image access fail...\n" << endl;
//         return -1;//fail
//     cout << "image size : " << srcImage.cols << " " << srcImage.rows << endl;
//     resize(srcImage, srcImage, Size(srcImage.cols/5, srcImage.rows/5));
//
//     cvtColor(srcImage, hsvImage, COLOR_BGR2HSV); //히스토그램은 밝기값을 통해 계산하기 때문에
//                                                  //RGB 영상을 HSV 영상으로 바꾼 후 H 채널만 분리하도록 한다.
//
//     Mat hist = pre_histogram_backprojection(); // 여기서 pre_histogram 함수 사용
//     Mat backProject;
//     calcBackProject(&hsvImage, 1, channels, hist, backProject, ranges);
//
//     Mat backProject2;
//     normalize(backProject, backProject2, 0, 255, NORM_MINMAX, CV_8U); //정규화하여 더 나은 시각화를 이룸
//
//     threshold(backProject2, backProject2, 180, 255, CV_THRESH_BINARY);
//
//     Mat mask2 = getStructuringElement(MORPH_RECT, Size(4, 4));
//     dilate(backProject2, backProject2, mask2, Point(-1, -1), 1);
//
//     cv::resize(srcImage, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
//
//     bool answer = pixel_detector(backProject2, "first"); // 여기서 pixet_detector 함수 사용
//     cout << "첫번째 탐지" << endl;
//     if(answer == 1) cout << "장애물 감지!!" << endl;
//     else cout << "감지되지 않았습니다" << endl;
//
//     cout << "두번째 탐지" << endl;
//     bool answer2 = pixel_detector(backProject2, "second");
//     if(answer2 == 1) cout << "장애물 감지!!" << endl;
//     else cout << "감지되지 않았습니다" << endl;
//
//     bool answer3 = pixel_detector(backProject2, "third");
//     if(answer3 == 1) cout << "장애물 감지!!" << endl;
//     else cout << "감지되지 않았습니다" << endl;
//
//     if(answer == 0 && answer2 == 1 && answer3 == 1){
//         return 1;//left
//     }
//     else if(answer == 1 && answer2 == 1 && answer3 == 0){
//         return 2;//right
//     }
//     else return -1;//fail
// }
//


void OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
    Scalar lineColor = cv::Scalar(255,255,255);
    
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    //cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

    cv::Mat contours;
    cv::Canny(srcRGB, contours, 125, 350);
    
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, PI/180, 80);
    
    cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
    //printf("Lines detected: %d\n", lines.size());

    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
    while (it!=lines.end()) 
    {
        float rho = (*it)[0];
        float theta = (*it)[1];
        
        if (theta < PI/4. || theta > 3.*PI/4.)
        {
            cv::Point pt1(rho/cos(theta), 0); 
            cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
            cv::line(srcRGB, pt1, pt2, lineColor, 1);

        } 
        else 
        { 
            cv::Point pt1(0,rho/sin(theta));
            cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
            cv::line(srcRGB, pt1, pt2, lineColor, 1);
        }
        //printf("line: rho=%f, theta=%f\n", rho, theta);
        ++it;
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
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

bool get_intersectpoint(const Point& AP1, const Point& AP2,
	const Point& BP1, const Point& BP2, Point* IP)
{
	double t;
	double s;
	double under = (BP2.y - BP1.y)*(AP2.x - AP1.x) - (BP2.x - BP1.x)*(AP2.y - AP1.y);
	if (under == 0) return false;

	double _t = (BP2.x - BP1.x)*(AP1.y - BP1.y) - (BP2.y - BP1.y)*(AP1.x - BP1.x);
	double _s = (AP2.x - AP1.x)*(AP1.y - BP1.y) - (AP2.y - AP1.y)*(AP1.x - BP1.x);

	t = _t / under;
	s = _s / under;

	if (t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
	if (_t == 0 && _s == 0) return false;

	IP->x = AP1.x + t * (double)(AP2.x - AP1.x);
	IP->y = AP1.y + t * (double)(AP2.y - AP1.y);

	return true;
}

float data_transform(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float get_slope(const Point& p1, const Point& p2) {

	float slope;

	if (p2.y - p1.y != 0.0) {
		slope = ((float)p2.y - (float)p1.y) / ((float)p2.x - (float)p1.x);
	}
	return slope;
}


bool hough_left(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> linesL;
  vector<Vec2f> newLinesL;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){

    HoughLines(img, linesL, 1, CV_PI / 180, threshold);

    for(size_t j = 0; j < linesL.size(); j++){

      Vec2f temp;

      float rho = linesL[j][0];
      float theta = linesL[j][1];

      if(CV_PI / 18 >= theta || theta >= CV_PI / 18 * 8) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesL.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesL.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesL.size() > 1) {
  			for (size_t i = 0; i < newLinesL.size(); i++) {
  				count++;
  				float rho = newLinesL[i][0];
  				float theta = newLinesL[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;


  		return false;
  	}
  	return true;
}

bool hough_right(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> linesR;
  vector<Vec2f> newLinesR;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesR, 1, CV_PI / 180, threshold);



    for(size_t j = 0; j < linesR.size(); j++){

      Vec2f temp;

      float rho = linesR[j][0];
      float theta = linesR[j][1];

      if(CV_PI / 18 * 10 >= theta || theta >= CV_PI / 18 * 17) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesR.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesR.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesR.size() > 1) {
  			for (size_t i = 0; i < newLinesR.size(); i++) {
  				count++;
  				float rho = newLinesR[i][0];
  				float theta = newLinesR[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;

  		return false;
  	}
  	return true;
}

bool hough_curve(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> lines;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){
    HoughLines(img, lines, 1, CV_PI / 180, threshold);


    int clusterCount = 2;
  		Mat h_points = Mat(lines.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (lines.size() > 1) {
  			for (size_t i = 0; i < lines.size(); i++) {
  				count++;
  				float rho = lines[i][0];
  				float theta = lines[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;

  		return false;
  	}
  	return true;
}

int curve_detector(Mat& leftImg, Mat& rightImg, int number){

  bool error = true;

  float xLeft, xRight, slope, steer, skewness, y;
  int angle;
  float skewnessValue;

  switch(number){
    case 1 : case 3 : case 4 : case 5 : case 777 :
      y = 60.0;
    break;
    case 2 :
      y = 30.0;
    break;
    case 6 :
      y = 90.0;
    break;
  }

  Mat oriImg, roiImg, hsvImg, binaryImg, binaryImg1, cannyImg;
  Point p1, p2;

  hconcat(leftImg, rightImg, cannyImg);


  // switch(number){
  //   case 1 :  case 6 :
  //     cvtColor(oriImg, hsvImg, CV_BGR2HSV);
  //     inRange(hsvImg, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg);
  //     break;
  //   case 2 :
  //     roiImg = oriImg(Rect(0, oriImg.rows/2, oriImg.cols, oriImg.rows/2));
  //     cvtColor(roiImg, hsvImg, CV_BGR2HSV);
  //     inRange(hsvImg, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg);
  //     inRange(roiImg, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg1);
  //     binaryImg = binaryImg | binaryImg1;
  //     break;
  //   case 3 : case 4 : case 5 :
  //     cvtColor(oriImg, hsvImg, CV_BGR2HSV);
  //     inRange(hsvImg, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg);
  //     inRange(oriImg, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg1);
  //     binaryImg = binaryImg | binaryImg1;
  //     break;
  // }
  //
  // Canny(binaryImg, cannyImg, 150, 250);

  switch(number){
    case 1 : case 2 : case 5: case 777:
      error = hough_curve(cannyImg, roiImg, &p1, &p2);
      break;
    case 3 : case 6 :
      error = hough_left(cannyImg, roiImg, &p1, &p2);
      break;
    case 4 :
      error = hough_right(cannyImg, roiImg, &p1, &p2);
      break;
  }


  if(number == 777){
    skewnessValue = 2.5;
  }
  else{
    skewnessValue = 2.0;
  }

  slope = get_slope(p1, p2);

    if(error){
      return 1520;
    }
    else if(slope < 0.0){ // right rotate

      if(slope < -1.2) slope = -1.2;
      else if(slope > -0.2) slope = -0.2;

      steer =  data_transform(slope, -1.2, -0.2, 100.0, 500.0);

      xLeft = (y - p1.y + slope * p1.x) / slope;

      if(xLeft < -120.0) xLeft = -120.0;
      else if(xLeft > 220.0) xLeft = 220.0;

      skewness = data_transform(xLeft, -120.0, 220.0, 0.0, skewnessValue);

      steer = 1520.0 - (steer * skewness);
      angle = steer;

      if(angle > 1520){
        angle = 1520;
      }

      return angle;
    }
    else{

      if(slope < 0.2) slope = 0.2;
      else if(slope > 1.2) slope = 1.2;

      steer =  data_transform(slope, 0.2, 1.2, -500.0, -100.0);

      xRight = (y - p1.y + slope * p1.x) / slope;

      if(xRight < 100.0) xRight = 100.0;
      else if(xRight > 440.0) xRight = 440.0;

      skewness = data_transform(xRight, 100, 440, -skewnessValue, 0.0);

      steer = 1520.0 + (steer * skewness);
      angle = steer;

      if(angle < 1520){
        return 1520;
      }

      return angle;
    }


}


int line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[], int modeNum){

  int angle = 1520;
  Point p1, p2, p3, p4, p5;

  bool left_error = true;
  bool right_error = true;

  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //output
  Mat resRGB(ih, iw, CV_8UC3); //reuslt

  Mat oriImg;
  Mat leftROI, rightROI, roiImg;
  Mat hsvImg1, hsvImg2;
  Mat binaryImg1, binaryImg2, binaryImg3, binaryImg4;
  Mat cannyImg1, cannyImg2;

  if(modeNum == 6){
    leftROI = srcRGB(Rect(0, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/2));
    rightROI = srcRGB(Rect(srcRGB.cols/2, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/2));
  }
  else{
    leftROI = srcRGB(Rect(0, srcRGB.rows/3 * 2, srcRGB.cols/2, srcRGB.rows/3));
    rightROI = srcRGB(Rect(srcRGB.cols/2, srcRGB.rows/3 * 2, srcRGB.cols/2, srcRGB.rows/3));
  }

  cvtColor(leftROI, hsvImg1, CV_BGR2HSV);
  cvtColor(rightROI, hsvImg2, CV_BGR2HSV);

  inRange(hsvImg1, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
  inRange(hsvImg2, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg2);

  if(modeNum == 1 || modeNum == 6 || modeNum == 777){ // 777 : Curve mission
    hconcat(binaryImg1, binaryImg2, resRGB);
    cvtColor(resRGB, resRGB, CV_GRAY2BGR);
    resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
  }
  else if(modeNum == 2){
    hconcat(binaryImg1, binaryImg2, resRGB);
    oriImg = resRGB(Rect(0, resRGB.rows/3, resRGB.cols, resRGB.rows/3 * 2));
    cvtColor(oriImg, oriImg, CV_GRAY2BGR);
    resize(oriImg, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
  }
  else if(modeNum == 3 || modeNum == 4 || modeNum == 5){
    inRange(leftROI, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg3);
    inRange(rightROI, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg4);

    binaryImg1 = binaryImg1 | binaryImg3;
    binaryImg2 = binaryImg2 | binaryImg4;

    hconcat(binaryImg1, binaryImg2, resRGB);

    cvtColor(resRGB, resRGB, CV_GRAY2BGR);
    resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
  }


  Canny(binaryImg1, cannyImg1, 150, 250);
  Canny(binaryImg2, cannyImg2, 150, 250);

  left_error = hough_left(cannyImg1, leftROI, &p1, &p2);
  right_error = hough_right(cannyImg2, rightROI, &p3, &p4);

  if(left_error || right_error){

    angle = curve_detector(cannyImg1, cannyImg2, modeNum);

  }
  else{

    get_intersectpoint(p1, p2, Point(p3.x + 160, p3.y), Point(p4.x + 160, p4.y), &p5);


    float steer;
    float x_Difference = 160.0 - p5.x;

    if(x_Difference > 0.0){
      steer = 1520.0 + 0.1 * x_Difference;
    }
    else if(x_Difference < 0.0){
      steer = 1520.0 + 0.1 * x_Difference;
    }
    else{
      steer = 1520.0;
    }
    angle = steer;
  }

  // resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

  if(angle > 2000){
    angle = 2000;
  }
  else if(angle < 1000){
    angle = 1000;
  }

  return angle;

}



