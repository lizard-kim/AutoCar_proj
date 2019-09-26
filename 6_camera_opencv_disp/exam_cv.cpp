
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

#include "car_lib.h"
#include "stop_when_accident.h"
#define PI 3.1415926

using namespace std;
using namespace cv;

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
     // ÄÉ´Ï ¾Ë°í¸®Áò Àû¿ë
    cv::Mat contours;
    cv::Canny(srcGRAY, // ±×·¹ÀÌ·¹º§ ¿µ»ó
        contours, // °á°ú ¿Ü°û¼±
        125,  // ³·Àº °æ°è°ª
        350);  // ³ôÀº °æ°è°ª

    // ³ÍÁ¦·Î È­¼Ò·Î ¿Ü°û¼±À» Ç¥ÇöÇÏ¹Ç·Î Èæ¹é °ªÀ» ¹ÝÀü
    //cv::Mat contoursInv; // ¹ÝÀü ¿µ»ó
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
    // ¹à±â °ªÀÌ 128º¸´Ù ÀÛÀ¸¸é 255°¡ µÇµµ·Ï ¼³Á¤
 
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
void OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{

	int range_count = 0;
	Mat img_input, img_result, img_gray;
	Scalar blue(10, 200, 50);
	Scalar red(0, 0, 255);
	Mat rgb_color, hsv_color;
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

	Coloring(rgb_color, red);

	cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	//int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	//int value = (int)hsv_color.at<Vec3b>(0, 0)[2];
	int low_hue = hue - 5;//¿¿¿ ¿¿
	int high_hue = hue + 0.2;
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);

	// namedWindow("CAM", 0);
	// resizeWindow("CAM", 1280, 720);
	//while(1){
	//¿¿¿¿¿¿ ¿¿¿¿ image¿ ¿¿  
	//cap.read(img_input);
	img_input = srcRGB;

	//¿¿¿¿¿¿ ¿¿¿¿ ¿¿  
	cvtColor(img_input, img_gray, COLOR_BGR2HSV);

	//¿¿¿ ¿¿¿¿ ¿¿
	Mat binary_image;
	//threshold(img_hsv, img_hsv, 125, 255, THRESH_BINARY_INV | THRESH_OTSU);
	Mat img_mask1, img_mask2;
	inRange(img_gray, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
	if (range_count == 2) {
		inRange(img_gray, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
		img_mask1 |= img_mask2;
	}

	//contour¿ ¿¿¿.
	vector<vector<Point> > contours;
	findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	//contour¿ ¿¿¿¿¿.
	vector<Point> approx;
	img_result = img_mask1.clone();

	for (size_t i = 0; i < contours.size(); i++){
		// printf("wtfsdfafasdf\n");
		// printf("i = %d\n", i);
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		// approxPolyDP(Mat(contours[i]), approx, 1, true);
		// printf("wtf\n");

		if (fabs(contourArea(Mat(approx))) > 1200)  //¿¿¿ ¿¿¿¿ ¿¿¿¿¿ ¿¿. 
		{
			int size = approx.size();

			//Contour¿ ¿¿¿¿ ¿¿¿ ¿¿¿.
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

			//¿¿¿ ¿¿¿¿.
			if (size == 7){
				setLabel(img_result, "left!", contours[i]); //¿¿¿
			}
			else if (size > 7){
				setLabel(img_result, "circle!!", contours[i]); //¿
				cout << "circle" << endl;
			}
			/*
			//¿¿¿ ¿¿ ¿¿¿¿ ¿¿ convex¿¿ ¿¿ ¿¿
			else if (size == 4 && isContourConvex(Mat(approx))) 
			setLabel(img_result, "rectangle", contours[i]); //¿¿¿

			//¿ ¿¿¿ ¿¿ ¿¿¿ ¿¿¿ ¿¿¿ ¿¿¿ ¿¿¿ ¿¿
			else setLabel(img_result, to_string(approx.size()), contours[i]);
			*/
		}
	}  
	dstRGB = img_result.clone();
	// printf("wtf2\n");
    resize(img_result, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
	// printf("wtf3\n");
	// outBuf = img_result;
	//imshow("input", img_input);
	//imshow("result", img_result);
	//waitKey(1);
	//	}
	//return 0;
}

    
        /*
    //cout << "akkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk" << endl;
    //[TODO]
    int cam_id = 0;
    int range_count=0;

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    
    Scalar red = cv::Scalar(0,0,255);
	Mat rgb_color = Mat(1, 1, CV_8UC3, red);//coloring red in all pixels
	Mat hsv_color;
    
    cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	int value = (int)hsv_color.at<Vec3b>(0, 0)[2];

    int low_hue = hue - 3;//¿¿¿ ¿¿
	int high_hue = hue + 3;
    int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	if (low_hue < 10 ) {
		range_count = 2;

		high_hue1 = 180;
		low_hue1 = low_hue + 180;
		high_hue2 = high_hue;
		low_hue2 = 0;
	}
	else if (high_hue > 170) {
		range_count = 2;

		high_hue1 = low_hue;
		low_hue1 = 180;
		high_hue2 = high_hue - 180;
		low_hue2 = 0;
	}
	else {
		range_count = 1;

		low_hue1 = low_hue;
		high_hue1 = high_hue;
	}

    //videocapture cap(cam_id);
    //if(!cap.isopened()){
    //    cout << "wtf error!!" << endl;
    //    return -1;
    //}
    //namedwindow("cam", 0);
    //resizewindow("cam", 1280, 720);
    Mat img_hsv;

    double width = iw;//cap.get(cv_cap_prop_frame_width);
    double height = ih;//cap.get(cv_cap_prop_frame_height);
    int whitenum = 0;

    cvtColor(srcRGB, img_hsv, COLOR_BGR2HSV);//hsv¿ ¿¿


    //¿¿¿ hsv ¿¿¿ ¿¿¿¿ ¿¿¿ ¿¿¿
    Mat img_mask1, img_mask2;
    inRange(img_hsv, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
    if (range_count == 2) {
            inRange(img_hsv, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
            img_mask1 |= img_mask2;
    }


    //morphological opening ¿¿ ¿¿¿ ¿¿ 
    erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


    //morphological closing ¿¿¿ ¿¿ ¿¿¿ 
    dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    for(int i = 0; i < iw; i++){
            for(int j = 0; j < ih; j++){
                    if(img_mask1.at<uchar>(j,i)==255){
                            whitenum++;
                    }
            }
    }

    //imshow("cam", img_mask1);

    //waitkey(1);
    if(whitenum > 100){
            cout << "red!!!!" << whitenum << endl;
            //alarm_write(on);
    }
    whitenum = 0;

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);


*/


    /*
    Scalar lineColor = cv::Scalar(255,255,255);
    
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    //cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

    // Ä³´Ï ¾Ë°í¸®Áò Àû¿ë
    cv::Mat contours;
    cv::Canny(srcRGB, contours, 125, 350);
    
    // ¼± °¨Áö À§ÇÑ ÇãÇÁ º¯È¯
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, PI/180, // ´Ü°èº° Å©±â (1°ú ¥ð/180¿¡¼­ ´Ü°èº°·Î °¡´ÉÇÑ ¸ðµç °¢µµ·Î ¹ÝÁö¸§ÀÇ ¼±À» Ã£À½)
        80);  // ÅõÇ¥(vote) ÃÖ´ë °³¼ö
    
    // ¼± ±×¸®±â
    cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
    //printf("Lines detected: %d\n", lines.size());

    // ¼± º¤ÅÍ¸¦ ¹Ýº¹ÇØ ¼± ±×¸®±â
    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
    while (it!=lines.end()) 
    {
        float rho = (*it)[0];   // Ã¹ ¹øÂ° ¿ä¼Ò´Â rho °Å¸®
        float theta = (*it)[1]; // µÎ ¹øÂ° ¿ä¼Ò´Â µ¨Å¸ °¢µµ
        
        if (theta < PI/4. || theta > 3.*PI/4.) // ¼öÁ÷ Çà
        {
            cv::Point pt1(rho/cos(theta), 0); // Ã¹ Çà¿¡¼­ ÇØ´ç ¼±ÀÇ ±³Â÷Á¡   
            cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
            // ¸¶Áö¸· Çà¿¡¼­ ÇØ´ç ¼±ÀÇ ±³Â÷Á¡
            cv::line(srcRGB, pt1, pt2, lineColor, 1); // ÇÏ¾á ¼±À¸·Î ±×¸®±â

        } 
        else // ¼öÆò Çà
        { 
            cv::Point pt1(0,rho/sin(theta)); // Ã¹ ¹øÂ° ¿­¿¡¼­ ÇØ´ç ¼±ÀÇ ±³Â÷Á¡  
            cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
            // ¸¶Áö¸· ¿­¿¡¼­ ÇØ´ç ¼±ÀÇ ±³Â÷Á¡
            cv::line(srcRGB, pt1, pt2, lineColor, 1); // ÇÏ¾á ¼±À¸·Î ±×¸®±â
        }
        //printf("line: rho=%f, theta=%f\n", rho, theta);
        ++it;
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
*/
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



