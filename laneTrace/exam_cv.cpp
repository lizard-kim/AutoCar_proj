
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
signed short OpenCV_red_Detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) // detect red stop sign
{

	int range_count = 0;
	int stopornot = 1;
	Mat img_input, img_gray, img_hsv, img_result;
	signed short speed_ratio = 1; //[TODO]basic speed you can edit this value!
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
	inRange(img_hsv, Scalar(low_hue1, 150, 150), Scalar(255, 255, 255), img_mask1);
	// inRange(img_input, Scalar(low_hue1, 250, 250), Scalar(high_hue1, 255, 255), test);
	if (range_count == 2) {
		inRange(img_hsv, Scalar(low_hue2, 150, 150), Scalar(255, 255, 255), img_mask2);
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
				speed_ratio = 0;
			}
		}
	}
	// cvtColor(test, srcRGB, COLOR_BGR2GRAY);
	srcRGB = img_result;
	resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

	return speed_ratio;
}

int OpenCV_green_Detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) // mechanism is similar with red_detection func...
{

	int range_count = 0;
	Mat img_input, img_result, img_gray;
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
	int low_hue = hue - 30;//¿¿¿ ¿¿
	int high_hue = hue + 20;
	int low_hue1 = 0, low_hue2 = 0;
	int high_hue1 = 0, high_hue2 = 0;

	MakeLimit(low_hue, low_hue1, low_hue2, high_hue, high_hue1, high_hue2, range_count);

	img_input = srcRGB;

	cvtColor(img_input, img_gray, COLOR_BGR2HSV);

	Mat binary_image;
	Mat img_mask1, img_mask2;

	//accept green filter for detect Traffic light
	inRange(img_gray, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
	if (range_count == 2) {
		inRange(img_gray, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
		img_mask1 |= img_mask2;
	}

	vector<vector<Point> > contours;
	findContours(img_mask1, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	vector<Point> approx;

	for (size_t i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		if (fabs(contourArea(Mat(approx))) > 300) //[TODO]we have to do fine tuning 
		{
			int size = approx.size();

			if (size % 2 == 0) {
				line(img_gray, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_gray, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_gray, approx[k], 3, Scalar(0, 0, 255));
			}
			else {
				line(img_gray, approx[0], approx[approx.size() - 1], Scalar(0, 255, 0), 3);

				for (int k = 0; k < size - 1; k++)
					line(img_gray, approx[k], approx[k + 1], Scalar(0, 255, 0), 3);
				for (int k = 0; k < size; k++)
					circle(img_gray, approx[k], 3, Scalar(0, 0, 255));
			}

			if (size == 7){//go left!
				// setLabel(img_result, "left!", contours[i]); //¿¿¿
				cout << "left" << endl;
				is_Traffic_Light = 1; //left signal
			}
			else if (size >= 8){//circle, go right!!
				// setLabel(img_result, "circle!!", contours[i]); //¿
				cout << "circle" << endl;
				is_Traffic_Light = 2; //right signal
			}
			// display result
			// resize(img_gray, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
		}
	}  
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

