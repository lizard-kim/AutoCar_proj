#include "laneDetection.h"
#include "passing_master.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <cmath>
#include <vector>


#define PI 3.1415926

using namespace std;
using namespace cv;

#ifdef __cplusplus
extern "C" {
#endif
/*
struct passing {
    Mat hist;
    int hist_check = 0;
};
*/

passing *pass = new passing;
//pass* passi;

bool pixel_detector(Mat image, char* order){
	int count = 0;
    if(order == "first") {
        for(int j = (2*(image.rows))/3; j < image.rows; j++){
            uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
            for(int i = 0; i < (image.cols)/3; i++){
                //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
                if(int(pointer[i]) == 255) count ++;
                pointer[i]++;
            }
	    }
    }
    else if(order == "second"){
        for(int j = (2*(image.rows))/3; j < image.rows; j++){
            uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
            for(int i = image.cols/3; i < (2*image.cols)/3; i++){
                //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
                if(int(pointer[i]) == 255) count ++;
                pointer[i]++;
            }
	    }
    }
    else if(order == "third"){
        for(int j = (2*(image.rows))/3; j < image.rows; j++){
            uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
            for(int i = (2*image.cols)/3; i < image.cols; i++){
                //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
                if(int(pointer[i]) == 255) count ++;
                pointer[i]++;
            }
	    }
    }
	
	cout << "count 값 : " << count << endl;
	return count > 1000 ? true : false;
}

void pre_histogram_backprojection(unsigned char* srcBuf, int iw, int ih){
    
    Mat srcImage(ih, iw, CV_8UC3, srcBuf);
    //Mat srcImage = imread("overroad2.jpg", IMREAD_COLOR); // 이게 기준값!
    //if (srcImage.empty())
	//	return ; 
    cout << "이미지 높이 ih : \n" << ih << endl; 
    cout << "이미지 너비 iw : \n" << iw << endl;
    cout << "이미지의 크기는 : " << srcImage.cols << " " << srcImage.rows << endl;
    resize(srcImage, srcImage, Size(srcImage.cols/5, srcImage.rows/5));
    Mat hsvImage;
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);
    //Rect roi(Point(480, 570), Point(495, 585));
    Rect roi(Point( (1/2)*iw - 20, ih - 30), Point( (1/2)*iw + 20, ih - 10));
    rectangle(srcImage, roi, Scalar(0, 0, 255), 2);  
    Mat roiImage = hsvImage(roi);

    int histSize = 256;
	float hValue[] = { 0, 256 };
    float sValue[] = { 0, 256 };
    float vValue[] = { 0, 256 };
	const float* ranges[] = { hValue, sValue, vValue };
	int channels[] = {0, 1, 2};
	int dims = 3; // dimenstion 차원

	Mat hist;
	calcHist(&roiImage, 1, channels, Mat(), hist, dims, &histSize, ranges); //관심 영역을 히스토그램 계산

    //passing->hist
    //passing pre;
    //struct pass pre;
    passi->hist = hist;
    passi->hist_check = 1;
}

int histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, Mat hist)
{  
    Mat dstRGB(nh, nw, CV_8UC3, outBuf); // 나중에 
    //Mat srcRGB(ih, iw, CV_8UC3, srcBuf); // 이미지 받아오기
    Mat srcImage(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    Mat hsvImage;
    float hValue[] = { 0, 256 };
    float sValue[] = { 0, 256 };
    float vValue[] = { 0, 256 };
	const float* ranges[] = { hValue, sValue, vValue };
	int channels[] = {0, 1, 2};
    //passing pre;


    if (srcImage.empty())
		cout << "이미지 인식 실패\n" << endl;
        return -1;//fail
    cout << "이미지의 크기는 : " << srcImage.cols << " " << srcImage.rows << endl;
    resize(srcImage, srcImage, Size(srcImage.cols/5, srcImage.rows/5));
   
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV); //히스토그램은 밝기값을 통해 계산하기 때문에
												 //RGB 영상을 HSV 영상으로 바꾼 후 H 채널만 분리하도록 한다.

    //Mat hist = pre_histogram_backprojection(srcBuf, iw, ih); // 여기서 pre_histogram 함수 사용
    Mat backProject;
	calcBackProject(&hsvImage, 1, channels, hist, backProject, ranges);

    Mat backProject2;
	normalize(backProject, backProject2, 0, 255, NORM_MINMAX, CV_8U); //정규화하여 더 나은 시각화를 이룸

    threshold(backProject2, backProject2, 180, 255, CV_THRESH_BINARY);

    Mat mask2 = getStructuringElement(MORPH_RECT, Size(4, 4));
    dilate(backProject2, backProject2, mask2, Point(-1, -1), 1);

    cv::resize(srcImage, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    bool answer = pixel_detector(backProject2, "first"); // 여기서 pixet_detector 함수 사용
    cout << "첫번째 탐지" << endl;
	if(answer == 1) cout << "장애물 감지!!" << endl;
	else cout << "감지되지 않았습니다" << endl;

    cout << "두번째 탐지" << endl;
	bool answer2 = pixel_detector(backProject2, "second");
	if(answer2 == 1) cout << "장애물 감지!!" << endl;
	else cout << "감지되지 않았습니다" << endl;

    bool answer3 = pixel_detector(backProject2, "third");
	if(answer3 == 1) cout << "장애물 감지!!" << endl;
	else cout << "감지되지 않았습니다" << endl;

    if(answer == 0 && answer2 == 1 && answer3 == 1){
        return 1; //left
    }
    else if(answer == 1 && answer2 == 1 && answer3 == 0){
        return 2; //right
    }
    else return -1; //fail
}


#ifdef __cplusplus
}
#endif
