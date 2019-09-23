#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "car_lib.h"
//안녕
using namespace std;
using namespace cv;

int main(){
    int CAM_ID = 0;
    int range_count=0;
    Scalar red(0, 0, 255); // red
	Mat rgb_color = Mat(1, 1, CV_8UC3, red);//1픽셀 빨간색으로 색칠
	Mat hsv_color;
    
    cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);//색공간 변환 rgb를 COLOR를 거쳐 hsv에 저장

	int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
	int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
	int value = (int)hsv_color.at<Vec3b>(0, 0)[2];

    int low_hue = hue - 3;//임계값 정의
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

    VideoCapture cap(CAM_ID);
    if(!cap.isOpened()){
        cout << "wtf error!!" << endl;
        return -1;
    }
    //namedWindow("CAM", 0);
    //resizeWindow("CAM", 1280, 720);
    Mat img_frame, img_hsv;

    double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    int whitenum = 0;
    while(1){
        cap.read(img_frame);
        if (img_frame.empty()) {
		    cerr << "ERROR! blank frame grabbed\n";
		    break;
        }
        cvtColor(img_frame, img_hsv, COLOR_BGR2HSV);//HSV로 변환

    
		//지정한 HSV 범위를 이용하여 영상을 이진화
		Mat img_mask1, img_mask2;
		inRange(img_hsv, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
		if (range_count == 2) {
			inRange(img_hsv, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
			img_mask1 |= img_mask2;
		}


		//morphological opening 작은 점들을 제거 
		erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


		//morphological closing 영역의 구멍 메우기 
		dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        for(int i = 0; i < width; i++){
            for(int j = 0; j < height; j++){
                if(img_mask1.at<uchar>(j,i)==255){
                    whitenum++;
                }
            }
        }

        //imshow("CAM", img_mask1);

        //waitKey(1);
        if(whitenum > 100000){
            cout << "red!!!!" << whitenum << endl;
            Alarm_Write(ON);
        }
        whitenum = 0;
    }
    return 0;
}
