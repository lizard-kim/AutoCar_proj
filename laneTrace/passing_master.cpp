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

bool pixel_detector(Mat image){
	int count = 0;
    int count1 = 0;
    int count2 = 0;
    for(int j = 80; j < 170; j++){
            uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
            for(int i = 0; i < 110; i++){
                //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
                if(int(pointer[i]) > 0) count ++;
                pointer[i]++;
            }
	    }
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
        cout << " <<<<<<<<<<<<<<<<<< count 값 : " << count << endl;
    for(int j = 80; j < 170; j++){
            uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
            for(int i = 110; i < 210; i++){
                //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
                if(int(pointer[i]) > 0) count1 ++;
                pointer[i]++;
            }
	    }
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count1 값 : " << count1 << endl;
    for(int j = 80; j < 170; j++){
            uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
            for(int i = 210; i < 320; i++){
                //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
                if(int(pointer[i]) > 0) count2 ++;
                pointer[i]++;
            }
	    }
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        cout << "<<<<<<<<<<<<<<<<<<  count2 값 : " << count2 << endl;
        

    // if(order == "first") {
    //     for(int j = 140; j < 170; j++){
    //         uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
    //         for(int i = 0; i < 100; i++){
    //             //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
    //             if(int(pointer[i]) == 255) count ++;
    //             pointer[i]++;
    //         }
	//     }
    //     cout << "count 값 : " << count << endl;
    // }
    // else if(order == "second"){
    //     for(int j = 140; j < 170; j++){
    //         uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
    //         for(int i = 120; i < 200; i++){
    //             //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
    //             if(int(pointer[i]) == 255) count1 ++;
    //             pointer[i]++;
    //         }
	//     }
    //     cout << "count1 값 : " << count1 << endl;
    // }
    // else if(order == "third"){
    //     for(int j = 140; j < 180; j++){
    //         uchar* pointer = image.ptr<uchar>(j); //j번째 행에 접근
    //         for(int i = 220; i < 320; i++){
    //             //cout << "(" << i << ", " << j << ") " << "pixel 값 : " << int(pointer[i]) << endl;
    //             if(int(pointer[i]) == 255) count2 ++;
    //             pointer[i]++;
    //         }
	//     }
    //     cout << "count2 값 : " << count2 << endl;
    // }
    bool answer = abs(count - count1) > abs(count1 - count2) ? true : false;
    cout << "###########  answer == " << answer << endl;
    cout << "###########  answer == " << answer << endl;
	return abs(count - count1) > abs(count1 - count2) ? true : false; // false이면 오른쪽, true이면 왼쪽으로, 값이 같아도 왼쪽으로
}


Mat pre_histogram_backprojection(unsigned char* srcBuf, int iw, int ih){
    //Mat srcImage = imread("capture.png", IMREAD_COLOR); // 이게 기준값!
    Mat srcImage(ih, iw, CV_8UC3, srcBuf);
    Mat srcImage2;
    //if (srcImage.empty())
	//	return ;
    resize(srcImage, srcImage2, Size(320, 180));
    //Mat srcImage2 = srcImage.clone(); // 여기에 사각형을 그리자 (시각화)
    cout << "srcImage size(cols, rows) : " << srcImage.cols << " " << srcImage.rows << endl;
    Mat hsvImage;
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV);
    //Rect roi1(Point(0, 140), Point(80, 170));
    Rect roi(Point(130, 140), Point(190, 160));
    //Rect roi3(Point(220, 140), Point(320, 170));
    //rectangle(srcImage2, roi1, Scalar(0, 0, 255), 2);
    rectangle(srcImage2, roi, Scalar(0, 0, 255), 2);
    //rectangle(srcImage, roi3, Scalar(0, 0, 255), 2);
    Mat roiImage = hsvImage(roi);
    //imshow("srcImage_pre", srcImage2);
    //waitKey(0);
    
    int histSize = 256;
	float hValue[] = { 0, 256 };
    float sValue[] = { 0, 256 };
    float vValue[] = { 0, 256 };
	const float* ranges[] = { hValue, sValue, vValue };
	int channels[] = {0, 1, 2};
	int dims = 3; // dimenstion 차원

	Mat hist, hist2;
    cout << "what.." << endl;
	calcHist(&roiImage, 1, channels, Mat(), hist, dims, &histSize, ranges); //관심 영역을 히스토그램 계산
    //cout << "this is hist value" << double(hist.at<uchar>(5,5)) << endl << endl;
    cout << "hist.cols : " << hist.cols << "hist.rows : " << hist.rows << endl;
    return hist;
}

char* histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{   

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    cout << "come in passing_master.cpp in passing_master" << endl;
    cout << "come in passing_master.cpp in passing_master" << endl;
    cout << "come in passing_master.cpp in passing_master" << endl;
    cout << "srcBuf real size(ih, iw) : " << ih << " " << iw << " << in histogram_backprojection" << endl; 
    //Mat dstRGB(nh, nw, CV_8UC3, outBuf); // 나중에 
    //Mat srcRGB(ih, iw, CV_8UC3, srcBuf); // 이미지 받아오기
    //Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    //Mat resRGB(ih, iw, CV_8UC3);
    Mat hsvImage;
    float hValue[] = { 0, 256 };
    float sValue[] = { 0, 256 };
    float vValue[] = { 0, 256 };
	const float* ranges[] = { hValue, sValue, vValue };
	int channels[] = {0, 1, 2};
    //passing pre;

    cout << "이미지의 크기는 : " << srcRGB.cols << " " << srcRGB.rows << endl;
    if (srcRGB.empty()){
        cout << "이미지 인식 실패\n" << endl;
        return "fail"; //fail
    }
    //cout << "이미지의 크기는 : " << srcRGB.cols << " " << srcRGB.rows << endl;
    //resize(srcImage, srcImage, Size(srcImage.cols/5, srcImage.rows/5));
   
    cvtColor(srcRGB, hsvImage, COLOR_BGR2HSV); //히스토그램은 밝기값을 통해 계산하기 때문에
												 //RGB 영상을 HSV 영상으로 바꾼 후 H 채널만 분리하도록 한다.
    Mat hist;
    cout << "go to pre_histogram" << endl;
    cout << "go to pre_histogram" << endl;
    cout << "go to pre_histogram" << endl;
    cout << "go to pre_histogram" << endl;
    hist = pre_histogram_backprojection(srcBuf, iw, ih); // 여기서 pre_histogram 함수 사용
    Mat backProject;
	calcBackProject(&hsvImage, 1, channels, hist, backProject, ranges);

    Mat backProject2;
	normalize(backProject, backProject2, 0, 255, NORM_MINMAX, CV_8U); //정규화하여 더 나은 시각화를 이룸

    threshold(backProject2, backProject2, 180, 255, CV_THRESH_BINARY);

    Mat mask2 = getStructuringElement(MORPH_RECT, Size(4, 4));
    dilate(backProject2, backProject2, mask2, Point(-1, -1), 1);

    //cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    cv::resize(backProject2, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    bool answer = pixel_detector(backProject2); // 여기서 pixet_detector 함수 사용
    cout << "start pixel detector!!! " << endl;

     // false이면 오른쪽, true이면 왼쪽으로

    //bool answer = pixel_detector(backProject2);
    if(answer == false){
        return "right";
        cout << "--------------right--------------" << endl;
    }
    else if (answer == true){
        return "left";
        cout << "--------------left---------------" << endl;
    }

    // cout << "두번째 탐지" << endl;
	// bool answer2 = pixel_detector(backProject2, "second");
	// if(answer2 == 1) cout << "장애물 감지!!" << endl;
	// else cout << "감지되지 않았습니다" << endl;

    // bool answer3 = pixel_detector(backProject2, "third");
	// if(answer3 == 1) cout << "장애물 감지!!" << endl;
	// else cout << "감지되지 않았습니다" << endl;

    // if(answer == 0 && answer2 == 1 && answer3 == 1){
    //     return "left"; //left
    // }
    // else if(answer == 1 && answer2 == 1 && answer3 == 0){
    //     return "right"; //right
    // }
    // else return "fail"; //fail
}

char* stop_line_detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{  

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);


    //Mat image_ori = imread("stop_line.jpg"); // 7개
    Mat image_ori(ih, iw, CV_8UC3, srcBuf);
    //Mat image_ori = imread("stop_line_2.jpg"); // 7개
    //Mat image_ori = imread("stop_line_3.jpg"); // 24개 - 가까워지면
    cout << "111111111111111111111" << endl;
    Mat image;

    cout << "aaaaaaaaaaaa size : " << image_ori.cols << " " << image_ori.rows << endl;
    cout << "aaaaaaaaaaaa size2 : " << ih << " " << iw << endl;

    //resize(image_ori, image_ori, Size(320, 180), 0, 0, CV_INTER_LINEAR);

    cout << "111111111111111111111" << endl;

    // 1. grayscale로 바꾸어 줍니다
    cvtColor(image_ori, image, CV_RGB2GRAY);

    cout << "111111111111111111111" << endl;

    // 2. local averaging을 실행합니다. 이것은 gaussian blur와는 다릅니다
    Mat image_blur;
    blur(image, image_blur, Size(5,5));

    cout << "111111111111111111111" << endl;

    // 3. canny edge 함수를 사용해서 외곽선을 추출합니다
    Mat image_canny;
    Canny(image_blur, image_canny, 50, 150);

    cout << "111111111111111111111" << endl;

    // 3-2. sobel filter 함수를 사용해서 외곽선을 추출합니다
    Mat image_sobel;
    Sobel(image_blur, image_sobel, CV_8UC1, 0, 1);

    cout << "2222222222222222222222" << endl;

    // 4. ROI를 설정합니다
    //Rect rect(image_canny.rows, image_canny.cols*3/4, image_canny.rows, image_canny.cols/2);
    //Rect rect(0, image_canny.cols*1/2, image_canny.rows, image_canny.cols);
    //Rect rc(x,y,w,h);

    Rect rect(45, image_canny.rows/2 + 10, image_canny.cols-45, image_canny.rows/2 - 20); // 형식은 (x,y,w,h)을 따른다
    //Mat rectangle_test(Size(320, 180), CV_8UC3, Scalar(0,0,0));
    //rectangle(rectangle_test, rect, Scalar(0,255,0), 3);
    //imshow("test", rectangle_test);
    //image_canny = image_canny(rect);
    image_canny = image_canny(rect);

    // 5. 허프변환을 통해 외곽선 중 직선을 검출합니다
    Mat image_empty(Size(image_ori.cols, image_ori.rows/2), CV_8UC3, Scalar(0,0,0));
    Mat image_empty_2(Size(image_ori.cols, image_ori.rows/2), CV_8UC3, Scalar(0,0,0));
    vector<Vec4i> lines;
    vector<Vec4i> lines_two;
    double rho = 1;
    double theta = CV_PI/180;
    int hough_threshold = 15;
    double minLineLength = 20; // 직선을 구성하는 픽셀의 최소 길이
    double maxLineGap = 10; // 이걸 작게 설정하면 기울기가 0보다 큰 직선들이 검출된다.. 왜일까?

    cout << "2222222222222222222222" << endl;


    HoughLinesP(image_canny, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);
    for (int i=0; i<lines.size(); i++){
        Vec4i L = lines[i];
        line(image_empty, Point(L[0],L[1]), Point(L[2],L[3]), Scalar(0,0,255), 1, CV_AA);
    }
    cout << "line.size() : " << lines.size() << endl;
    

    // 6. 침식, 팽창 연산을 통해 검출된 직선을 굵게 만든다
    Mat eroded, dilated, dilated_gray; 
    Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    //erode(image_empty, eroded, mask, cv::Point(-1, -1), 3);
    dilate(image_canny, dilated, mask, cv::Point(-1, -1), 3);
    //cvtColor(dilated, dilated_gray, COLOR_BGR2GRAY);

    cout << "2222222222222222222222" << endl;

    // 7. 다시 허프변환을 통해 외곽선 중 직선을 검출합니다
    HoughLinesP(dilated, lines_two, rho, theta, hough_threshold, minLineLength, maxLineGap);
    for (int i=0; i<lines_two.size(); i++){
        Vec4i L_two = lines_two[i];
        line(image_empty_2, Point(L_two[0],L_two[1]), Point(L_two[2],L_two[3]), Scalar(0,0,255), 1, CV_AA);
    }
    cout << "line_two.size() : " << lines_two.size() << endl;

    // HoughLinesP(image_sobel, lines_2, rho, theta, hough_threshold, minLineLength, maxLineGap);
    // for (int i=0; i<lines.size(); i++){
    //     Vec4i L2 = lines_2[i];
    //     line(image_empty_2, Point(L2[0],L2[1]), Point(L2[2],L2[3]), Scalar(0,0,255), 1, LINE_AA);
    // }
    cout << "2222222222222222222222" << endl;


    // 8. 검출한 line 의 기울기를 구하고, 기울기가 10도 이상 낮은 직선을 filtering 합니다
    double slope_threshold = 0.005;
    double slope;
    vector<Vec4i> new_lines;
    vector<double> slopes;

    for(int i=0; i<lines_two.size(); i++){
        Vec4i L_new = lines_two[i];
        int x1 = L_new[0];
        int y1 = L_new[1];
        int x2 = L_new[2];
        int y2 = L_new[3];

        if (x2 - x1 == 0){
            slope = 50;
        }
        else 
            slope = (double)((y2 - y1) / (double)(x2 - x1));

        if (fabs(slope) < slope_threshold){
            slopes.push_back(slope);
            new_lines.push_back(L_new); // 우리가 원하는 정지선은 new_lines에 담긴다
            cout << "x1, y1, x2, y2 = " << x1 << " " << y1 << " " << x2 << " " << y2 << " " << endl;
        }
    }
    cout << "2222222222222222222222" << endl;


    // 9. 기울기가 0.2보다 직선을 검출한 그림을 imshow 합니다
    Mat image_empty_3(Size(image_ori.cols, image_ori.rows/2), CV_8UC3, Scalar(0,0,0));
    for (int i=0; i<new_lines.size(); i++){
        Vec4i L_three = lines_two[i];
        line(image_empty_3, Point(L_three[0],L_three[1]), Point(L_three[2],L_three[3]), Scalar(255,255,255), 1, CV_AA);
    }
    cout << "line_slope.size() : " << new_lines.size() << endl;
    
    for (int i=0; i<slopes.size(); i++){
        cout << "slopes : " << slopes[i] << " ";
    }

    srcRGB = image_empty_3;
    resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    if (new_lines.size()>5){
        cout << "return valuse is stop in passing_master.cpp" << endl;
        return "stop";
    }
    
    cout << "return value is go in passing_master.cpp" << endl;
    return "go";
    cout << "2222222222222222222222" << endl;
    //namedWindow("image");
    //imshow("image", image);
    //imshow("image_blur", image_blur);
    //imshow("image_canny", image_canny);
    //imshow("image_sobel", image_sobel);
    //imshow("image_hough", image_empty);
    //imshow("image_eroded", dilated);
    //imshow("image_hough_2", image_empty_2);
    //imshow("image_slope", image_empty_3);
    cout << "cols, rows : " << image_empty.cols << ", " << image_empty.rows << endl;
    //waitKey(0);

    //srcRGB = image_empty_3;
    //resize(srcRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    return 0;

}

Mat filterColors_2(Mat &input) {
    int range_count = 0;

//    Scalar red(0, 0, 255);
//    Scalar blue(255, 0, 0);
    Scalar yellow(0, 255, 255);

    Mat rgb_color = Mat(1, 1, CV_8UC3, yellow);
    Mat hsv_color;

    cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);

    int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
    int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
    int value = (int)hsv_color.at<Vec3b>(0, 0)[2];

    int low_hue = hue - 10;
    int high_hue = hue + 10;

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

    Mat img_hsv;

    //HSV로 변환
    cvtColor(input, img_hsv, COLOR_BGR2HSV);

    //지정한 HSV 범위를 이용하여 영상을 이진화
    Mat img_mask1, img_mask2;
    inRange(img_hsv, Scalar(low_hue1, 20, 20), Scalar(high_hue1, 255, 255), img_mask1);
    if (range_count == 2) {
        inRange(img_hsv, Scalar(low_hue2, 20, 20), Scalar(high_hue2, 255, 255), img_mask2);
        img_mask1 |= img_mask2;
    }

   //morphological opening 작은 점들을 제거
   erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
   dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
//
//    //morphological closing 영역의 구멍 메우기
//    dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
//    erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    input = img_mask1;
    return input;
}

int contour_size(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh){
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);

    Mat filter;
    filter = filterColors_2(srcRGB);

    //Find contours 
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(filter, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
    // imshow("findContours", filter);
    // waitKey(0);
    cout << "contours.size() : " << contours.size() << endl;
    int contour_size=0;
    for (size_t i = 0; i < contours.size(); i++) {
        cout << "contourArea" << " " << i << " " << contourArea(contours[i]) << endl;
        contour_size = contour_size < contourArea(contours[i]) ? contourArea(contours[i]) : contour_size;
    }
    cout << "max size : " << contour_size << endl;
    return contour_size;
}


#ifdef __cplusplus
}
#endif
