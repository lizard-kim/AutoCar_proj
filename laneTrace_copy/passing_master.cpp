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

Mat pre_histogram_backprojection(unsigned char* srcBuf, int iw, int ih){
    
    //Mat srcImage(ih, iw, CV_8UC3, srcBuf);
    Mat srcImage = imread("overroad2.jpg", IMREAD_COLOR); // 이게 기준값!
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
    return hist;
    //passing->hist
    //passing pre;
    //struct pass pre;
}

char* histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, Mat hist)
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
        return;//fail
    cout << "이미지의 크기는 : " << srcImage.cols << " " << srcImage.rows << endl;
    resize(srcImage, srcImage, Size(srcImage.cols/5, srcImage.rows/5));
   
    cvtColor(srcImage, hsvImage, COLOR_BGR2HSV); //히스토그램은 밝기값을 통해 계산하기 때문에
												 //RGB 영상을 HSV 영상으로 바꾼 후 H 채널만 분리하도록 한다.

    Mat hist = pre_histogram_backprojection(srcBuf, iw, ih); // 여기서 pre_histogram 함수 사용
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
        return "left"; //left
    }
    else if(answer == 1 && answer2 == 1 && answer3 == 0){
        return "right"; //right
    }
    else return "fail"; //fail
}

char* stop_line_detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{  
    //Mat image_ori = imread("stop_line.jpg"); // 7개
    Mat image_ori(ih, iw, CV_8UC3, srcBuf);
    //Mat image_ori = imread("stop_line_2.jpg"); // 7개
    //Mat image_ori = imread("stop_line_3.jpg"); // 24개 - 가까워지면
    Mat image(ih, iw, CV_8UC3);

    resize(image_ori, image_ori, Size(320, 180), 0, 0, CV_INTER_LINEAR);

    // 1. grayscale로 바꾸어 줍니다
    cvtColor(image_ori, image, CV_RGB2GRAY);

    // 2. local averaging을 실행합니다. 이것은 gaussian blur와는 다릅니다
    Mat image_blur;
    blur(image, image_blur, Size(5,5));

    // 3. canny edge 함수를 사용해서 외곽선을 추출합니다
    Mat image_canny;
    Canny(image_blur, image_canny, 50, 150);

    // 3-2. sobel filter 함수를 사용해서 외곽선을 추출합니다
    Mat image_sobel;
    Sobel(image_blur, image_sobel, CV_8U, 0, 1);

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

    HoughLinesP(image_canny, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);
    for (int i=0; i<lines.size(); i++){
        Vec4i L = lines[i];
        line(image_empty, Point(L[0],L[1]), Point(L[2],L[3]), Scalar(0,0,255), 1, LINE_AA);
    }
    cout << "line.size() : " << lines.size() << endl;
    

    // 6. 침식, 팽창 연산을 통해 검출된 직선을 굵게 만든다
    Mat eroded, dilated, dilated_gray; 
    Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    //erode(image_empty, eroded, mask, cv::Point(-1, -1), 3);
    dilate(image_canny, dilated, mask, cv::Point(-1, -1), 3);
    //cvtColor(dilated, dilated_gray, COLOR_BGR2GRAY);


    // 7. 다시 허프변환을 통해 외곽선 중 직선을 검출합니다
    HoughLinesP(dilated, lines_two, rho, theta, hough_threshold, minLineLength, maxLineGap);
    for (int i=0; i<lines_two.size(); i++){
        Vec4i L_two = lines_two[i];
        line(image_empty_2, Point(L_two[0],L_two[1]), Point(L_two[2],L_two[3]), Scalar(0,0,255), 1, LINE_AA);
    }
    cout << "line_two.size() : " << lines_two.size() << endl;

    // HoughLinesP(image_sobel, lines_2, rho, theta, hough_threshold, minLineLength, maxLineGap);
    // for (int i=0; i<lines.size(); i++){
    //     Vec4i L2 = lines_2[i];
    //     line(image_empty_2, Point(L2[0],L2[1]), Point(L2[2],L2[3]), Scalar(0,0,255), 1, LINE_AA);
    // }

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

    // 9. 기울기가 0.2보다 직선을 검출한 그림을 imshow 합니다
    Mat image_empty_3(Size(image_ori.cols, image_ori.rows/2), CV_8UC3, Scalar(0,0,0));
    for (int i=0; i<new_lines.size(); i++){
        Vec4i L_three = lines_two[i];
        line(image_empty_3, Point(L_three[0],L_three[1]), Point(L_three[2],L_three[3]), Scalar(255,255,255), 1, LINE_AA);
    }
    cout << "line_slope.size() : " << new_lines.size() << endl;
    
    for (int i=0; i<slopes.size(); i++){
        cout << "slopes : " << slopes[i] << " ";
    }

    if (new_lines.size()>10){
        return "stop";
    }

    return "go";
    
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
    waitKey(0);
    return 0;

}

#ifdef __cplusplus
}
#endif
