#include "laneDetection.h"

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
// 영상에서 선분을 추출하여 lines에 저장
void findLines(Mat &image, vector<Vec4i> &lines) {
    // Hough Transform 파라미터
    float rho = 1; // distance resolution in pixels of the Hough grid
    float theta = 1 * PI / 180; // angular resolution in radians of the Hough grid
    int hough_threshold = 15;    // minimum number of votes(intersections in Hough grid cell)
    float minLineLength = 15; //minimum number of pixels making up a line
    float maxLineGap = 2;   //maximum gap in pixels between connectable line segments

    // 직선 검출 (Hough Transform)
    HoughLinesP(image, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);
}


// 흑백 이진 영상에서 오브젝트 경계를 검출
void findBoundary(Mat &input) {
    vector< vector<Point> > contours;
    findContours(input, contours, RETR_LIST, CHAIN_APPROX_NONE);
    input = Mat::zeros(input.rows, input.cols, CV_8UC1);
    drawContours(input, contours, -1, 255, 2);
}


// sliding window 방식으로 곡선 차선 검출
void searchLine(const Mat& frame, vector<Vec2i> &line_points, int pivot_x, int pivot_y, int gap) {
    // 검출된 직선들을 저장할 벡터
    vector<Vec4i> lines;
    int width = frame.cols;
    int height = frame.rows;
    // 검색할 상위 영역 프레임 추출
    int x_gap = 300; // 위로 슬라이딩하여 검색할 영역의 좌우 크기
    int x_min = pivot_x - x_gap; if (x_min < 0) x_min = 0;
    int x_max = pivot_x + x_gap; if (x_max > width) x_max = width;
    int y_min = pivot_y - gap;
    int y_max = pivot_y;
    // 프레임 끝까지 탐색되면 종료
    if (y_min < 0) return;
    // 검색할 프레임 자르기
    Mat search_frame = frame(Range(y_min, y_max), Range(x_min, x_max));
    // 추출한 프레임에 대해 선분 검색
    findLines(search_frame, lines);

    // 선분 검출이 되면, 해당 선분의 상단점을 차선 위 점으로 추가
    int x1, y1, x2, y2, x_temp, y_temp;
    if (!lines.empty()) {
        // 인식된 점들 중 피벗과 이어진 점 추출
        for (int i = 0; i < (int)lines.size(); i++) {
            // 검출된 선분의 시작점과 끝점 좌표
            x1 = lines[i][0], y1 = lines[i][1], x2 = lines[i][2], y2 = lines[i][3];
            // 점 두 개의 순서가 바뀌었으면 서로 교환
            if (y1 < y2) {
                x_temp = x2; y_temp = y2;
                x2 = x1; y2 = y1;
                x1 = x_temp; y1 = y_temp;
            }
            // 피벗과 가까운 선 추출
            if (abs(pivot_x - (x_min + x1)) <= 5) {
                line_points.push_back(Point2i(x_min + x1, y_min + y1));
                line_points.push_back(Point2i(x_min + x2, y_min + y2));
                searchLine(frame, line_points, x_min + x2, y_min, gap);
                break;
            }
        } return; // 피벗과 이어진 선분이 없으면 종료
    } else return; // 검출된 선분이 없으면 종료
}



double laneDetection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh) {
    // 그레이스케일로 frame에 저장
    Mat frame;

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);

    cvtColor(srcRGB, frame, CV_BGR2GRAY);
    double angle;

    // 입력 영상의 가로, 세로 길이
    int width = frame.cols;
    int height = frame.rows;
//    float width = iw;
//    float height = ih;

    cv::Rect myROI(0, (int)((float)height*0.5), width, (int)((float)height*0.5)); // (x,y,w,h)
    // ROI 영역으로 자르기
    frame = frame(myROI);
    int new_width = frame.cols;
    int new_height = frame.rows;

    // Adaptive 이진화 및 잡음제거
    medianBlur(frame, frame, 5);
    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 179, -30);
    erode(frame, frame, getStructuringElement(MORPH_RECT, Size(5, 3)));
    dilate(frame, frame, getStructuringElement(MORPH_RECT, Size(5, 5)));

    // warping
    float width_ratio = 0.25; // 사다리꼴의 상단과 하단 간의 비율
    float height_ratio = 0.8; // 밑변과 높이 간의 비율
    // Warping 전의 이미지 상의 좌표
    vector<Point2f> corners(4);
    corners[0] = Point2f(0, 0); // left top
    corners[1] = Point2f(new_width, 0); // right top
    corners[2] = Point2f(0, new_height); // left bot
    corners[3] = Point2f(new_width, new_height); // right bot
    new_height = (int)((float)new_width*height_ratio);
    Size warpSize(new_width, new_height);
    Mat warpframe(warpSize, frame.type());
    // Warping 후의 좌표
    vector<Point2f> warpCorners(4);
    warpCorners[0] = Point2f(0, 0);
    warpCorners[1] = Point2f(warpframe.cols, 0);
    warpCorners[2] = Point2f((float)warpframe.cols*((1-width_ratio)/2), warpframe.rows);
    warpCorners[3] = Point2f((float)warpframe.cols*(width_ratio+(1-width_ratio)/2), warpframe.rows);
    // Transformation Matrix 구하기
    Mat trans = getPerspectiveTransform(corners, warpCorners);
    // Warping
    warpPerspective(frame, warpframe, trans, warpSize);

    // 경계 영역 추출
    findBoundary(warpframe);
    // 곡선 차선 검출해서 line_points 에 저장
    int slide_num = 10;
    int gap = (int)((float)new_height/(float)slide_num); // 각 슬라이드의 높이
    vector<Vec2i> line_points; // 직선 위 점들을 저장할 벡터
    vector<Vec4i> lines;
    // 맨 처음 피벗 차선 추출
    Mat pivot_frame = warpframe(Range(new_height - gap, new_height), Range(0, new_width));
    // 첫 프레임에 대하여 피벗 차선 검출
    findLines(pivot_frame, lines);
    // 차선이 검출된 경우, 모든 피벗 차선에 대하여 위로 올라가며 가장 긴 차선 검출
    if (!lines.empty()) {
        int x1, y1, x2, y2, x_temp, y_temp;
        // 일정 길이 이상의 차선을 검출할 때까지 반복
        for (int i = 0; i < (int)lines.size(); i++) {
            x1 = lines[i][0]; y1 = lines[i][1]; x2 = lines[i][2]; y2 = lines[i][3];
            if (y1 < y2) {
                x_temp = x2; y_temp = y2;
                x2 = x1; y2 = y1;
                x1 = x_temp; y1 = y_temp;
            }
            line_points.push_back(Point2i(x1, new_height - gap + y1));
            line_points.push_back(Point2i(x2, new_height - gap + y2));
            searchLine(warpframe, line_points, x2, new_height - gap + y2, gap);
            // 충분히 긴 직선이 검출되면 해당 선을 추종 차선으로 선택, 아니면 다시 실행
            if (line_points.size() >= 5) break;
            else {
                line_points.clear();
                continue;
            }
        }
    }

    if (!line_points.empty()) {
        cvtColor(warpframe, warpframe, COLOR_GRAY2BGR);
        for (int i = 0; i < (int)line_points.size() - 1; i++) {
            line(warpframe, line_points[i], line_points[i+1], Scalar(0, 0, 255), 2);
        }
        double slope = 0, slope_sum = 0;
//        for (int i = 0; i < (int)line_points.size() - 1; i++) {
//            // tan(theta) = (x2 - x1) / (y1 - y2)
//            slope = (double)(line_points[i+1][0] - line_points[i][0]) / (double)(line_points[i][1] - line_points[i+1][1]);
//            slope_sum += slope;
//        }
//        slope = slope_sum / (double)(line_points.size() - 1);
        slope = (double)(line_points[(int)line_points.size() - 1][0] - line_points[0][0]) / (double)(line_points[0][1] - line_points[(int)line_points.size() - 1][1]);
        angle = atan(slope) * 180 / PI;
        // 차선과의 거리에 따른 주행보정
        int diff = line_points[0][0] - (new_width / 2); // dif = 차선 - 차량 위치
        if (diff*angle >= 0) {
            angle = angle * (1 + ((double)abs(diff) - (double)new_width/8)/40);
        }
        else {
            angle = angle * (1 + ((double)new_width/8 - (double)abs(diff))/40);
        }
        // cout << "angle : " << angle << endl;
    }
    else {
        // cout << "Lane not detected ㅜㅜ" << endl;
    }

    // srcRGB = warpframe;
    // cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    return angle;
}

#ifdef __cplusplus
}
#endif
