#include "laneDetection.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cstring>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc_c.h>


#include <cmath>
#include <vector>

#define PI 3.1415926

using namespace std;
using namespace cv;

#ifdef __cplusplus
extern "C" {
#endif

// 흑백 이진 영상에서 오브젝트 경계를 검출
void findBoundary(Mat &input) {
    vector<vector<Point> > contours;
    findContours(input, contours, RETR_LIST, CHAIN_APPROX_NONE);
    input = Mat::zeros(input.rows, input.cols, CV_8UC1);
    drawContours(input, contours, -1, 255, 2);
}



// 3차식의 계수와 원하는 y 좌표를 대입하면 steer 반환
double getSteerwithCurve(vector<double> &ans, int y) {

    vector<double> diff; // 미분한 값 계수 저장

    int num = ans.size();
    for (int i = 0; i < num; i++)
    {
        diff.push_back((num - (i + 1))*ans[i]); // 미분 실행
    }

    double gradient = 0.0; //기울기 initial

    for (int i = 0; i < (int)diff.size() - 1; i++)
    {
        gradient += diff[i] * (pow(y, (diff.size() - (i + 2)))); //(y,x) 점을 미분값에 대입후 합... 기울기가 나옴
    }

    double descent_deg = -atan(gradient)*180/CV_PI; // 세타값 (degree 환산)
    double descent_rad = -atan(gradient); // 세타값 (radian 환산)

    return descent_deg;
}



// 특정 y 좌표의 점 추출
void getCurvePoint(vector<double> &ans, int y, Point2i &destination) {
    double coef[900];
    int ans_size = ans.size();

    // 벡터보다 배열에 접근하는게 훨 빠르기 때문에 후에 계산을 위하여 배열에 넣어주었다.
    for (int i = 0; i < ans_size; i++) {
        coef[i] = ans.at(i);
    }

    double x = 0;
    for (int j = 0; j < ans_size; j++) {
        x += coef[j] * pow(y, ans_size - 1 - j);
    }

    destination = Point2i(x, y);
}




// 점의 방정식을 곡선으로 근사
void curveFitting(Mat &input, Mat &output, vector<Point2d> &route, vector<double> &ans, int start, int end) {

    if (route.size() <= 3) {
        cerr << "Lack of Points to fitting !" << endl;
        return;
    }
    cout << "a" << endl;
    vector<pair<double, double> > v;

    for (size_t i = 0; i < route.size(); i++) {
        v.push_back(make_pair(route[i].x, route[i].y));
    }
    int n = v.size();
    Mat A;

    cout << "aa" << endl;
    // A를 완성시키는 이중 for문
    // 3 차 방정식으로 근사한다. 이 숫자 바꿔 주면 n-1 차 방정식까지 근사 가능.
    for (size_t i = 0; i < v.size(); i++) {
        vector<double> tmp;
        for (int j = 3; j >= 0; j--) {
            double x = v[i].second; // x 하고 y 를 바꾸기로 함. 그래야 함수가 만들어지니까
            tmp.push_back(pow(x,j));
        }
        A.push_back(Mat(tmp).t());
    }

    cout << "aaa" << endl;
    Mat B; // B 에는 y 좌표들을 저장한다.
    vector<double> tmp;
    for (int i = 0; i < n; i++) {
        double y = v[i].first; // x 하고 y 바꾸기로 함. 그래야 함수가 만들어지니까
        tmp.push_back(y);
    }
    B.push_back(Mat(tmp));

    // X = invA * B; // X = A(-1)B
    Mat X = ((A.t() * A).inv()) * A.t() * B;
    cout << "aaaa" << endl;
    // X 에서 차례대로 뽑아내서 ans 에 담는다.
    // 몇차 방정식으로 근사할껀지 정할때 건드려줘야 되는부분, 3차 방정식으로 근사할꺼면 4 ㅇㅇ.
    for (int i = 0; i < 4; i++) {
        ans.push_back(X.at<double>(i, 0));
    }
    // 앞에서 부터 0인지 검사해서 0이면 지운다. 차수 조절을 위하여. 가운데 0은 안지워짐.
    while (!ans.empty()) {
        if (ans.at(0) == 0) ans.erase(ans.begin());
        else break;
    }
    double coef[900];
    // ans_size - 1 차 다항식이 만들어진거고, ans_size만큼 a,b,c,d,e `` 가 있다.
    int ans_size = ans.size();

    // 벡터보다 배열에 접근하는게 훨 빠르기 때문에 후에 계산을 위하여 배열에 넣어주었다.
    for (int i = 0; i < ans_size; i++) {
        coef[i] = ans.at(i);
    }
    cout << "aaaaa" << endl;
    // 계산된 곡선 그리기
    output = input.clone();
    for (int i = end; i >= start; i -= 1) {
        double x1 = 0, x2 = 0;
        for (int j = 0; j < ans_size; j++) {
            x1 += coef[j] * pow(i, ans_size - 1 - j);
            x2 += coef[j] * pow(i - 1, ans_size - 1 - j);
        }
        line(output, Point(x1, i), Point(x2, i - 1), Scalar(0, 0, 255), 2);
    }
    cout << "aaaaaa" << endl;
}





void filterColors(Mat &input) {
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

//    //morphological opening 작은 점들을 제거
//    erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
//    dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
//
//    //morphological closing 영역의 구멍 메우기
//    dilate(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
//    erode(img_mask1, img_mask1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    input = img_mask1;
}


// int connectedCom(Mat &frame, Mat &labels, Mat &stats, Mat centroids) {
//     int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);
//     return cnt;


// }



void laneDetection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, double *output_angle, double *output_ratio) {
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);

    Mat frame = srcRGB;
    //cvtColor(srcRGB, frame, CV_BGR2GRAY);

    filterColors(frame);

    // 입력 영상의 가로, 세로 길이
    int width = frame.cols;
    int height = frame.rows;
//    float width = iw;
//    float height = ih;

    cv::Rect myROI(0, (int)((float)height*0.45), width, (int)((float)height*0.55)); // (x,y,w,h)
    // ROI 영역으로 자르기
    frame = frame(myROI);
    int new_width = frame.cols;
    int new_height = frame.rows;

//    // Adaptive 이진화 및 잡음제거
//    cvtColor(frame, frame, COLOR_BGR2GRAY);
//    medianBlur(frame, frame, 5);
//    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 51, -30);
//    erode(frame, frame, getStructuringElement(MORPH_RECT, Size(5, 3)));
//    dilate(frame, frame, getStructuringElement(MORPH_RECT, Size(5, 5)));

    // warping
    float width_ratio = 0.35; // 사다리꼴의 상단과 하단 간의 비율
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


    double steer = 0;
    double ratio = 1;
    double lane_width = (double)new_width * width_ratio * 0.5;
    vector<Point2d> line_points;
    vector<double> ans;
    int lane_type = 0;
    int lane_idx = -1;


    /// 1. 최소 기준으로 차선 후보집합 선별
    // 외곽선 검색
    vector<vector<Point> > contours;
    Point2d lane_top(new_width, new_height), lane_bot(0, 0);
    findContours(warpframe, contours, RETR_LIST, CHAIN_APPROX_NONE);
    int bot_dx = 10000; // 차선 후보 시작점 좌표 차이
    for (size_t i = 0; i < contours.size(); i++) {
//        stats.push_back(make_pair(Point2d(0, height), Point2d(0, 0)));
        if (contours[i].size() <= 200) continue;

        vector<Point2d> line_temp; line_temp.clear();
        for (size_t j = 0; j < contours[i].size(); j++) {
            Point2d point((double)contours[i][j].x, (double)contours[i][j].y);
            if (lane_top.y > point.y) {
                lane_top.x = point.x; lane_top.y = point.y;
            }
            if (lane_bot.y < point.y) {
                lane_bot.x = point.x; lane_bot.y = point.y;
            }
            line_temp.push_back(point);
        }

        /// 2. 중앙부에 가장 가까운 선을 우선 선별하여 차선으로 정의
        int dx = lane_bot.x - new_width/2;
        if (abs(dx) < abs(bot_dx)) {
            bot_dx = dx;
            lane_idx = i;
            line_points.clear();
            line_points.resize((int)line_temp.size());
            copy(line_temp.begin(), line_temp.end(), line_points.begin());
        }
        cout << "bot_dx : " << bot_dx << endl;
    }
    // 외곽선 그리기
    Mat frame_show = Mat::zeros(new_height, new_width, CV_8UC3);
    drawContours(frame_show, contours, -1, Scalar(255, 255, 255), -1);
    if (lane_idx != -1) drawContours(frame_show, contours, lane_idx, Scalar(50, 200, 50), -1);

    cout << "contours.size() : " << contours.size() << endl;
    cout << "line_points.size() : " << line_points.size() << endl;



    /// 3. 왼쪽/오른쪽 차선 구분 및 목표점 정의
    // 차선이 인식되지 않은 경우
    if (lane_idx == -1) {
        cout << "lane not detected !!!!!" << endl;
        ratio = 0;
    }
    // 차선이 인식된 경우
    else {
        cout << "fuck1" << endl;
        // 1) 차선 타입 정의
        if (bot_dx >= 0) lane_type = 1; // 오른쪽 차선
        else lane_type = -1; // 왼쪽 차선
        cout << "fuck2" << endl;
        curveFitting(frame_show, frame_show, line_points, ans, lane_top.y, lane_bot.y);
        cout << "fuck3" << endl;
        // 3) 목적지 방향 및 조향 계산
        // *무게중심 좌표
        Point2i cen = Point2i((lane_top.x + lane_bot.x)/2, (lane_top.y + lane_bot.y)/2);
        Point2i cp, des;
        double angle;
        // **무게중심에 해당하는 곡선 위 좌표
        getCurvePoint(ans, cen.y, cp);
        angle = getSteerwithCurve(ans, cen.y);
            cout << "fuck4" << endl;
        // ***곡선 차선에 대한 차선 타입 보정
        if (angle >= 45) lane_type = -1;
        else if (angle <= -45) lane_type = 1;
        // ****목적지 좌표 계산
        // 왼쪽 차선일 경우
        if (lane_type == -1) {
            des.x = cp.x + lane_width/cos(angle*CV_PI/180);
            des.y = cp.y;
        }
        // 오른쪽 차선일 경우
        else if (lane_type == 1) {
            des.x = cp.x - lane_width/cos(angle*CV_PI/180);
            des.y = cp.y;
        }
        circle(frame_show, des, 5, Scalar(250, 150, 100), -1);
    cout << "fuck5" << endl;
        // *****조향각 계산
        // 방향벡터 계산
        Point2d direction(des.x - (double)new_width/2, des.y - new_height);
        double direction_norm = sqrt((pow(direction.x, 2) + pow(direction.y, 2)));
        // direction =  direction * 50 / direction_norm;
        direction.x = direction.x * 50 / direction_norm;
        direction.y = direction.y * 50 / direction_norm;
        line(frame_show, Point2i(new_width/2, new_height), Point2i((int)direction.x + new_width/2, (int)direction.y + new_height), Scalar(150, 100, 200), 2);
        // 조향 계산
        steer = atan(direction.x / abs(direction.y)) * 180 / CV_PI;
        cout << "steer : " << steer << endl;
        // 최대 조향을 벗어날 경우 조향 제한
        if (abs(steer) >= 45) {
            cout << "steer out of range !!!" << endl;
            steer = 0;
            ratio = -1;
            // if (steer < 0) steer = -45;
            // else steer = 45;
        }
    }
    cout << "fuck6" << endl;
    ostringstream temp;
    temp << steer;
    string str = temp.str();
    putText(frame_show, "steer : " + str, Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255));
    

    srcRGB = frame_show;
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    cout << "fuck7" << endl;

    *output_angle = steer;
    if (ratio > 0) ratio = cos(steer*180/CV_PI);
    *output_ratio = ratio;
        cout << "fuck8" << endl;
}






#ifdef __cplusplus
}
#endif
