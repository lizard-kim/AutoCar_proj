#include "laneDetection.h"

#include <iostream>

int main() {


    Mat input = imread("/Users/parkkyuyeol/Downloads/IMG_1130.jpg", IMREAD_COLOR);
    if (input.empty()) {
        cerr << "Image load failed" << endl;
        return 0;
    }

    laneDetection(input, 100, 100, input, 100, 100);

//    VideoCapture cap("/Users/parkkyuyeol/Desktop/무제 폴더 2/예선 영상_미션.mp4");
//    if (!cap.isOpened()) {
//        cerr << "Video open failed" << endl;
//        return 0;
//    }
//
//    double fps = cap.get(CAP_PROP_FPS);
//    int delay = cvRound(1000 / fps);
//
//    Mat table = Mat::zeros(200, 400, CV_8UC3);
//    Mat table_show, frame_show;
//
//    int distance = -1;
//
//
//    clock_t start, end;
//    double result = 0, sum = 0;
//    int timecount = 0;
//
//    Mat frame;
//    while (true) {
//        start = clock();
//
//        cap >> frame;
//        if (frame.empty()) break;
//
//        start = clock();
//        distance = laneDetection(frame);
//        end = clock();
//
//        if (distance == -1) {
//            cap.set(CAP_PROP_POS_FRAMES, cap.get(CAP_PROP_POS_FRAMES) +30);
//        } else cap.set(CAP_PROP_POS_FRAMES, cap.get(CAP_PROP_POS_FRAMES) + 10);
//
//
//        sum += (double)(end - start);
//        timecount ++;
//
//        if (timecount % 2 == 0) {
//            result = sum / 5; sum = 0;
//        }
//
//        table_show = table.clone();
//        putText(table_show, "Time(s) : " + to_string(result/CLOCKS_PER_SEC), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
//        putText(table_show, "distance : " + to_string(distance), Point(10, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
//        imshow("distance", table_show);
//
//    }

    return 0;
}