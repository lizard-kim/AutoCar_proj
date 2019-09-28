//
// Created by 박규열 on 24/09/2019.
//

#include "laneDetection.h"

#include <vector>


// 영상에서 선분을 추출하여 lines에 저장
int findLines(Mat &image) {
    int width = image.cols;
    int height = image.rows;

    auto* data = (uchar*)image.data;
    int count = 0;
    for (int x = 0; x < width; x++) {
        count = 0;
        for (int y = 0; y < height; y++) {
            if (data[y*width + x] == 255) count++;
        }
        if (count >= (int)(height/2)) {
            return x;
        }
        cout << "x, count, width, height : " << x << " " << count << " " << width << " " << height << endl;
    }
    return -1;
}


// 흑백 이진 영상에서 오브젝트 경계를 검출
void findBoundary(Mat &input) {
    vector<vector<Point>> contours;
    findContours(input, contours, RETR_LIST, CHAIN_APPROX_NONE);
    input = Mat::zeros(input.rows, input.cols, CV_8UC1);
    drawContours(input, contours, -1, 255, 5);
}


void searchLine(const Mat& frame, vector<Vec2i> &line_points, int pivot, int step, int gap, int slide_num, int new_width, int new_height) {
    vector<Vec4i> lines; // 검출된 직선들을 저장할 벡터

    // 검색할 상위 영역 프레임 추출
    int x_gap = 200; // 위로 슬라이딩하여 검색할 영역의 좌우 크기
    int x_min = (pivot - x_gap >= 0) ? pivot - x_gap : 0;
    int x_max = (pivot + x_gap <= new_width) ? pivot + x_gap : new_width;
    int y_min = new_height - (step + 1) * gap;
    int y_max = new_height - step * gap;
    Mat search_frame = frame(Range(y_min, y_max), Range(x_min, x_max));
    // 추출한 프레임에 대해 선분 검색
    int new_pivot = findLines(search_frame);
    cout << "pivot : " << pivot << " " << "new pivot : " << new_pivot << endl;
//    imshow("search", search_frame);
//    waitKey();
    // 선분 검출이 되면, 해당 선분의 상단점을 차선 위 점으로 추가
    if (new_pivot != -1) {
        cout << "pivot : " << pivot << "new pivot : " << new_pivot << endl;
        line_points.push_back(Point2i(x_min + new_pivot, y_min));
        if (step + 1 < slide_num)
            searchLine(frame, line_points, new_pivot, step + 1, gap, slide_num,  new_width, new_height);
        else return;
    } else return;
}



void laneDetection(Mat &srcRGB, int iw, int ih, Mat &dstRGB, int nw, int nh) {

    // 그레이스케일로 frame에 저장
    Mat frame;
    cvtColor(srcRGB, frame, CV_BGR2GRAY);

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
    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 151, -40);
    erode(frame, frame, getStructuringElement(MORPH_RECT, Size(10, 10)));
    dilate(frame, frame, getStructuringElement(MORPH_RECT, Size(20, 20)));

    // warping
    float width_ratio = 0.2; // 사다리꼴의 상단과 하단 간의 비율
    float height_ratio = 0.6; // 밑변과 높이 간의 비율
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

    imshow("beforewarp", frame);
    imshow("warp", warpframe);
    waitKey();

    cout << "fuck0" << endl;

    //findBoundary(warpframe);
    // 곡선 차선 검출해서 line_points에 저장
    int is_found = 0;
    int slide_num = 10;
    int gap = (int)((float)new_height/(float)slide_num); // 각 슬라이드의 높이
    vector<Vec2i> line_points; // 직선 위 점들을 저장할 벡터
    vector<Vec4i> lines;
    cout << new_width << " " << new_height << " " << gap << endl;
    cout << warpframe.cols << " " << warpframe.rows << " " << warpframe.type() << endl;
    cout << 0 << " " << new_width << " " << new_height - gap << " " << new_height << endl;
    Mat pivot_frame = warpframe(Range(new_height - gap, new_height), Range(0, new_width));
//    Mat pivot_frame = warpframe(Rect(0, new_height, new_width, gap));
    cout << "fuck0.5" << endl;

    vector<int> pivots;
    auto* data = (uchar*)pivot_frame.data;
    int count = 0;
    for (int x = 0; x < pivot_frame.cols; x++) {
        count = 0;
        for (int y = 0; y < pivot_frame.rows; y++) {
            if (data[y*pivot_frame.cols + x] == 255) count++;
        }
        if (count >= (int)(pivot_frame.rows/2)) {
            pivots.push_back(x);
        }
    }
    cout << "fuck1" << endl;
    if (!pivots.empty()) {
        cout << "fuck2" << endl;

        for (auto & pivot : pivots) {
            cout << "pivot : " << pivot << endl;
            line_points.push_back(Point2i(pivot, new_height - gap));
            searchLine(warpframe, line_points, pivot, 1, gap, slide_num, new_width, new_height);
            cout << "fuck4" << endl;
            // 충분히 긴 직선이 검출되면 해당 선을 추종 차선으로 선택, 아니면 다시 실행
            if (line_points.size() >= 3) {
                is_found = 1;
                break;
            }
            else {
                line_points.clear();
                continue;
            }
        }
    }
    cvtColor(warpframe, warpframe, COLOR_GRAY2BGR);
    for (int i = 0; i < (int)line_points.size() - 1; i++) {
        line(warpframe, line_points[i], line_points[i+1], Scalar(0, 0, 255), 20);
    }

    imshow("detect", warpframe);
    waitKey();
//    if (is_found) {
//        for (auto & point : line_points)
//            circle(warpframe, point, 5, Scalar(0, 0, 255), 2);
//        imshow("detect", warpframe);
//        waitKey();
//    }

//
//
//
//    for (int i = 0; i < (int)lines.size(); i++) {
//
//
//    }
//
//
//
//    // 한 슬라이드씩 올라가면서 차선 검색
//    for (int step = 0; step < slide_num; step++) {
//        Mat slide_frame = warpframe(Range(0, new_width),Range(new_height - (step + 1)*gap, new_height - step*gap));
//        findLines(slide_frame, lines[step]);
//        if (lines[step].size() != 0)
//
//
//    }
//
//
//
//
//
//
//    img(Range(100, 200), Range(100,200))
//
//
//
//
//    int slide_num = 10;
//    int slide_sum = 0;
//    int gap = (int)((float)new_height/(float)slide_num); // 각 슬라이드의 높이
//    vector<Vec4i> lines[slide_num]; // 매 슬라이드 별 line들을 저장할 벡터
//    auto* data = (uchar*)warpframe.data; // Mat 행렬 픽셀 데이터 접근을 위한 포인터
//    // 한 슬라이드씩 올라가면서 차선 검색
//    for (int step = slide_num; step>0; step--) {
//
//
//    }
//
//
//
//    int pivot_left = -1, pivot_right = -1;
//    // 한 slide 씩 올라가면서 차선 검색
//    for(int step = slide_num; step > 0; step--) {
//        // 중앙에서부터 왼쪽 방향으로 차선 검색 진행
//        for(int x = (int)((float)new_width/2); x>=0; x--) {
//            // 특정 x에 대해 세로선 흰 픽셀 개수의 합
//            slide_sum = 0;
//            for (int y = gap * step; y > gap * (step - 1); y++) {
//                if (data[new_width * y + x] == 255)
//                    slide_sum++;
//            }
//            // 만약 gap의 절반 이상 픽셀이 흰색이면, 차선 위 점으로 간주
//            if (slide_sum > gap * 0.5) {
//                pivot_left = x;
//                break;
//            }
//        }
//        // 중앙에서부터 오른쪽 방향으로 차선 검색 진행
//        for(int x = (int)((float)new_width/2); x<=new_width; x++) {
//            // 특정 x에 대해 세로선 흰 픽셀 개수의 합
//            slide_sum = 0;
//            for (int y = gap * step; y > gap * (step - 1); y++) {
//                if (data[new_width * y + x] == 255)
//                    slide_sum++;
//            }
//            // 만약 gap의 절반 이상 픽셀이 흰색이면, 차선 위 점으로 간주
//            if (slide_sum > gap * 0.5) {
//                pivot_right = x;
//                break;
//            }
//        }
//
//        // 양쪽 차선 모두 검출되면
//        if (pivot_left != -1 && pivot_right != -1) {
//
//
//
//
//        }
//
//
//
//
//    }
//
//
//    for(int x = (int)((float)new_width/2); x>0; x--) {
//        // 한 slide 씩 올라가면서 차선 검색
//        for(int step = slide_num; step > 0; step--) {
//            slide_sum = 0;
//            // 특정 x에 대해 세로선 흰 픽셀 개수의 합
//            for (int y = gap * step; y > gap * (step - 1); y++) {
//                slide_sum += (data[new_width * y + x] == 255) ? 1 : 0;
//            }
//            // 만약 gap의 절반 이상 픽셀이 흰색이면, 차선 위 점으로 간주
//            if (slide_sum > gap*0.5) {
//                left_point.push_back(x);
//                break;
//            }
//        }
//    }
//
//    for(int step = slide_num; step > 0; step--) {
//        slide_sum = 0;
//        for(int y = gap*step; y > gap*(step-1); y++)
//            slide_sum += (data[new_width*y + x] == 255) ? 1 : 0; // 특정 x에 대해 세로선 흰 픽셀 개수의 합
//
//
//
//
//    }
//
//    for (int y = 0; y < height; y++) {
//
//        // y번째 row에 대한 주소를 포인터에 저장한 후
//        uchar* pointer_input = frame.ptr<uchar>(y);
//        uchar sum = 0;
//        for (int x = 0; x < width; x++) {
//            sum +=
//
//            // row 포인터로부터 (x * 3 )번째 떨어져 있는 픽셀을 가져옵니다.
//            //0, 1, 2 순서대로 blue, green, red 채널값을 가져올 수있는 이유는 하나의 픽셀이 메모리상에 b g r 순서대로 저장되기 때문입니다.
//            uchar b = pointer_input[x * 3 + 0];
//            uchar g = pointer_input[x * 3 + 1];
//            uchar r = pointer_input[x * 3 + 2];
//
//            pointer_ouput[x] = (r + g + b) / 3.0;
//        }
//    }
//
//
//
//
//
//
//    // 관심영역 설정 (set ROI (X, Y, W, H)).
//    Rect rect(static_cast<int>(width*(1-rec_width)/2), static_cast<int>(height*(1-rec_height)), rec_width * width, rec_height * height);
//
//    // 관심영역 자르기 (Crop ROI).
//    input = input(rect);
//
//
//
//
//
//
//
//
//    Scalar lineColor = cv::Scalar(255,255,255);
//    Mat resRGB(ih, iw, CV_8UC3);
//
//    cv::Mat contours;
//    cv::Canny(srcRGB, contours, 125, 350);
//
//    std::vector<cv::Vec2f> lines;
//    cv::HoughLines(contours, lines, 1, PI/180, 80);
//
//    cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
//    //printf("Lines detected: %d\n", lines.size());
//
//    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
//
//    while (it!=lines.end())
//
//    {
//        float rho = (*it)[0];
//        float theta = (*it)[1];
//
//        if (theta < PI/4. || theta > 3.*PI/4.)
//
//        {
//            cv::Point pt1(rho/cos(theta), 0);
//            cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
//            cv::line(srcRGB, pt1, pt2, lineColor, 1);
//        }
//        else
//        {
//            cv::Point pt1(0,rho/sin(theta));
//            cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
//            cv::line(srcRGB, pt1, pt2, lineColor, 1);
//        }
//        //printf("line: rho=%f, theta=%f\n", rho, theta);
//        ++it;
//    }
//    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}




void showimage(const string& name, Mat &original) {
    Mat image_show;
    int key;
    resize(original, image_show, Size(480, 270));

    imshow(name, image_show);
    key = waitKey(1);
    if (key == 27) destroyAllWindows();
}


















