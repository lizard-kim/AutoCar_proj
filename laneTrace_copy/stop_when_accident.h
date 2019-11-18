#include "opencv2/opencv.hpp" 
#include <iostream>  
#include <string> 

using namespace cv;

void setLabel(Mat& image, string str, vector<Point> contour);
void Coloring(Mat &layer, Scalar color);
void MakeLimit(int& low_hue, int& low_hue1, int &low_hue2, int &high_hue, int &high_hue1, int &high_hue2, int &range_count);
