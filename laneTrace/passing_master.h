#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

//koo
bool pixel_detector(Mat image);
Mat pre_histogram_backprojection(unsigned char* srcBuf, int iw, int ih);
char* histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
char* stop_line_detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
int contour_size(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

#ifdef __cplusplus
}
#endif



#endif //LANEDETECTION_H

