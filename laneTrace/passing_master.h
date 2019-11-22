#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

//koo
bool pixel_detector(Mat image, char* order);
Mat pre_histogram_backprojection(unsigned char* srcBuf, int iw, int ih);
char* histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, Mat hist);
char* stop_line_detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

#ifdef __cplusplus
}
#endif



#endif //LANEDETECTION_H

