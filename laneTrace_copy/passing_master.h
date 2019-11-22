#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

//koo
char* histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, Mat hist);
Mat pre_histogram_backprojection(unsigned char* srcBuf, int iw, int ih);
char* stop_line_detection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

/*
class Passing{
public:
    Mat hist;
    int hist_check = 0;
}
*/
#ifdef __cplusplus
}
#endif



#endif //LANEDETECTION_H

