#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

//koo
int histogram_backprojection(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);


#ifdef __cplusplus
}
#endif



#endif //LANEDETECTION_H

