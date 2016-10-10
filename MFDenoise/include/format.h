#ifndef FORMAT_H
#define FORMAT_H
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include "vector"
#include <string>
#include "iostream"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

typedef unsigned char byte;

class Format
{
public:
    Format(vector <string> &_pathVec,int _width,int _height);
    ~Format();
public:
    bool decode(vector <Mat> &_outImg);
    bool getYchannel(vector <Mat> &_outImg);//get Y channel from yuv420sp
private:
    bool yvu420ps2rgba(byte yuv420sp[],Mat &out);//yuv420sp to rgba
//    Mat YUV420_To_Mat(unsigned char* pYUV420, int width, int height);
//    bool YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u,
//                         unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y);
//    void yv12Tnv12(byte src[],byte des[]);
private:
    int buffSize;
    vector <string> pathVec;
    int width;
    int height;
};



#endif // FORMAT_H

