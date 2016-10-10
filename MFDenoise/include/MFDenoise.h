//
// Created by linqi on 16/7/25.
//

#ifndef MFD_MFD_H
#define MFD_MFD_H

#include "MutGetHomography.h"
#include "MyStruct.h"

class MFDenoise {
public:
    MFDenoise();
    ~MFDenoise();

public:
    int process(Mat & _outMat, int mode = RANSAC);
    bool setMode(int homoMethod = RANSAC);
    bool initData(vector <Mat>& srcImages,vector <Mat>& grays);
    bool Test2Pics(Mat & _outMat);
private:
    Mat mWarpPerImage;
    Mat mDstImage;
    Mat mStandar;
    vector <Mat> m_images;
    vector <Mat> m_grays;
    double work_begin;
    double work_end;
    float mWidth;
    float mHeight;
    int HomoMethod;

private:
    void workBegin();
    void workEnd(char* module_name = "null");
};


#endif //MFD_MFD_H
