//
// Created by linqi on 16-3-23.
//

#ifndef MY_JNI_MYSTRUCT_H
#define MY_JNI_MYSTRUCT_H


#include "iostream"
#include "vector"
#include "string"
#include "opencv2/opencv.hpp"




using namespace cv;
using namespace std;

namespace mystruct
{

    class HomIntMat
    {
    public:
        inline HomIntMat()
        {
            index = -1;
        }
        inline HomIntMat(Mat &homography,int _index)
        {
            Homography = homography;
            //if index < 0,it means the homography is empty!
            index = _index;
        }

    public:
        Mat Homography;
        int index;
    };

    class HomMats
    {
    public:
        inline HomMats(){}
        inline HomMats(Mat homography,Mat &_srcImage)
        {
            Homography = homography;
            srcImage = _srcImage;
        }

    public:
        Mat Homography;
        Mat srcImage;
    };

    class HomImageMat
    {
    public:
        inline HomImageMat(){}
        inline HomImageMat(float *homography,Mat &_srcImage)
        {
            memcpy(Homography,homography, sizeof(homography)/ sizeof(float));
            srcImage = _srcImage;
        }

    public:
        float Homography[9];
        Mat srcImage;
    };

    class fHomography
    {
    public:
        inline fHomography(){}
        inline fHomography(float *homography)
        {
            memcpy(Homography,homography, sizeof(homography)/ sizeof(float));
        }

    public:
        float Homography[9];
    };

}


#endif //MY_JNI_MYSTRUCT_H
