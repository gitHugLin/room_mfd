//
// Created by linqi on 16/7/25.
//

#include "../include/MFDenoise.h"

#define LOGE printf

MFDenoise::MFDenoise() {
    work_begin = 0;
    work_end = 0;
    HomoMethod = RANSAC;
    mWidth = 0;
    mHeight = 0;
    mWarpPerImage = NULL;
}

MFDenoise::~MFDenoise() {

}

void MFDenoise::workBegin()
{
    work_begin = getTickCount();
}

void MFDenoise::workEnd(char* module_name)
{
    work_end = getTickCount() - work_begin;
    double Time = work_end /((double)getTickFrequency() )* 1000.0;
    LOGE("[%s] TIME = %lf ms \n", module_name, Time);
}

bool MFDenoise::setMode(int homoMethod )
{
    bool ret = true;
    switch (homoMethod) {
        case RANSAC:
            HomoMethod = RANSAC;
            break;
        case RHO:
            HomoMethod = RHO;
            break;
        case LMEDS:
            HomoMethod = LMEDS;
            break;
        default:
            HomoMethod = RANSAC;
            ret = false;
            break;
    }
    return ret;
}

bool MFDenoise::initData(vector <Mat>& srcImages,vector <Mat>& grays){

    int length = srcImages.size();
    bool ret = true;
    if(6 != length) {
        return false;
    }
    mWidth = srcImages[0].cols;
    mHeight = srcImages[0].rows;
    m_images.clear();
    m_grays.clear();
    mStandar = srcImages[2];
    for(size_t i = 0; i < length; i++){
        m_images.push_back(srcImages[i]);
        m_grays.push_back(grays[i]);
    }
    return true;
}

int MFDenoise::process(Mat & _outMat, int mode){

    cout<<"MFDenoise process is begin!"<<endl;
    vector <HomIntMat> homIntMatVec;
    workBegin();
    MutGetHomography MTFeature(m_grays);
    MTFeature.setMode(HomoMethod);
    MTFeature.process(homIntMatVec);
    workEnd("MutGetHomography");

    HomMats mathom;
    vector <HomMats> HomVec;
    HomVec.clear();
//    Mat eye = Mat::eye(3,3,CV_32F);
//    mathom.Homography = eye;
//    mathom.srcImage = m_images[2];
//    HomVec.push_back(mathom);
    for(int k = 0; k < 5; k++)
    {
        int index = k;
        if(k > 1)
            index = k + 1;
        for(int i = 0; i < 5; i++)
        {
            if(homIntMatVec[i].index == index )
            {
//                LOGE("Homography = %d",index);
//                cout<<endl;
//                double *temp = (double *)homIntMatVec[i].Homography.data;
//                for(int j = 0;j < 9;j++)
//                {
//                    LOGE("Mat = %lf",*(double*)(temp+j));
//                    cout<<endl;
//                }
                if(index == 3){
                    Mat eye = Mat::eye(3,3,CV_32F);
                    mathom.Homography = eye;
                    mathom.srcImage = m_images[2];
                    HomVec.push_back(mathom);
                    mathom.Homography = homIntMatVec[i].Homography;
                    mathom.srcImage = m_images[index];
                    HomVec.push_back(mathom);
                    break;
                }else {
                    mathom.Homography = homIntMatVec[i].Homography;
                    mathom.srcImage = m_images[index];
                    HomVec.push_back(mathom);
                    break;
                }
            }
            else if(homIntMatVec[i].index == -1)
            {
                LOGE("Homography not found.");
                cout<<endl;
                _outMat = NULL;//m_images[3];
                return -1;
            }
        }
    }
    double alpha = 0.5;
    mDstImage = mStandar;
    for (int i = 0; i < 6; i++) {
        warpPerspective(HomVec[i].srcImage,mWarpPerImage,HomVec[i].Homography,Size(mWidth, mHeight));
        double beta = 1 - alpha;
        addWeighted( mWarpPerImage, alpha,mDstImage, beta, 0.0,mDstImage);//图像混合的函数
    }
    imwrite("mfd.jpg",mDstImage);
    LOGE("addWeighted is finished.");
    cout<<endl;
    return 1;
}

bool MFDenoise::Test2Pics(Mat & _outMat){
    int index = 1;
    bool ret = true;
    GetHomography homography(m_grays[2], m_grays[index]);

    HomIntMat myIntMat;
    homography.setMode(HomoMethod);
    myIntMat.Homography = homography.getHomography();
//    double *temp = (double *)myIntMat.Homography.data;
//    for(int j = 0;j < 9;j++)
//    {
//        LOGE("Mat = %lf",*(double*)(temp+j));
//        cout<<endl;
//    }
    if (myIntMat.Homography.empty())
        cout<<"can not match these two pics!"<<endl;
    else
        cout<<"match these two pics successfully!"<<endl;
    double alpha = 0.5;
    for (size_t i = 0; i < 6; i++) {
        warpPerspective(m_images[index],mWarpPerImage,myIntMat.Homography,Size(mWidth, mHeight));
        double beta = 1 - alpha;
        addWeighted( mWarpPerImage, alpha,mDstImage, beta, 0.0,mDstImage);//图像混合的函数
    }
    imwrite("mfdTest.jpg",mDstImage);
    return ret;
}