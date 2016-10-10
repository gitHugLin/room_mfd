//
// Created by linqi on 16-4-5.
//

#include "../include/OrbPatch.h"
#include "../include/MyPointSetRegistrator.h"
#include "log.h"
#include <unistd.h>
//#define  _DEBUG_MODE_
#define  _SUM_TIME_


OrbPatch::OrbPatch()
{
    origin = Point2f(0,0);
    mISO = 2;
}

OrbPatch::~OrbPatch()
{

}

//开始计时
void OrbPatch::workBegin()
{
    work_begin = getTickCount();
}

//结束计时
void OrbPatch::workEnd()
{
    work_end = getTickCount() - work_begin;
    cout<<"time = "<<work_end /((double)getTickFrequency() )* 1000.0<<"ms"<<endl;
}

void OrbPatch::createPatch(const int nFeatures,const int nlevels,const int edgeThreshold,int _descriptorSize)
{

    m_detector = MyORB::create(nFeatures,1,nlevels,edgeThreshold,0,2,ORB::HARRIS_SCORE,_descriptorSize,10);

    if(!m_detector)
        assert("Can not create Detector of ORB!");
    m_matcher = DescriptorMatcher::create("BruteForce-Hamming");

    if(!m_matcher)
        assert("Can not create Matcher of BruteForce-Hamming!");

    return ;
}

OrbPatch::OrbPatch(Point2f _origin,const Mat _queryImage,const Mat _trainImage )
{
    origin = _origin;
    qImage = _queryImage;
    tImage = _trainImage;
}

void OrbPatch::setPatch(const Point2f _origin,const Mat _queryImage,const Mat _trainImage)
{
    if(!m_detector)
        assert("Please Create Patch frist!");
    origin = _origin;
    qImage = _queryImage;
    tImage = _trainImage;
    return;
}

static bool com(KeyPoint pfirst, KeyPoint psecond)
{
    if(pfirst.pt.x == psecond.pt.x){
        if(pfirst.pt.y < psecond.pt.y)
            return true;
        else
            return false;
    }
    else if(pfirst.pt.x < psecond.pt.x)
        return true;
    else
        return false;
}

int OrbPatch::detectKeypoints(void)
{
    int ret = 0;
    assert(qImage.channels() == 1);
    assert(tImage.channels() == 1);
    qKeypoints.clear();
    tKeypoints.clear();
//        m_detector->clear();
    m_detector->detect(qImage, qKeypoints);
    m_detector->detect(tImage, tKeypoints);
    int qNums = qKeypoints.size();
    int tNums = tKeypoints.size();
    ret = std::min(qNums,tNums);
    LOGE("The numbers of detect KeyPoints = %d", ret);

    return ret;
}

void OrbPatch::extractDescriptors(float _iso)
{
    if(qKeypoints.empty() ||tKeypoints.empty())
        return;
    //    m_detector->clear();
    m_detector->compute(qImage, qKeypoints,qDescriptor,mISO);
    m_detector->compute(tImage, tKeypoints, tDescriptor,mISO);

    return ;
}

bool OrbPatch::detectAndCompute(void)
{
    bool ret = false;
    int n = detectKeypoints();
    if(n > 0) {
        extractDescriptors();
        return true;
    }
    return ret;
}

static bool copare(DMatch pfirst, DMatch psecond)
{
    return pfirst.distance < psecond.distance;
}

void OrbPatch::sortMatcher(vector<DMatch>& matcher)
{
    std::sort(matcher.begin(),matcher.end(),copare);
    return;
}

void OrbPatch::checkOffset(vector<DMatch>& matcher)
{
    vector<DMatch> stable_DMatch;
    stable_DMatch.clear();
    int nMatchers = matcher.size();
    if(nMatchers < 1)
        return;
    int xOffset = 0,yOffset = 0;
    for (int matchIndex = 0; matchIndex < nMatchers; matchIndex++) {
        const DMatch& match = matcher[matchIndex];
        double dErrorY = qKeypoints[match.queryIdx].pt.y
                         - tKeypoints[match.trainIdx].pt.y;
        double dErrorX = qKeypoints[match.queryIdx].pt.x
                         - tKeypoints[match.trainIdx].pt.x;
        xOffset += dErrorX;
        yOffset += dErrorY;
    }
    yOffset = yOffset / nMatchers;
    xOffset = xOffset / nMatchers;
    for (int matchIndex = 0; matchIndex < nMatchers; matchIndex++)
    {
        const DMatch& match = matcher[matchIndex];
        double errorY = qKeypoints[match.queryIdx].pt.y
                         - tKeypoints[match.trainIdx].pt.y;
        double errorX = qKeypoints[match.queryIdx].pt.x
                         - tKeypoints[match.trainIdx].pt.x;

        float qAngle = qKeypoints[match.queryIdx].angle;
        float tAngle = tKeypoints[match.trainIdx].angle;
        float errorAngle = qAngle - tAngle;

        double symbolX = errorX * xOffset;
        double symbolY = errorY * yOffset;
        double offsetX = errorX - xOffset;
        double offsetY = errorY - yOffset;
        if(symbolX < 0 || symbolY < 0 || fabs(errorAngle) > 5) {
            continue;
        }
        if(xOffset > 0) {
            if(offsetX > 5)
            continue;
        } else {
            if(offsetX < -5)
            continue;
        }
        if(yOffset > 0) {
            if(offsetY > 5)
            continue;
        } else {
            if(offsetY < -5)
            continue;
        }
        stable_DMatch.push_back(match);
    }
    sortMatcher(stable_DMatch);
    matcher.swap(stable_DMatch);
    return;
}

static bool comPoint(Point2f pfirst, Point2f psecond)
{
    return pfirst.x < psecond.x;
}


void OrbPatch::knnMatch(void)
{
    if(qDescriptor.empty() ||tDescriptor.empty())
        return ;
    const int k = 2;
    vector< vector<DMatch> > knnMatches;
    if(!m_matcher)
        assert("Please Create Patch frist!");
    m_matcher->clear();
    m_matcher->add(vector<Mat>(1, tDescriptor));
    m_matcher->train();
    m_matcher->knnMatch(qDescriptor, knnMatches, k);
    // m_matcher->radiusMatch(qDescriptor, knnMatches, 20.0);

    stable_DMatch.clear();
    for (int matchIndex = 0; matchIndex < knnMatches.size(); matchIndex++)
    {
        const DMatch& bestMatch = knnMatches[matchIndex][0];
        const DMatch& betterMatch = knnMatches[matchIndex][1];
        float rate = betterMatch.distance / bestMatch.distance;

        if(rate > 1.5 && bestMatch.distance < 150)
        {
            double dErrorY = qKeypoints[bestMatch.queryIdx].pt.y
                             - tKeypoints[bestMatch.trainIdx].pt.y;
            double dErrorX = qKeypoints[bestMatch.queryIdx].pt.x
                             - tKeypoints[bestMatch.trainIdx].pt.x;
            if(fabs(dErrorY) < 30 && fabs(dErrorX) < 30 )
                stable_DMatch.push_back(bestMatch);
        }
    }
    Point2f qPoints;
    Point2f tPoints;
    vector<Point2f> qPVec;
    vector<Point2f> tPVec;
    tPVec.clear();
    qPVec.clear();

    int nDMatches = stable_DMatch.size();

    for(int index = 0;index < nDMatches;index++) {
        qPoints.x = qKeypoints[stable_DMatch[index].queryIdx].pt.x;
        qPoints.y = qKeypoints[stable_DMatch[index].queryIdx].pt.y;
        tPoints.x = tKeypoints[stable_DMatch[index].trainIdx].pt.x;
        tPoints.y = tKeypoints[stable_DMatch[index].trainIdx].pt.y;
        qPVec.push_back(qPoints);
        tPVec.push_back(tPoints);
    }
    if(nDMatches > 4) {
        vector<uchar> inliersMask(tPVec.size());
        findHomography(qPVec, tPVec, RANSAC, 3, inliersMask, 2000, 0.995);
        // rk::MyFindHomography(qPVec, tPVec, RANSAC, 3, inliersMask, 2000, 0.995);
        // findFundamentalMat(qPVec, tPVec, RANSAC, 3, 0.99, inliersMask);
        vector<DMatch> inliers;
        for (int i = 0; i < inliersMask.size(); i++){
            if (inliersMask[i])
                inliers.push_back(stable_DMatch[i]);
        }
        stable_DMatch.swap(inliers);
    }
    checkOffset(stable_DMatch);

// #if 0
//     static int time = 1;
//     Mat dst;
//     char path[255];
//     memset(path, 0, 255 * sizeof(char));
//     sprintf(path, "/data/local/matchers/%d.jpg", time);
//
//     if(stable_DMatch.size() > 0)
//     {
//         time++;
//         //cout<<"pointsMatches = "<<n<<endl;
//         saveMatches(qImage, qKeypoints, tImage, tKeypoints, stable_DMatch, dst);
//         imwrite(path, dst);
//     }
// #endif
    Point2f queryPoints;
    Point2f trainPoints;
    qPointsVec.clear();
    tPointsVec.clear();
//    vector<Point2f>(qPointsVec).swap(qPointsVec);
//    vector<Point2f>(tPointsVec).swap(tPointsVec);

    int iDMatches = stable_DMatch.size();
    for(int index = 0;index < iDMatches;index++)
    {
        //because of pyrDown,we need to mutilply both x and y with 4
        queryPoints.x = qKeypoints[stable_DMatch[index].queryIdx].pt.x*2.0f + origin.x;
        queryPoints.y = qKeypoints[stable_DMatch[index].queryIdx].pt.y*2.0f + origin.y;

        trainPoints.x = tKeypoints[stable_DMatch[index].trainIdx].pt.x*2.0f + origin.x;
        trainPoints.y = tKeypoints[stable_DMatch[index].trainIdx].pt.y*2.0f + origin.y;

        qPointsVec.push_back(queryPoints);
        tPointsVec.push_back(trainPoints);
    }
    return ;
}

int OrbPatch::getPointsMatchers(vector <Point2f> & _queryPoints,vector <Point2f> & _trainPoints,float _iso)
{
    mISO = _iso;
    if(!qImage.empty() ||!tImage.empty())
        assert("Please set Patch frist!");

    _queryPoints.clear();
    _trainPoints.clear();

    bool n = detectAndCompute();
    if(n == false){
        return 0;
    }
    knnMatch();
    int ret = qPointsVec.size();
    _queryPoints = qPointsVec;
    _trainPoints = tPointsVec;

    return ret;
}

//保存特征点，在图上体现出来
void OrbPatch::saveKeypoints(const Mat& image, const vector<KeyPoint>& keypoints, Mat& outImage)
{
    assert(!keypoints.empty());

    cv::drawKeypoints(image, keypoints, outImage, /*Scalar(255,255,0),*/Scalar::all(-1),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

}

//保存匹配的特征点，在图上体现出来
void OrbPatch::saveMatches(const Mat& queryImage,
                        const vector<KeyPoint>& queryKeypoints,
                        const Mat& trainImage,
                        const vector<KeyPoint>& trainKeypoints,
                        const vector<DMatch>& matches,
                        Mat& outImage)
{
    assert(!queryKeypoints.empty());
    assert(!trainKeypoints.empty());

    cv::drawMatches(queryImage, queryKeypoints, trainImage, trainKeypoints, matches, outImage,
                    Scalar::all(-1), Scalar::all(-1),vector<char>(),  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

}
//
