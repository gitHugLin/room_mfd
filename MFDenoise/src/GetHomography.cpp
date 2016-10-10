//
// Created by linqi on 16-4-5.
//

#include "../include/GetHomography.h"
#include "../include/MyPointSetRegistrator.h"
#include "log.h"
#include<algorithm>


GetHomography::~GetHomography()
{
  pthread_mutex_destroy(&g_mutex);

}

GetHomography::GetHomography(Mat &srcImage, Mat &dstImage) {

  pthread_mutex_init(&g_mutex, NULL);
  if (!srcImage.empty() || !dstImage.empty())
    assert("Error in Homography: input image can not be empty!");
  assert(srcImage.channels() == 1);
  assert(dstImage.channels() == 1);

  m_patch.createPatch(10, 1, 16, 31);
  // m_patch.createPatch(200,3,13,25);

  m_queryPoints.clear();
  m_trainPoints.clear();

  trainPattern = srcImage;
  queryPattern = dstImage;
  mWidth = srcImage.cols;
  mHeight = srcImage.rows;

  // scaleSize 是在pyrDown里缩放的大小，这个不能随便改动
  HomoMethod = RANSAC;
  scaleSize = 1;
  work_begin = 0;
  work_end = 0;
  mISO = 2;
}

//开始计时
void GetHomography::workBegin() { work_begin = getTickCount(); }

//结束计时
void GetHomography::workEnd() {
  work_end = getTickCount() - work_begin;
  double gTime = work_end / ((double)getTickFrequency()) * 1000.0;
  LOGE("TIMES = %lf",gTime);
}

void GetHomography::savePoints(vector<Point2f> &_queryPoints,
                               vector<Point2f> &_trainPoints,int sub) {
  pthread_mutex_lock(&g_mutex);
  if(sub < 0){
      int npPoint = _queryPoints.size();
      int npPoints = std::min(10,npPoint);
      for(int i = 0; i < npPoints; i++) {
          m_queryPoints.push_back(_queryPoints[i]);
          m_trainPoints.push_back(_trainPoints[i]);
      }
      pthread_mutex_unlock(&g_mutex);
      return;
  }

  int npPoints = _queryPoints.size();
  for(int i = 0; i < npPoints - sub; i++) {
      m_queryPoints.push_back(_queryPoints[i]);
      m_trainPoints.push_back(_trainPoints[i]);
  }

  pthread_mutex_unlock(&g_mutex);
  return;
}

bool GetHomography::detectPoints(const int _patchSize, const int keyPointThread,
                                 const int robEdgeThreshold) {
  LOGE("detectPoints;");
  bool ret = true;
  int patchSzie = _patchSize;
  // robEdgeThread mutiply 2 side of each patch and the scale size of pyramid
  int roiPWidth = patchSzie - robEdgeThreshold * 2 * 2;
  //central area of picture
  // double centerX = (mWidth-patchSzie)/2.0f,centerY = (mHeight-patchSzie)/2.0f;
  // Point2f cOrigin(centerX, centerY);
  // Rect cROI(centerX, centerY, patchSzie,patchSzie);
  // Mat cRoiQueryArea(queryPattern, cROI);
  // Mat cRoiTrainArea(trainPattern, cROI);
  // pyrDown(cRoiQueryArea, cRoiQueryArea);
  // pyrDown(cRoiTrainArea, cRoiTrainArea);
  // m_patch.setPatch(cOrigin, cRoiQueryArea, cRoiTrainArea);
  // vector<Point2f> cQueryPoints,cTrainPoints;
  // cQueryPoints.clear();
  // cTrainPoints.clear();
  // m_patch.getPointsMatchers(cQueryPoints, cTrainPoints,mISO);
  // int nMatchers = cQueryPoints.size();
  // if(nMatchers > 0){
  //     savePoints(cQueryPoints, cTrainPoints,-1);
  // }
  // LOGE("nMatchers = %d",nMatchers);

  for (int patch_index = 0; patch_index < 4; patch_index++) {
    int iKeyPoints = 0; // the sum of min keyPoints between query and train
    int cols_index = 0; // cols index
    int rows_index = 0; // rows index

    switch (patch_index) {
    case 0: // left top corner
    {
      int timeCount = 0;
      // LOGE("CORNER 0");
      while (iKeyPoints <= keyPointThread) {
        // LOGE("right top corner!");
        timeCount++;
        // int moveRange = cols_index*patchSzie;
        int moveRange = cols_index * roiPWidth;
        if (moveRange > queryPattern.cols / 2 /* - patchSzie */) // row changed
        {
          rows_index++;
          cols_index = 0;
          moveRange = 0;
          if (roiPWidth * rows_index > queryPattern.rows - patchSzie ||
              roiPWidth * rows_index > trainPattern.rows - patchSzie) {
            LOGE("points in left top corner are too little!\n");
            LOGE("left top corner KeyPoints = %d", iKeyPoints);
            ret = false;
            return ret;
            break;
          }

        }
        Point2f Origin(roiPWidth * cols_index, roiPWidth * rows_index);
        Rect ROI(roiPWidth * cols_index, roiPWidth * rows_index, patchSzie,
                 patchSzie);
        Mat roiQueryArea(queryPattern, ROI);
        Mat roiTrainArea(trainPattern, ROI);

        pyrDown(roiQueryArea, roiQueryArea);
        pyrDown(roiTrainArea, roiTrainArea);
        // pyrDown(roiQueryArea, roiQueryArea);
        // pyrDown(roiTrainArea, roiTrainArea);

        m_patch.setPatch(Origin, roiQueryArea, roiTrainArea);

        vector<Point2f> queryPoints, trainPoints;
        m_patch.getPointsMatchers(queryPoints, trainPoints,mISO);

        iKeyPoints += queryPoints.size();
        int sub = iKeyPoints - keyPointThread;
        if(sub > 0) {
            savePoints(queryPoints, trainPoints,sub);
        } else {
            savePoints(queryPoints, trainPoints,0);
        }

        cols_index++;
      }
      LOGE("left top corner timeCount = %d", timeCount);
    } break;
    case 1: // right top corner
    {
      // LOGE("CORNER 1");
      int timeCount = 0;
      while (iKeyPoints <= keyPointThread) {
        // LOGE("right top corner!");
        timeCount++;
        int moveRange = rows_index * roiPWidth;
        if (moveRange > queryPattern.rows / 2 /* - patchSzie */) // cols changed
        {
          cols_index++;
          rows_index = 0;
          moveRange = 0;
          if (roiPWidth * cols_index > queryPattern.cols - patchSzie ||
              roiPWidth * cols_index > trainPattern.cols - patchSzie) {
            LOGE("points in right top corner are too little!\n");
            LOGE("right top corner KeyPoints = %d", iKeyPoints);
            ret = false;
            return ret;
            break;
          }

        }

        Point2f Origin(trainPattern.cols - patchSzie - cols_index * roiPWidth,
                       roiPWidth * rows_index);

        Rect ROI(trainPattern.cols - patchSzie - cols_index * roiPWidth,
                 roiPWidth * rows_index, patchSzie, patchSzie);
        Mat roiQueryArea(queryPattern, ROI);
        Mat roiTrainArea(trainPattern, ROI);

        pyrDown(roiQueryArea, roiQueryArea);
        pyrDown(roiTrainArea, roiTrainArea);
        // pyrDown(roiQueryArea, roiQueryArea);
        // pyrDown(roiTrainArea, roiTrainArea);

        m_patch.setPatch(Origin, roiQueryArea, roiTrainArea);

        vector<Point2f> queryPoints, trainPoints;
        m_patch.getPointsMatchers(queryPoints, trainPoints,mISO);

        iKeyPoints += queryPoints.size();
        int sub = iKeyPoints - keyPointThread;
        if(sub > 0) {
            savePoints(queryPoints, trainPoints,sub);
        } else {
            savePoints(queryPoints, trainPoints,0);
        }
        rows_index++;
      }
      LOGE("right top corner timeCount = %d", timeCount);
    } break;
    case 2: // right bottom corner
    {
      // LOGE("CORNER 2");
      int timeCount = 0;
      while (iKeyPoints <= keyPointThread) {
        // LOGE("left bottom corner!");
        timeCount++;
        int moveRange = cols_index * roiPWidth;
        if (moveRange > queryPattern.cols / 2 /* - patchSzie */) // row changed
        {
          rows_index++;
          cols_index = 0;
          moveRange = 0;
          if (roiPWidth * rows_index > queryPattern.rows - patchSzie ||
              roiPWidth * rows_index > trainPattern.rows - patchSzie) {
            LOGE("points in right bottom corner are too little!\n");
            LOGE("right bottom corner KeyPoints = %d", iKeyPoints);
            ret = false;
            return ret;
            break;
          }

        }
        Point2f Origin(queryPattern.cols - patchSzie - roiPWidth * cols_index,
                       queryPattern.rows - patchSzie - roiPWidth * rows_index);

        Rect ROI(queryPattern.cols - patchSzie - roiPWidth * cols_index,
                 queryPattern.rows - patchSzie - roiPWidth * rows_index,
                 patchSzie, patchSzie);
        Mat roiQueryArea(queryPattern, ROI);
        Mat roiTrainArea(trainPattern, ROI);

        pyrDown(roiQueryArea, roiQueryArea);
        pyrDown(roiTrainArea, roiTrainArea);
        // pyrDown(roiQueryArea, roiQueryArea);
        // pyrDown(roiTrainArea, roiTrainArea);

        m_patch.setPatch(Origin, roiQueryArea, roiTrainArea);

        vector<Point2f> queryPoints, trainPoints;
        m_patch.getPointsMatchers(queryPoints, trainPoints,mISO);
        iKeyPoints += queryPoints.size();
        int sub = iKeyPoints - keyPointThread;
        if(sub > 0) {
            savePoints(queryPoints, trainPoints,sub);
        } else {
            savePoints(queryPoints, trainPoints,0);
        }
        cols_index++;
      }
      LOGE("right bottom corner timeCount = %d", timeCount);
    } break;
    case 3: // left bottom corner
    {
      // LOGE("CORNER 3");
      int timeCount = 0;
      while (iKeyPoints <= keyPointThread) {
        timeCount++;
        // LOGE("right bottom corner!");
        int moveRange = rows_index * roiPWidth;
        if (moveRange > queryPattern.rows / 2 /* - patchSzie */) // col changed
        {
          cols_index++;
          rows_index = 0;
          moveRange = 0;

          if (roiPWidth * cols_index > queryPattern.cols - patchSzie ||
              roiPWidth * cols_index > trainPattern.cols - patchSzie) {
            LOGE("points in left bottom corner are too little!\n");
            LOGE("left bottom corner KeyPoints = %d", iKeyPoints);
            ret = false;
            return ret;
            break;
          }
        }

        Point2f Origin(roiPWidth * cols_index,
                       queryPattern.rows - patchSzie - roiPWidth * rows_index);

        Rect ROI(roiPWidth * cols_index,
                 queryPattern.rows - patchSzie - roiPWidth * rows_index,
                 patchSzie, patchSzie);
        Mat roiQueryArea(queryPattern, ROI);
        Mat roiTrainArea(trainPattern, ROI);

        pyrDown(roiQueryArea, roiQueryArea);
        pyrDown(roiTrainArea, roiTrainArea);
        // pyrDown(roiQueryArea, roiQueryArea);
        // pyrDown(roiTrainArea, roiTrainArea);

        m_patch.setPatch(Origin, roiQueryArea, roiTrainArea);

        vector<Point2f> queryPoints, trainPoints;
        m_patch.getPointsMatchers(queryPoints, trainPoints,mISO);
        iKeyPoints += queryPoints.size();
        int sub = iKeyPoints - keyPointThread;
        if(sub > 0) {
            savePoints(queryPoints, trainPoints,sub);
        } else {
            savePoints(queryPoints, trainPoints,0);
        }
        rows_index++;
      }
      LOGE("left bottom corner timeCount = %d", timeCount);
    } break;
    default:
      break;
    }
  }

  return ret;
}

bool GetHomography::setMode(int homoMethod) {
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

static bool copare(Point2f pfirst, Point2f psecond)
{
    if(pfirst.x == psecond.x){
        if(pfirst.y < psecond.y)
            return true;
        else
            return false;
    }
    else if(pfirst.x < psecond.x)
        return true;
    else
        return false;
}

Mat GetHomography::getHomography(float _iso,double reprojectionThreshold) {
  mISO = _iso;
  LOGD("GetHomography mISO = %f",mISO);
  workBegin();
  bool ret = detectPoints();
  LOGD("DETECT POINTS");
  workEnd();
  if (ret) {
    // Mat homography =
    //     rk::MyFindHomography(m_queryPoints, m_trainPoints, RANSAC,
    //                    reprojectionThreshold, noArray(), 2000, 0.995);
    Mat homography =
        findHomography(m_queryPoints, m_trainPoints, RANSAC,
                       reprojectionThreshold, noArray(), 2000, 0.995);
    m_Homography = homography.inv();
  }
  return m_Homography;
}
