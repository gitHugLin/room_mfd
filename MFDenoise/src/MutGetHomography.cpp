//
// Created by linqi on 16-4-5.
//

#include "../include/MutGetHomography.h"
#include "log.h"

// #define LOGE printf


MutGetHomography::MutGetHomography(const vector<Mat> &images) {
  if (images.size() != 6)
    assert("Error in MutilThread: please input six images!");
  m_images.clear();
  m_homographys.clear();

  sem_init(&sem_id, 0, 0);
  pthread_mutex_init(&g_mutex, NULL);

  for (size_t i = 0; i < images.size(); i++) {
    m_images.push_back(images[i]);
  }

  Width = m_images[0].size().width;
  Height = m_images[0].size().height;
  HomoMethod = RANSAC;
  work_begin = 0;
  work_end = 0;
  mISO = 2;
}

//开始计时
void MutGetHomography::workBegin() { work_begin = getTickCount(); }

//结束计时
void MutGetHomography::workEnd() {
  double gTime = 0.0;
  work_end = getTickCount() - work_begin;
  gTime = work_end / ((double)getTickFrequency()) * 1000.0;
  LOGE("TIME = %lf ms \n", gTime);
}

MutGetHomography::~MutGetHomography() {
  pthread_mutex_destroy(&g_mutex);
  sem_destroy(&sem_id);
}

bool MutGetHomography::setMode(int homoMethod) {
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

void MutGetHomography::MutilgetHom() {
  thFirst = new MyThread(this);
  thSecond = new MyThread(this);
  thThird = new MyThread(this);
  thFourth = new MyThread(this);
  /*设置线程优先级为最高级*/
  this->set_thread_priority(90);
  thFirst->set_thread_priority(80);
  thSecond->set_thread_priority(80);
  thThird->set_thread_priority(80);
  thFourth->set_thread_priority(80);
  thSecond->start();
  thFirst->start();
  thThird->start();
  thFourth->start();
  start();

  thFirst->join();
  thSecond->join();
  thThird->join();
  thFourth->join();
}

void MutGetHomography::run() {
  if (is_equals(this)) {
    pthread_mutex_lock(&g_mutex);
    // Mat standard = m_images[2].clone();
    GetHomography homography(m_images[2], m_images[0]);
    pthread_mutex_unlock(&g_mutex);

    HomIntMat myIntMat;
    homography.setMode(HomoMethod);
    myIntMat.Homography = homography.getHomography(mISO);
    if (myIntMat.Homography.empty())
      myIntMat.index = -1;
    else
      myIntMat.index = 0;

    pthread_mutex_lock(&g_mutex);
    m_homographys.push_back(myIntMat);
    if (m_homographys.size() == 5)
        // mSemophore.SignalSemaphore();
      sem_post(&sem_id);
    pthread_mutex_unlock(&g_mutex);
    return;
  } else if (is_equals(thSecond)) {
    pthread_mutex_lock(&g_mutex);
    GetHomography homography(m_images[2], m_images[1]);
    // Mat standard = m_images[2].clone();
    // GetHomography homography(standard, m_images[1]);
    pthread_mutex_unlock(&g_mutex);

    HomIntMat myIntMat;
    homography.setMode(HomoMethod);
    myIntMat.Homography = homography.getHomography(mISO);
    if (myIntMat.Homography.empty())
      myIntMat.index = -1;
    else
      myIntMat.index = 1;

    pthread_mutex_lock(&g_mutex);
    m_homographys.push_back(myIntMat);
    if (m_homographys.size() == 5)
        // mSemophore.SignalSemaphore();
      sem_post(&sem_id);
    pthread_mutex_unlock(&g_mutex);
    return;
  } else if (is_equals(thFirst)) {
    pthread_mutex_lock(&g_mutex);
    GetHomography homography(m_images[2], m_images[3]);
    // Mat standard = m_images[2].clone();
    // GetHomography homography(standard, m_images[3]);
    pthread_mutex_unlock(&g_mutex);

    HomIntMat myIntMat;
    homography.setMode(HomoMethod);
    myIntMat.Homography = homography.getHomography(mISO);
    if (myIntMat.Homography.empty())
      myIntMat.index = -1;
    else
      myIntMat.index = 3;

    pthread_mutex_lock(&g_mutex);
    m_homographys.push_back(myIntMat);
    if (m_homographys.size() == 5)
        // mSemophore.SignalSemaphore();
      sem_post(&sem_id);
    pthread_mutex_unlock(&g_mutex);
    return;
  } else if (is_equals(thThird)) {
    pthread_mutex_lock(&g_mutex);
    GetHomography homography(m_images[2], m_images[4]);
    // Mat standard = m_images[2].clone();
    // GetHomography homography(standard, m_images[4]);
    pthread_mutex_unlock(&g_mutex);

    HomIntMat myIntMat;
    homography.setMode(HomoMethod);
    myIntMat.Homography = homography.getHomography(mISO);
    if (myIntMat.Homography.empty())
      myIntMat.index = -1;
    else
      myIntMat.index = 4;

    pthread_mutex_lock(&g_mutex);
    m_homographys.push_back(myIntMat);
    if (m_homographys.size() == 5)
        // mSemophore.SignalSemaphore();
     sem_post(&sem_id);
    pthread_mutex_unlock(&g_mutex);
    return;
  } else if (is_equals(thFourth)) {
    pthread_mutex_lock(&g_mutex);
    GetHomography homography(m_images[2], m_images[5]);
    // Mat standard = m_images[2].clone();
    // GetHomography homography(standard, m_images[5]);
    pthread_mutex_unlock(&g_mutex);

    HomIntMat myIntMat;
    homography.setMode(HomoMethod);
    myIntMat.Homography = homography.getHomography(mISO);
    if (myIntMat.Homography.empty())
      myIntMat.index = -1;
    else
      myIntMat.index = 5;

    pthread_mutex_lock(&g_mutex);
    m_homographys.push_back(myIntMat);
    if (m_homographys.size() == 5)
        // mSemophore.SignalSemaphore();
     sem_post(&sem_id);
    pthread_mutex_unlock(&g_mutex);
    return;
  }
}

void MutGetHomography::process(vector<HomIntMat> &HomVec,float _iso) {
  mISO = _iso;
  LOGD("MutGetHomography mISO = %f",mISO);
  MutilgetHom();
  // LOGE("sem_wait !\n");
  HomVec.clear();
  sem_wait(&sem_id);
  // mSemophore.TryWaitSemaphore();
  // six frames need five Homography matrix
  for (int i = 0; i < m_homographys.size(); i++) {
    HomVec.push_back(m_homographys[i]);
  }
  if (NULL != thFirst) {
    delete thFirst;
    thFirst = NULL;
  }
  if (NULL != thSecond) {
    delete thSecond;
    thSecond = NULL;
  }
  if (NULL != thThird) {
    delete thThird;
    thThird = NULL;
  }
  if (NULL != thFourth) {
    delete thFourth;
    thFourth = NULL;
  }
}
