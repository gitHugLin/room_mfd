//
// Created by linqi on 16-4-5.
//

#ifndef MY_JNI_MUTGETHOMOGRAPHY_H
#define MY_JNI_MUTGETHOMOGRAPHY_H


#include "../include/GetHomography.h"
#include "../include/MyThread.h"
#include <semaphore.h>
#include "MyStruct.h"
#include "MySemophore.h"


using namespace mystruct;


class MutGetHomography: public MyThread
{

public:
    MutGetHomography(const vector <Mat> & images);
    ~MutGetHomography();
public:
    void process(vector <HomIntMat> & HomVec,float _iso = 2);
    bool setMode(int homoMethod = RANSAC);
public:
    void workBegin();
    void workEnd();
public:
    MyThread* thFirst;
    MyThread* thSecond;
    MyThread* thThird;
    MyThread* thFourth;

    void MutilgetHom();
    void run();
private:
    int HomoMethod;
    int Width;
    int Height;
    Mat gray2;
    int64 work_begin;
    int64 work_end;
    vector <HomIntMat> m_homographys;
    vector <Mat> m_images;
private:
    float mISO;
    pthread_mutex_t g_mutex;
    // MySemophore mSemophore;
    sem_t sem_id;

};



#endif //MY_JNI_MUTGETHOMOGRAPHY_H
