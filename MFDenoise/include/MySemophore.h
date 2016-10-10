//
// Created by 林奇 on 16/7/29.
//

#ifndef MFD_MYSEMOPHORE_H
#define MFD_MYSEMOPHORE_H

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <assert.h>
#include <semaphore.h>
#include "string.h"


class MySemophore {
public:
    MySemophore();
    MySemophore(char inName[], int inStartingCount);
    ~MySemophore();

private:
    char mName[256];
    int mStartingCount;
    sem_t * mSemaphore;
public:
    void SignalSemaphore();
    void WaitSemaphore();
    bool TryWaitSemaphore();

private:
    bool CreateSemaphore();
    bool DestroySemaphore();
    bool ClearSemaphore();
};


#endif //MFD_MYSEMOPHORE_H
