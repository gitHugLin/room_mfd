//
// Created by 林奇 on 16/7/29.
//

#include "../include/MySemophore.h"

MySemophore::MySemophore(){
    memset(mName,0, 256*sizeof(char));
    char defaultName[] = "MySemophore";
    memcpy(mName,defaultName, sizeof(defaultName));
    mSemaphore = NULL;
    mStartingCount = 1;
    CreateSemaphore();
}

MySemophore::MySemophore(char inName[], int inStartingCount){
    memset(mName,0, 256*sizeof(char));
    int length = strlen(inName);
    for(size_t i = 0;i < length; i++) {
        mName[i] = inName[i];
    }
//    memcpy(mName,inName, sizeof(inName));
    mStartingCount = inStartingCount;
    CreateSemaphore();
}

MySemophore::~MySemophore(){
    DestroySemaphore();
    ClearSemaphore();
}

bool MySemophore::CreateSemaphore()
{
    mSemaphore = sem_open(mName, O_CREAT, 0644,mStartingCount);

    if( mSemaphore == SEM_FAILED )
    {
        switch( errno )
        {
            case EEXIST:
                printf( "Semaphore with name '%s' already exists.\n", mName);
                break;

            default:
                printf( "Unhandled error: %d.\n", errno );
                break;
        }

        assert(false);
        return SEM_FAILED;
    }

    return true;
}


bool MySemophore::DestroySemaphore()
{
    int retErr = sem_close(mSemaphore);

    if( retErr == -1 )
    {
        switch( errno )
        {
            case EINVAL:
                printf( "inSemaphore is not a valid sem_t object." );
                break;

            default:
                printf( "Unhandled error: %d.\n", errno );
                break;
        }

        assert(false);
        return false;
    }

    return true;
}

void MySemophore::SignalSemaphore()
{
    sem_post( mSemaphore );
}

void MySemophore::WaitSemaphore()
{
    sem_wait( mSemaphore );
}

bool MySemophore::TryWaitSemaphore()
{
    int retErr = sem_trywait( mSemaphore );

    if( retErr == -1 )
    {
        if( errno != EAGAIN )
        {
            printf( "Unhandled error: %d\n", errno );
            assert( false );
        }

        return false;
    }

    return true;
}

bool MySemophore::ClearSemaphore()
{
    int retErr = sem_unlink(mName);

    if (retErr == -1) {

        return false;

    }

    return true;
}
