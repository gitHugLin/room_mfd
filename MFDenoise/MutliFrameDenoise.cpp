#include "MutliFrameDenoise.h"
#include "log.h"
#include "include/PerspectiveAdd.h"
#include <android/log.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES2/gl2platform.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>

using namespace std;
using namespace android;

static double work_begin = 0;
static double work_end = 0;
static double gTime = 0;

static void workBegin()
{
    work_begin = getTickCount();
}

static void workEnd()
{
    work_end = getTickCount() - work_begin;
    gTime = work_end /((double)getTickFrequency() )* 1000.0;
    LOGE("TIME = %lf ms \n",gTime);
}

MutliFrameDenoise::MutliFrameDenoise():initialized(false)
{
}

MutliFrameDenoise::~MutliFrameDenoise()
{
	LOGD("----------free MutliFrameDenoise");
	//initialized = false;
}


void  MutliFrameDenoise::initOpenGLES(alloc_device_t *m_alloc_dev, int width, int height)
{
    //getImageUnderDir("/data/isptune","pgm");
    g_APUnit = new PerspectiveAdd();
    g_APUnit->initOpenGLES(m_alloc_dev, width, height);

	initialized = true;
}

void  MutliFrameDenoise::updateImageData(struct cv_fimc_buffer *m_buffers_capture) {
	g_APUnit->updateImageData(m_buffers_capture);
}


long  MutliFrameDenoise::processing(int* targetAddr, float _iso,int mode)
{
	LOGD("MutliFrameDenoise::processing... mISO = %f",_iso);
	if (!initialized) {
		LOGE("processing failed - check init first.");
		return NULL;
	}
    mISO = _iso;
	workBegin();
    Mat outMat;
    int HomMethod = RANSAC; //RHO RANSAC LMEDS

    g_APUnit->setMode(HomMethod);
    g_APUnit->Progress(outMat, targetAddr,mISO, mode);

    workEnd();
    //return (long)imgData;
    return 0;

}

int MutliFrameDenoise::getResult(int targetAddr) {
	g_APUnit->getResult(targetAddr);
	return 0;
}

void MutliFrameDenoise::destroy() {
	delete g_APUnit;
	initialized = false;
}
