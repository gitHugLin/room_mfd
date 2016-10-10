//
// Created by linqi on 16-4-5.
//
#include "include/PerspectiveAdd.h"

#include <gui/GLConsumer.h>
#include <gui/Surface.h>
#ifdef ANDROID5X
#include <gui/IGraphicBufferConsumer.h>
#include <gui/IGraphicBufferProducer.h>
#endif
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferAllocator.h>
#include <gralloc_priv.h>
#include "include/utils.h"
#include "stdio.h"

using namespace android;

GLint TEXTURE_GROUP[] = {
	GL_TEXTURE0,
	GL_TEXTURE1,
	GL_TEXTURE2,
	GL_TEXTURE3,
	GL_TEXTURE4,
	GL_TEXTURE5,
	GL_TEXTURE6
};

int checkEglError(const char* op) {
    for (EGLint error = eglGetError(); error != EGL_SUCCESS; error
            = eglGetError()) {
        ALOGD("after %s() eglError %s (0x%x)\n", op, error,
                error);
        return -1;
    }

    return 0;
}

void checkGlError(const char* op) {
    for (GLint error = glGetError(); error; error
            = glGetError()) {
        LOGD("after %s() glError (0x%x)\n", op, error);
    }
}

buffer_handle_t bufferHandleAlloc(uint32_t w, uint32_t h, PixelFormat format, uint32_t usage) {

    buffer_handle_t handle;             
    int stride;
    GraphicBufferAllocator& allocator = GraphicBufferAllocator::get();
    status_t err = allocator.alloc(w, h, format, usage, &handle, &stride);
    fprintf(stderr, "bufferHandleAlloc status: %d stride = %d, handle = %p\n", err, stride, handle);
    if (err == NO_ERROR) {
        return handle;
    }
    return NULL;
}   

void bufferHandleFree(buffer_handle_t handle) {

    GraphicBufferAllocator& allocator = GraphicBufferAllocator::get();
    status_t err = allocator.free(handle);
}

PerspectiveAdd::PerspectiveAdd()
{
    work_begin = 0;
    work_end = 0;
    checkInitOpenGLES = false;
	mFoundHomography = false;
	mCurrentId = 0;
    mTargetGraphicBuffer=NULL;
    mTargetEGLImage = EGL_NO_IMAGE_KHR;
	for (int i=0; i<6; i++) {
		mCameraGLTexImage[i] = NULL;
		m_grays.push_back(Mat());
	}
	mISO = 2;
    //initOpenGLES();
}

PerspectiveAdd::~PerspectiveAdd()
{
	LOGD("~PerspectiveAdd is being excute");
	m_grays.clear();

    glUseProgram(0);
    glDeleteProgram(programObject);
    glDeleteTextures(1,&textureID1);
    glDeleteTextures(1,&textureID2);
    glDeleteTextures(1,&textureID3);
    glDeleteTextures(1,&textureID4);
    glDeleteTextures(1,&textureID5);
    glDeleteTextures(1,&textureID6);
    glDeleteTextures(1,&targetTexId);
	glDeleteFramebuffers(1, &fboTargetHandle);

	//add if by linqi in 2016-8-22
	if(display != NULL)
    	eglDestroyImageKHR(display, mTargetEGLImage);
    mTargetEGLImage = EGL_NO_IMAGE_KHR;

	if(mTargetBufferHandle != NULL) {
		ALOGD("########destroy main frame buffer.");
        bufferHandleFree(mTargetBufferHandle);
        mTargetBufferHandle = NULL;
    }

	//if (mTargetGraphicBuffer != NULL) {
		ALOGD(" |---destroy main graphic buffer.");
    	mTargetGraphicBuffer = NULL;
	//}

	if (mCameraGLTexImage != NULL) {
        for (int i=0; i<6; i++){
    		if(mCameraGLTexImage[i] != NULL) {
    			mCameraGLTexImage[i]->clear();
    			delete mCameraGLTexImage[i];
    			mCameraGLTexImage[i] = NULL;
    		}
        }
        //delete[] mCameraGLTexImage;
	}
    DestroyEGL();
}


bool PerspectiveAdd::setMode(int homoMethod )
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


int PerspectiveAdd::Progress(Mat & _outMat, int* targetAddr, float _iso,int mode)
{
	mISO = _iso;
	workBegin();

    if(checkInitOpenGLES == false)
    {
        assert("Please initOpenGLES first!");
        return 0;
    }

	if (0) {
	char filename[100];
	for (int i=0; i<6; i++) {
		sprintf(filename, "/data/local/gray/%d.jpg", i);
		imwrite(filename, m_grays[i]);
		// Mat y(mHeight, mWidth, CV_8UC1, (void*)(mCameraGLTexImage[i]->mBufferAddr));
		// sprintf(filename, "/data/local/pic/%d%d.jpg", i, i);
		// imwrite(filename, y);
	}
	}

	LOGD("PerspectiveAdd mISO = %f",mISO);
    vector <HomIntMat> homIntMatVec;
    workBegin();
    MutGetHomography MTFeature(m_grays);
	// LOGD("%s(%d)-<%s>",__FILE__, __LINE__, __FUNCTION__);
    MTFeature.setMode(HomoMethod);
	// LOGD("%s(%d)-<%s>",__FILE__, __LINE__, __FUNCTION__);
    MTFeature.process(homIntMatVec,mISO);
    workEnd("MutGetHomography");

	// LOGD("%s(%d)-<%s>",__FILE__, __LINE__, __FUNCTION__);
    fHomography fhom;
    vector <fHomography> HomVec;
    HomVec.clear();
    float prtHomography[9] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
    memcpy(fhom.Homography,prtHomography, sizeof(prtHomography));
    HomVec.push_back(fhom);
    for(int k = 0; k < 5; k++)
    {
        int index = k;
        if(k > 1)
            index = k + 1;
        for(int i = 0; i < 5; i++)
        {
            if(homIntMatVec[i].index == index )
            {
                //LOGE("Homography = %d",index);
                double *temp = (double *)homIntMatVec[i].Homography.data;
                for(int j = 0;j < 9;j++)
                {
                    fhom.Homography[j] = *(double*)(temp+j);
                    //LOGE("Mat = %lf",fhom.Homography[j]);
                }
                HomVec.push_back(fhom);
                break;
            }
            else if(homIntMatVec[i].index == -1)
            {
                /* memcpy(fhom.Homography,prtHomography, sizeof(prtHomography));
               HomVec.push_back(fhom);*/
               LOGE("Homography not found.");
                _outMat = NULL;//m_images[3];
				mFoundHomography = false;
                return -1;
            }
        }
    }

	for (int i = 0; i < 6; i++) {
		LOGE("Homographys = %d",i);
		for(int j = 0;j < 9;j++) {
			LOGE("Mat = %lf",HomVec[i].Homography[j]);
		}
	}
	mFoundHomography = true;
    workBegin();
    perspectiveAndAdd(1, 0, 0.0f, 0.167f, HomVec,_outMat, targetAddr, mode);
	perspectiveAndAdd(2, 1, 1.0f, 0.167f, HomVec,_outMat, targetAddr, mode);
	perspectiveAndAdd(0, 2, 1.0f, 0.167f, HomVec,_outMat, targetAddr, mode);
	perspectiveAndAdd(3, 3, 1.0f, 0.167f, HomVec,_outMat, targetAddr, mode);
	perspectiveAndAdd(4, 4, 1.0f, 0.167f, HomVec,_outMat, targetAddr, mode);
	perspectiveAndAdd(5, 5, 1.0f, 0.167f, HomVec,_outMat, targetAddr, mode);

	if (mode == OUTPUT_NONE) {
    	glFinish();
	}

    workEnd("perspectiveAndAdd");

    return 1;
}

int PerspectiveAdd::updateImageData(struct cv_fimc_buffer *m_buffers_capture) {
	//workBegin();
	if (m_buffers_capture->length != mWidth * mHeight * 3 / 2) return 0;
    //m_grays.clear();
    //for (int i = 0; i < 6; i++)
    {
    	//LOGD("(%d) fill grey mat vector, fd: %d, w-h = %d-%d, start: %.8x, length: %d", gettid(), m_buffers_capture->share_fd, (int)mWidth, (int)mHeight, m_buffers_capture->start, m_buffers_capture->length);

		for (int i=0; i<6; i++) {
			if (mCameraGLTexImage[i] == NULL) {
				mCurrentId = i;
				break;
			}

			if (mCameraGLTexImage[i]->mShareFd > 0 && mCameraGLTexImage[i]->mShareFd == m_buffers_capture->share_fd) {
				mCurrentId = i;
				break;
			}
		}
/*
		if (mCameraGLTexImage[mCurrentId] != NULL) {
			if (id != (mCurrentId - 1)) {
				if (id != (mCurrentId + 5)) {
					LOGD("##########WARNNING#################!@#$%^&*((()!@#$%^&*()!@#$%^&*())!@#$%^&*((((()!@#$%^&*******(((((!@#$");
				}
			}
		}
*/
		if (mCameraGLTexImage[mCurrentId] == NULL) {
			LOGD("create image id: %d", mCurrentId);
		    mCameraGLTexImage[mCurrentId] = new CameraGLTexImage(mCurrentId, display, (char*)m_buffers_capture->start, (m_buffers_capture->handle), m_buffers_capture->share_fd);
		    mCameraGLTexImage[mCurrentId]->createTexImage(mWidth, mHeight, HAL_PIXEL_FORMAT_YCrCb_NV12);//HAL_PIXEL_FORMAT_YV12);//HAL_PIXEL_FORMAT_YCrCb_420_SP//HAL_PIXEL_FORMAT_YCrCb_NV12);
		}
		checkGlError("createTexImage");
		//LOGD("create tex image done");
	    if (m_buffers_capture->handle == NULL && m_buffers_capture->share_fd == 0) {
			LOGD("update image id: %d", mCurrentId);
	    	mCameraGLTexImage[mCurrentId]->updateTexImage(mWidth, mHeight, HAL_PIXEL_FORMAT_YV12);//HAL_PIXEL_FORMAT_YCrCb_NV12);
	    }
		checkGlError("updateTexImage");
		//LOGD("update tex image done");

        Mat y(mHeight, mWidth, CV_8UC1, (void*)(m_buffers_capture->start));
		m_grays[mCurrentId] = y.clone();

		//LOGD("fill grey mat vector, fd: %d, shared_fd: %d", mCurrentId, m_buffers_capture->share_fd);
/*
		mCurrentId++;
		if (mCurrentId > 5) {
            mCurrentId = 0;
		}
*/
	}
	checkGlError("updateImageData");
	//workEnd("update-image");
	return 0;
}

int PerspectiveAdd::initOpenGLES(alloc_device_t *m_alloc_dev, int width, int height)
{
    checkInitOpenGLES = true;

    mWidth = width;
    mHeight = height;

    // Init EGL display, surface and context
    if(!InitEGL())
    {
        LOGE("Init EGL fail\n");
        return GL_FALSE;
    }

    int activeTexNum = 0;
    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexNum);

    int activeOESUnit = 0;
    //glGetTexParameteriv(GL_TEXTURE_EXTERNAL_OES, REQUIRED_TEXTURE_IMAGE_UNITS_OES, &activeOESUnit);
    LOGD("ACTIVE TEX NUM: %d, active OES unit: %d", activeTexNum, activeOESUnit);

    programObject = LoadProgram(gPerspectiveVertexShader,gPerspectiveFragmentShader);
    glUseProgram(programObject);
    checkGlError("LoadProgram");

    //get attribution unit index
    vPositionHandle = glGetAttribLocation(programObject, "a_position" );
    vTexCoordHandle = glGetAttribLocation(programObject, "a_texCoord" );
	vHomograyHandle = glGetUniformLocation(programObject, "uMVPMatrix");

	/*
    vHomograyHandle1 = glGetUniformLocation(programObject, "uMVPMatrix1");
    vHomograyHandle2 = glGetUniformLocation(programObject, "uMVPMatrix2");
    vHomograyHandle3 = glGetUniformLocation(programObject, "uMVPMatrix3");
    vHomograyHandle4 = glGetUniformLocation(programObject, "uMVPMatrix4");
    vHomograyHandle5 = glGetUniformLocation(programObject, "uMVPMatrix5");
    vHomograyHandle6 = glGetUniformLocation(programObject, "uMVPMatrix6");
    */
    vSizeHandle = glGetUniformLocation(programObject, "textureSize");

    checkGlError("initOpenGLES-glGetUniformLocation");

    float gSize[2] = {mWidth,mHeight};
    glUniform2fv(vSizeHandle,1,gSize);

    // Set the sampler texture unit index
    /*
    glUniform1i(glGetUniformLocation(programObject, "u_samplerTexture1"), 1);
    glUniform1i(glGetUniformLocation(programObject, "u_samplerTexture2"), 2);
    glUniform1i(glGetUniformLocation(programObject, "u_samplerTexture3"), 3);
    glUniform1i(glGetUniformLocation(programObject, "u_samplerTexture4"), 4);
    glUniform1i(glGetUniformLocation(programObject, "u_samplerTexture5"), 5);
    glUniform1i(glGetUniformLocation(programObject, "u_samplerTexture6"), 6);
	*/
    int location;
    location = glGetUniformLocation(programObject, "texture");
        glUniform1i(location, 7);
        LOGD("glGetUniformLocation texture: %d", location);

    mTextureLocation = glGetUniformLocation(programObject, "u_samplerTexture");
	    glUniform1i(mTextureLocation, 0);
	    LOGD("glGetUniformLocation u_samplerTexture: %d", mTextureLocation);

	mLayerNumLocation = glGetUniformLocation(programObject, "layerNum");
		glUniform1i(mTextureLocation, 0);
	    LOGD("glGetUniformLocation u_samplerTexture: %d", mTextureLocation);

    mParam1location = glGetUniformLocation(programObject, "param1");
        glUniform1f(mParam1location, 0.0f);
        LOGD("glGetUniformLocation param1: %d", mParam1location);
    mParam2location = glGetUniformLocation(programObject, "param2");
        glUniform1f(mParam2location, 0.167f);
        LOGD("glGetUniformLocation param2: %d", mParam2location);
    checkGlError("initOpenGLES-glUniform");

    initializeTmpResEGLImage(m_alloc_dev, (int) mWidth, (int) mHeight, &targetTexId, &fboTargetHandle, GL_TEXTURE7);
    checkGlError("initializeTmpResEGLImage");

	//updateImageData(m_buffers_capture);

	LOGD("init egl env done.");

    return GL_TRUE;
}

void PerspectiveAdd::initializeTmpResEGLImage(alloc_device_t *m_alloc_dev, int fboWidth, int fboHeight, GLuint *tex,
                                        GLuint * fbo, GLuint texGroup)
{
    glGenTextures(1, tex);
    glActiveTexture(texGroup);
    checkGlError("initializeTmpResEGLImage-gentex");
    glBindTexture(GL_TEXTURE_2D, *tex);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    checkGlError("initializeTmpResEGLImage-inittex");

	if (m_alloc_dev == NULL) {
#if 1
        mTargetBufferHandle = bufferHandleAlloc(fboWidth, fboHeight, HAL_PIXEL_FORMAT_RGBA_8888, GraphicBuffer::USAGE_HW_TEXTURE);
        mTargetGraphicBuffer = new GraphicBuffer(fboWidth, fboHeight, HAL_PIXEL_FORMAT_RGBA_8888,
            GraphicBuffer::USAGE_HW_TEXTURE, fboWidth, (native_handle_t*)mTargetBufferHandle, false);
#else
	    mTargetGraphicBuffer = new GraphicBuffer(fboWidth, fboHeight, HAL_PIXEL_FORMAT_RGBA_8888,
	                                             GraphicBuffer::USAGE_HW_TEXTURE | GraphicBuffer::USAGE_SW_WRITE_RARELY);
#endif
/*
	    int err = mTargetGraphicBuffer->lock(GRALLOC_USAGE_SW_READ_RARELY, (void **) (&mTargetGraphicBufferAddr));
	    if (err != 0 || mTargetGraphicBufferAddr == NULL)
	    {
	        LOGD("mYUVTexBuffer->lock(...) failed: %d\n", err);
	        return;
	    }
	    memset(mTargetGraphicBufferAddr, 0, fboWidth* fboHeight*3);
	    err = mTargetGraphicBuffer->unlock();
	    if (err != 0)
	    {
	        LOGD("mYUVTexBuffer->unlock() failed: %d\n", err);
	        return;
	    }
*/
	} else {
	 	int stride;
		int err = m_alloc_dev->alloc(m_alloc_dev,
	                                         fboWidth,
	                                         fboHeight,
	                                         HAL_PIXEL_FORMAT_RGBA_8888,
	                                         GraphicBuffer::USAGE_HW_TEXTURE,
	                                         &mTargetBufferHandle,
	                                         &stride);

		mTargetGraphicBuffer = new GraphicBuffer(fboWidth, fboHeight, HAL_PIXEL_FORMAT_RGBA_8888,
	                GraphicBuffer::USAGE_HW_TEXTURE, fboWidth, (native_handle_t*)mTargetBufferHandle, false);

	}

    if (mTargetEGLImage == EGL_NO_IMAGE_KHR) {
        EGLClientBuffer clientBuffer = (EGLClientBuffer)mTargetGraphicBuffer->getNativeBuffer();
        mTargetEGLImage = eglCreateImageKHR(display, EGL_NO_CONTEXT, EGL_NATIVE_BUFFER_ANDROID,
                                                clientBuffer, 0);
    }

    //glBindTexture(GL_TEXTURE_EXTERNAL_OES, _textureid);
    glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, (GLeglImageOES)mTargetEGLImage);
    checkGlError("initializeTmpResEGLImage-glEGLImageTargetTexture2DOES");

    glGenFramebuffers(1, fbo);
    LOGD("generate tex/fbo for target tex id: %d, fbo id: %d, w-h: %d-%d", *tex, *fbo, fboWidth, fboHeight);
    glBindFramebuffer(GL_FRAMEBUFFER, *fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, *tex, 0);
    checkGlError("initializeTmpResEGLImage-glFramebufferTexture2D");

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        LOGD("framebuffer statuc check fail: %d", status);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

int PerspectiveAdd::rebindFrameBufferObject(int fboTex, int fboId, long address) {
	checkGlError("rebindFrameBufferObject");
	int targetId = -1;
	for (int i=0; i<6; i++) {
		if ((unsigned long)(mCameraGLTexImage[i]->mBufferAddr) == address) {
			ALOGD("found target id: %d", i);
			targetId = i;
		}
	}

	if (targetId == -1) {
		LOGD("capture frame not found.");
		return -1;
	}

	glBindTexture(GL_TEXTURE_EXTERNAL_OES, fboTex);
	glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, (GLeglImageOES)mCameraGLTexImage[targetId]->mEGLImage);

    glBindFramebuffer(GL_FRAMEBUFFER, fboId);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTex, 0);
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        LOGD("framebuffer status check fail: %d", status);
    }
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return targetId;
}


#define CLAMP(a, b, c) (a > c ? c : (a < b ? b : a))
#define MY(a,b,c) (( int(a)*  0.2989  + int(b)*  0.5866  + int(c)*  0.1145))
#define MU(a,b,c) (( int(a)*(-0.1688) + int(b)*(-0.3312) + int(c)*  0.5000 + 128))
#define MV(a,b,c) (( int(a)*  0.5000  + int(b)*(-0.4184) + int(c)*(-0.0816) + 128))

#define DY(a) CLAMP(MY(a&0xff, a>>8&0xff, a>>16&0xff), 0, 255)//(MY(a,b,c) > 255 ? 255 : (MY(a,b,c) < 0 ? 0 : MY(a,b,c)))
#define DU(a) CLAMP(MU(a&0xff, a>>8&0xff, a>>16&0xff), 0, 255)//(MU(a,b,c) > 255 ? 255 : (MU(a,b,c) < 0 ? 0 : MU(a,b,c)))
#define DV(a) CLAMP(MV(a&0xff, a>>8&0xff, a>>16&0xff), 0, 255)//(MV(a,b,c) > 255 ? 255 : (MV(a,b,c) < 0 ? 0 : MV(a,b,c)))

static void RGBA2YUV420SP(int width, int height, char *RGB, char *YUV)
{
    int i,x,y,j;
    char *Y = NULL;
    char *UV = NULL;

    Y = YUV;
    UV = YUV + width*height;

	int a = *((int*)RGB);
	LOGD("first point, r-g-b: %.8x, %d-%d-%d-%d", a, a&0xff, a>>8&0xff, a>>16&0xff, a>>24&0xff);

    for(y=0; y < height; y++) {
        for(x=0; x < width; x++) {
            j = y*width + x;
            i = j*4;

            Y[j] = (unsigned char)(DY(*((int*)(RGB+i))));

            if(x%2 == 1 && y%2 == 1)
            {
                j = (width>>1) * (y>>1) + (x>>1);

                UV[j*2] = (unsigned char)
                       ((DU(*((int*)(RGB+i))) +
                         DU(*((int*)(RGB+i) - 1)) +
                         DU(*((int*)(RGB+i) - width)) +
                         DU(*((int*)(RGB+i) - width - 1)))/4);

                UV[j*2+1] = (unsigned char)
						((DV(*((int*)(RGB+i))) +
						  DV(*((int*)(RGB+i) - 1)) +
						  DV(*((int*)(RGB+i) - width)) +
						  DV(*((int*)(RGB+i) - width - 1)))/4);

            }
        }
    }
}

//render in here
int PerspectiveAdd::perspectiveAndAdd(int index, int texIndex, float param1, float param2, const vector <fHomography> & HomographyVec, Mat &dstImage, int* targetAddr, int mode)
{
	LOGD("%s(%d)-<%s>, tid(%d)",__FILE__, __LINE__, __FUNCTION__, gettid());
	checkGlError("perspectiveAndAdd");

    //workBegin();
    glUniformMatrix3fv(vHomograyHandle,1,GL_FALSE,HomographyVec[index].Homography);
	glUniform1f(mParam1location, param1);
	glUniform1f(mParam2location, param2);
	glUniform1i(mTextureLocation, texIndex);
	glUniform1f(mLayerNumLocation, (float)texIndex);

	/*
    glUniformMatrix3fv(vHomograyHandle1,1,GL_FALSE,HomographyVec[1].Homography);
    glUniformMatrix3fv(vHomograyHandle2,1,GL_FALSE,HomographyVec[2].Homography);
    glUniformMatrix3fv(vHomograyHandle3,1,GL_FALSE,HomographyVec[0].Homography);
    glUniformMatrix3fv(vHomograyHandle4,1,GL_FALSE,HomographyVec[3].Homography);
    glUniformMatrix3fv(vHomograyHandle5,1,GL_FALSE,HomographyVec[4].Homography);
    glUniformMatrix3fv(vHomograyHandle6,1,GL_FALSE,HomographyVec[5].Homography);
    */

	checkGlError("perspectiveAndAdd setviewport");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //begin to render
    glViewport(0, 0, mWidth, mHeight);
    const GLfloat vVertices[] = {
            // X,  Y, Z, W,    U, V
            1,  1, 0, 1,    1, 1, //Top Right
            -1,  1, 0, 1,    0, 1, //Top Left
            -1, -1, 0, 1,    0, 0, // Bottom Left
            1, -1, 0, 1,    1, 0 //Bottom Right
    };

    GLushort indices[] = { 0, 3, 2, 2, 1, 0 };
    GLsizei stride = 6*sizeof(GLfloat);

	int targetId = -1;// = rebindFrameBufferObject(targetTexId, fboTargetHandle, (long)targetAddr);
	checkGlError("perspectiveAndAdd rebindFrameBufferObject");
    glBindFramebuffer(GL_FRAMEBUFFER, fboTargetHandle);
    glClearColor(1.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Load the vertex position
    glVertexAttribPointer(vPositionHandle, 4, GL_FLOAT, GL_FALSE, stride,
                          vVertices);
    // Load the texture coordinate
    glVertexAttribPointer(vTexCoordHandle, 2, GL_FLOAT, GL_FALSE, stride,
                          &vVertices[4]);
    glEnableVertexAttribArray(vPositionHandle);
    glEnableVertexAttribArray(vTexCoordHandle);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices);
	if (mode != OUTPUT_NONE) {
    	glFinish();
	}
    glDisableVertexAttribArray(vPositionHandle);
    glDisableVertexAttribArray(vTexCoordHandle);
    //workEnd();

	checkGlError("perspectiveAndAdd draw");
	if (targetId >= 0) {
		int err = mCameraGLTexImage[targetId]->mGraphicBuffer->lock(GRALLOC_USAGE_SW_READ_RARELY, (void **) (&mTargetGraphicBufferAddr));
	    if (err != 0 || mTargetGraphicBufferAddr == NULL)
	    {
	        LOGD("mYUVTexBuffer->lock(...) failed: %d\n", err);
	        return -1;
	    }
	    Mat result(mHeight, mWidth, CV_8UC4, mTargetGraphicBufferAddr);
	    dstImage = result.clone();
	    //dstImage = result;

		//RGBA2YUV420SP(mWidth, mHeight, (char*)mTargetGraphicBufferAddr, (char*)targetAddr);

		//memcpy(targetAddr, mTargetGraphicBufferAddr, mHeight * mWidth * 1.5f);

	    err = mCameraGLTexImage[targetId]->mGraphicBuffer->unlock();
	    if (err != 0)
	    {
	        LOGD("mYUVTexBuffer->unlock() failed: %d\n", err);
	        return -1;
	    }
	} else {
		if (mode == OUTPUT_NONE) {
			// do nothing
			return 0;
		}
	    int err = mTargetGraphicBuffer->lock(GRALLOC_USAGE_SW_READ_RARELY, (void **) (&mTargetGraphicBufferAddr));
	    if (err != 0 || mTargetGraphicBufferAddr == NULL)
	    {
	        LOGD("mYUVTexBuffer->lock(...) failed: %d\n", err);
	        return -1;
	    }
		if (mode == OUTPUT_FMT_CV_MAT) {
		    Mat result(mHeight, mWidth, CV_8UC4, mTargetGraphicBufferAddr);
		    dstImage = result.clone();
		    //dstImage = result;
		}

		if (mode == OUTPUT_FMT_YUV) {
			rga_copy_and_scale(mWidth, mHeight, (int)mTargetGraphicBufferAddr, RK_FORMAT_RGBA_8888,
												mWidth, mHeight, (int)targetAddr, RK_FORMAT_YCbCr_420_SP);//RK_FORMAT_YCrCb_420_SP);
		}

		if (mode == OUTPUT_FMT_RGBA) {
			/*

			targetAddr = (int)mTargetGraphicBufferAddr;
			FILE* fp = fopen("/data/local/result.data", "ab+");
			if(fp == NULL)
		    {
		        LOGD("open /data/local/result.data failed\n");
		    } else {
		    	LOGD("write /data/local/result.data\n");
				fwrite(mTargetGraphicBufferAddr, 1, mHeight * mWidth * 4, fp);
				fclose(fp);
		    }
		    */
		    Mat result(mHeight, mWidth, CV_8UC4, mTargetGraphicBufferAddr);
			imwrite("/data/local/result.jpg", result);

			*targetAddr = (int)mTargetGraphicBufferAddr;
			//memcpy(targetAddr, mTargetGraphicBufferAddr, mHeight * mWidth * 4);
		}
		//RGBA2YUV420SP(mWidth, mHeight, (char*)mTargetGraphicBufferAddr, (char*)targetAddr);
		//memcpy(targetAddr, mTargetGraphicBufferAddr, mHeight * mWidth * 1.5f);
		if (mode == OUTPUT_NONE) {
			//Mat result(mHeight, mWidth, CV_8UC4, mTargetGraphicBufferAddr);
			//imwrite("/data/local/result.jpg", result);
		}

	    err = mTargetGraphicBuffer->unlock();
	    if (err != 0)
	    {
	        LOGD("mYUVTexBuffer->unlock() failed: %d\n", err);
	        return -1;
	    }
	}
    //workEnd("COMPOSITION");

    return GL_TRUE;
}

int PerspectiveAdd::getResult(int targetAddr) {
	if (mFoundHomography) {
	    LOGD("PerspectiveAdd::getResult start");
		int err = mTargetGraphicBuffer->lock(GRALLOC_USAGE_SW_READ_RARELY, (void **) (&mTargetGraphicBufferAddr));
		if (err != 0 || mTargetGraphicBufferAddr == NULL)
		{
			LOGD("mYUVTexBuffer->lock(...) failed: %d\n", err);
			return -1;
		}

		//Mat result(mHeight, mWidth, CV_8UC4, mTargetGraphicBufferAddr);
		//imwrite("/data/local/result.jpg", result);
		 rga_copy_and_scale(mWidth, mHeight, (int)mTargetGraphicBufferAddr, RK_FORMAT_RGBA_8888,
									mWidth, mHeight, targetAddr, RK_FORMAT_YCbCr_420_SP);//RK_FORMAT_YCrCb_420_SP);

		err = mTargetGraphicBuffer->unlock();
		if (err != 0)
		{
			LOGD("mYUVTexBuffer->unlock() failed: %d\n", err);
			return -1;
		}
	    LOGD("PerspectiveAdd::getResult end");
	}
	return 0;
}

int PerspectiveAdd::DestroyEGL()
{
    //Typical egl cleanup
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    //eglDestroySurface(display, mSurface);
    eglDestroyContext(display, context);
    eglTerminate(display);
    eglReleaseThread();

	return 0;
}

int PerspectiveAdd::InitEGL()
{

    NativeWindowType eglWindow = NULL;
    surface = NULL;

    EGLConfig configs[2];
    EGLBoolean eRetStatus;
    EGLint majorVer, minorVer;
    EGLint context_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};

    EGLint numConfigs;
    EGLint cfg_attribs[] = {EGL_BUFFER_SIZE,    EGL_DONT_CARE,
                            EGL_DEPTH_SIZE,     16,
                            EGL_RED_SIZE,       8,
                            EGL_GREEN_SIZE,     8,
                            EGL_BLUE_SIZE,      8,
                            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
                            EGL_NONE};

    // Get default display connection
    display = eglGetDisplay((EGLNativeDisplayType)EGL_DEFAULT_DISPLAY);
    if ( display == EGL_NO_DISPLAY )
    {
        return EGL_FALSE;
    }

    // Initialize EGL display connection
    eRetStatus = eglInitialize(display, &majorVer, &minorVer);
    if( eRetStatus != EGL_TRUE )
    {
        return EGL_FALSE;
    }
#ifdef ONLINE
    //Get a list of all EGL frame buffer configurations for a display
    eRetStatus = eglGetConfigs (display, configs, 2, &numConfigs);
    if( eRetStatus != EGL_TRUE )
    {
        return EGL_FALSE;
    }

    // Get a list of EGL frame buffer configurations that match specified attributes
    eRetStatus = eglChooseConfig (display, cfg_attribs, configs, 2, &numConfigs);
    if( eRetStatus != EGL_TRUE  || !numConfigs)
    {
        return EGL_FALSE;
    }
#ifdef ANDROID5X
    sp<IGraphicBufferProducer> producer;
    sp<IGraphicBufferConsumer> consumer;
    BufferQueue::createBufferQueue(&producer, &consumer);
    sp<GLConsumer> mST = new GLConsumer(consumer, 123, GLConsumer::TEXTURE_EXTERNAL, true,
            false);
    mST->setDefaultBufferSize(64, 64);
    mST->setDefaultBufferFormat(HAL_PIXEL_FORMAT_RGBA_8888);
    sp<Surface> mSTC = new Surface(producer);
#else
	sp<BufferQueue> bq = new BufferQueue();
	sp<GLConsumer> mST = new GLConsumer(bq, 123);
	mST->setDefaultBufferSize(64, 64);
	mST->setDefaultBufferFormat(HAL_PIXEL_FORMAT_RGBA_8888);
	sp<Surface> mSTC = new Surface(bq);
#endif
    sp<ANativeWindow> window = mSTC.get();
    eglWindow = window.get();

    //LOGE("eglCreateWindowSurface");
    // Create a new EGL window surface
    surface = eglCreateWindowSurface(display, configs[0], eglWindow, NULL);
    //if (surface == EGL_NO_SURFACE)
    //{
    //return EGL_FALSE;
    //}
#else
    EGLint attribListPbuffer[] = {
        EGL_WIDTH, mWidth,
        EGL_HEIGHT, mHeight,
        EGL_NONE
    };

    if (eglChooseConfig(display, cfg_attribs, &configs[0], 1, &numConfigs) == EGL_FALSE) {
        ALOGD("EGLUtils::selectConfigForNativeWindow() failed.");
        return EGL_FALSE;
    }

    surface = eglCreatePbufferSurface(display, configs[0], attribListPbuffer);
    checkEglError("eglCreatePbufferFromClientBuffer");
    if (surface == EGL_NO_SURFACE) {
        ALOGD("eglCreatePbufferSurface failed.\n");
        return EGL_FALSE;
    }
#endif
    // Set the current rendering API (EGL_OPENGL_API, EGL_OPENGL_ES_API,EGL_OPENVG_API)
    eRetStatus = eglBindAPI(EGL_OPENGL_ES_API);
    if (eRetStatus != EGL_TRUE)
    {
        return EGL_FALSE;
    }

    // Create a new EGL rendering context
    context = eglCreateContext (display, configs[0], EGL_NO_CONTEXT, context_attribs);
    if (context == EGL_NO_CONTEXT)
    {
        return EGL_FALSE;
    }

    // Attach an EGL rendering context to EGL surfaces
    eRetStatus = eglMakeCurrent (display, surface, surface, context);
    if( eRetStatus != EGL_TRUE )
    {
        return EGL_FALSE;
    }
    //If interval is set to a value of 0, buffer swaps are not synchronized to a video frame, and the swap happens as soon as the render is complete.
    eglSwapInterval(display,0);


    return EGL_TRUE;
}
/*
GLuint PerspectiveAdd::SetupGraphic() {

}
*/

GLuint PerspectiveAdd::LoadShader( GLenum type, const char *shaderSrc )
{
    GLuint shader;
    GLint compiled;

    // Create an empty shader object, which maintain the source code strings that define a shader
    shader = glCreateShader ( type );

    if ( shader == 0 )
        return 0;

    // Replaces the source code in a shader object
    glShaderSource ( shader, 1, &shaderSrc, NULL );
    // Compile the shader object
    glCompileShader ( shader );

    // Check the shader object compile status
    glGetShaderiv ( shader, GL_COMPILE_STATUS, &compiled );

    if ( !compiled )
    {
        GLint infoLen = 0;
        glGetShaderiv ( shader, GL_INFO_LOG_LENGTH, &infoLen );

        if ( infoLen > 1 )
        {
            char* infoLog = (char*)malloc (sizeof(char) * infoLen );
            glGetShaderInfoLog ( shader, infoLen, NULL, infoLog );
            LOGE ( "Error compiling shader:\n%s\n", infoLog );
            free ( infoLog );
        }

        glDeleteShader ( shader );
        return 0;
    }

    return shader;
}

GLuint PerspectiveAdd::LoadProgram( const char *vShaderStr, const char *fShaderStr )
{
    GLuint vertexShader;
    GLuint fragmentShader;
    GLuint programObject;
    GLint linked;

    // Load the vertex/fragment shaders
    vertexShader = LoadShader ( GL_VERTEX_SHADER, vShaderStr );
    fragmentShader = LoadShader ( GL_FRAGMENT_SHADER, fShaderStr );

    // Create the program object
    programObject = glCreateProgram( );
    if ( programObject == 0 )
        return 0;

    // Attaches a shader object to a program object
    glAttachShader ( programObject, vertexShader );
    glAttachShader ( programObject, fragmentShader );

    // Link the program object
    glLinkProgram ( programObject );

    // Check the link status
    glGetProgramiv ( programObject, GL_LINK_STATUS, &linked );

    if ( !linked )
    {
        GLint infoLen = 0;
        glGetProgramiv ( programObject, GL_INFO_LOG_LENGTH, &infoLen );

        if ( infoLen > 1 )
        {
            char* infoLog = (char*)malloc (sizeof(char) * infoLen );
            glGetProgramInfoLog ( programObject, infoLen, NULL, infoLog );
            LOGE ( "Error linking program:\n%s\n", infoLog );
            free ( infoLog );
        }
        glDeleteProgram ( programObject );
        return GL_FALSE;
    }

    // Free no longer needed shader resources
    glDeleteShader ( vertexShader );
    glDeleteShader ( fragmentShader );

    return programObject;
}

PerspectiveAdd::CameraGLTexImage::CameraGLTexImage(int id, EGLDisplay display, char* bufAddr, buffer_handle_t handle, int share_fd):
    mId(id),
	mShareFd(share_fd),
	mEmptyShareFd(0),
    eglDisplay(display),
    mBufferAddr(bufAddr),
    mGraphicBuffer(NULL),
    mHandle(handle),
    mEGLImage(EGL_NO_IMAGE_KHR)
{
};
PerspectiveAdd::CameraGLTexImage::~CameraGLTexImage() 
{
}
void PerspectiveAdd::CameraGLTexImage::clear() {
    ALOGD("destroy CameraGLTexImage, id: %d", mId);
	//add if by linqi in 2016-8-22
	if(eglDisplay != NULL) {
    	eglDestroyImageKHR(eglDisplay, mEGLImage);
    }

	if(mHandle != NULL) {
    	ALOGD("clean empty unused buffer, id: %d", mId);
        private_handle_t* pHnd = (private_handle_t *) mHandle;
        // backup shared fd.
        pHnd->share_fd = mEmptyShareFd;
		bufferHandleFree(mHandle);
		mHandle = NULL;
    }
    mGraphicBuffer = NULL;

    mEGLImage = EGL_NO_IMAGE_KHR;
};

int PerspectiveAdd::CameraGLTexImage::createTexImage(int width, int height, int format) {
    if (eglDisplay == NULL) return -1;
	ALOGD("createTexImage, w-h: %d-%d, fmt: %d", width, height, format);
	GLuint textureId;
	glGenTextures(1, &textureId);
	glActiveTexture(TEXTURE_GROUP[mId]);


    glBindTexture(GL_TEXTURE_EXTERNAL_OES, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (mHandle == NULL) {
		if (mShareFd != 0) {
			ALOGD("createTexImage, fd: %d", mShareFd);
			mHandle = bufferHandleAlloc(width, height, format, GraphicBuffer::USAGE_HW_TEXTURE);
			private_handle_t* pHnd = (private_handle_t *) mHandle;
			// backup shared fd.
            mEmptyShareFd = pHnd->share_fd;
			pHnd->share_fd = mShareFd;
			mGraphicBuffer = new GraphicBuffer(width, height, format, //mHandle->format,
                GraphicBuffer::USAGE_HW_TEXTURE, width, (native_handle_t*)mHandle, false);
		} else {
	        mGraphicBuffer = new GraphicBuffer(width, height, format,
	                GraphicBuffer::USAGE_HW_TEXTURE);
		}
    } else {
        mGraphicBuffer = new GraphicBuffer(width, height, format, //mHandle->format,
                GraphicBuffer::USAGE_HW_TEXTURE, width, (native_handle_t*)mHandle, false);

    }

    if (mEGLImage == EGL_NO_IMAGE_KHR) {
		ALOGD("createTexImage, eglimg");
        EGLClientBuffer clientBuffer = (EGLClientBuffer)mGraphicBuffer->getNativeBuffer();
        mEGLImage = eglCreateImageKHR(eglDisplay, EGL_NO_CONTEXT, EGL_NATIVE_BUFFER_ANDROID,
                clientBuffer, 0);
    }
	checkGlError("createTexImage eglCreateImageKHR");
	glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, (GLeglImageOES)mEGLImage);
	//glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, (GLeglImageOES)mEGLImage);
	checkGlError("createTexImage glEGLImageTargetTexture2DOES");

    return 0;
}

int PerspectiveAdd::CameraGLTexImage::updateTexImage(int width, int height, int format) {
	ALOGD("updateTexImage, w-h: %d-%d, fmt: %d, addr: %.8x", width, height, format, mBufferAddr);
    char* buf = NULL;
    status_t err = mGraphicBuffer->lock(GRALLOC_USAGE_SW_WRITE_RARELY, (void**)(&buf));
    if (err != 0) {
        ALOGD("yuvTexBuffer->lock(...) failed: %d\n", err);
        return -1;
    }
#if CPU_INPUT
    if (format == HAL_PIXEL_FORMAT_YCrCb_420_SP) {
        memcpy(buf, mBufferAddr, width*height*3/2);
    } else if (format == HAL_PIXEL_FORMAT_RGBA_8888) {
        int* img = (int*)malloc(width * height * sizeof(int));
        YUV420SP2RGBA(width, height, (unsigned char*)img, (unsigned char*)mBufferAddr);
        memcpy(buf, img, width*height*4);
        free(img);
    } else {
        memcpy(buf, mBufferAddr, width*height*3/2);
    }
#elif RGA_INPUT
#if PLATFORM_ARM
    rga_copy_and_scale(width, height, (int)mBufferAddr, RK_FORMAT_YCbCr_420_SP,
                          width, height, (int)buf, RK_FORMAT_YCbCr_420_SP);
#else
    rgaRawDataProcessTiled(width, height, (int)mBufferAddr, RK_FORMAT_YCbCr_420_SP, (int)buf, RK_FORMAT_YCbCr_420_SP);
#endif
#else
	memcpy(buf, mBufferAddr, width*height*3/2);
#endif
    err = mGraphicBuffer->unlock();
    if (err != 0) {
        ALOGD("yuvTexBuffer->unlock() failed: %d\n", err);
        return -1;
    }
    return 0;
}


buffer_handle_t PerspectiveAdd::CameraGLTexImage::bufferHandleAlloc(uint32_t w, uint32_t h, PixelFormat format, uint32_t usage) {

    buffer_handle_t handle;
    int stride;
    GraphicBufferAllocator& allocator = GraphicBufferAllocator::get();
    status_t err = allocator.alloc(w, h, format, usage, &handle, &stride);
    fprintf(stderr, "bufferHandleAlloc status: %d stride = %d, handle = %p\n", err, stride, handle);
    if (err == NO_ERROR) {
        return handle;
    }
    return NULL;
}

void PerspectiveAdd::CameraGLTexImage::bufferHandleFree(buffer_handle_t handle) {

    GraphicBufferAllocator& allocator = GraphicBufferAllocator::get();
    status_t err = allocator.free(handle);
}


void PerspectiveAdd::checkFBO()
{
    // FBO status check
    GLenum status;
    status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch(status)
    {
        case GL_FRAMEBUFFER_COMPLETE:
            LOGE("fbo complete");
            break;
        case GL_FRAMEBUFFER_UNSUPPORTED:
            LOGE("fbo unsupported");
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
            LOGE("GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
            LOGE("GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE_IMG:
            LOGE("GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE_IMG");
            break;
        default:
            LOGE("Framebuffer Error");
            break;
    }
}

void PerspectiveAdd::workBegin()
{
    work_begin = getTickCount();
}

void PerspectiveAdd::workEnd(char* module_name)
{
    work_end = getTickCount() - work_begin;
    double Time = work_end /((double)getTickFrequency() )* 1000.0;
    LOGE("[%s] TIME = %lf ms \n", module_name, Time);
}
