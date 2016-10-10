//
// Created by linqi on 16-4-5.
//

#ifndef MY_JNI_PERSPECTIVEADD_H
#define MY_JNI_PERSPECTIVEADD_H



#include "opencv2/opencv.hpp"
#include <android/log.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES2/gl2platform.h>
#include <stdio.h>
#include <stdlib.h>
#include <ui/GraphicBuffer.h>
#include <ui/PixelFormat.h>
#include "log.h"
#include "include/MyStruct.h"
#include "include/MutGetHomography.h"
#include <MutliFrameDenoise.h>
#include <hardware/gralloc.h>


using namespace android;
using namespace mystruct;
using namespace cv;

using android::GraphicBuffer;
using android::sp;

const char gPerspectiveVertexShader[] =
        "attribute vec4 a_position;\n"
                "attribute vec2 a_texCoord;\n"
                "varying vec2 texCoord;\n"
                "void main() {\n"
                "  texCoord = a_texCoord;\n"
                "  gl_Position = a_position;\n"
                "}\n";

const char gPerspectiveFragmentShader[] =
        "#extension GL_OES_EGL_image_external : require\n"
        "precision highp float;\n"
                "uniform mat3 uMVPMatrix;\n"
                "uniform vec2 textureSize;\n"
                "varying vec2 texCoord;\n"
                "uniform sampler2D texture;\n"
                "uniform samplerExternalOES u_samplerTexture;\n"
                "uniform float layerNum;\n"
                "uniform float param1;\n"
                "uniform float param2;\n"

                "void main() {\n"
                "  float width = textureSize.x;\n"
                "  float height = textureSize.y;\n"
                "  vec3 sPoint = vec3((texCoord.x)*width,(texCoord.y)*height,1.0);\n"
                "  float x = sPoint.x*uMVPMatrix[0][0]+sPoint.y*uMVPMatrix[0][1]+uMVPMatrix[0][2];\n"
                "  float y = sPoint.x*uMVPMatrix[1][0]+sPoint.y*uMVPMatrix[1][1]+uMVPMatrix[1][2];\n"
                "  float z = sPoint.x*uMVPMatrix[2][0]+sPoint.y*uMVPMatrix[2][1]+uMVPMatrix[2][2];\n"
                "  float X = x/(z*width);\n"
                "  float Y = y/(z*height);\n"
                "  vec2 v_texCoord = vec2(X,Y);\n"
                "gl_FragColor = texture2D(texture,texCoord)*param1;\n"
                "gl_FragColor += texture2D(u_samplerTexture,v_texCoord)*param2;\n"
                "}\n";


// const char gPerspectiveVertexShader[] =
//         "attribute vec4 a_position;\n"
//                 "uniform mat3 uMVPMatrix;\n"
//                 "uniform vec2 textureSize;\n"
//                 "attribute vec2 a_texCoord;\n"
//                 "varying vec2 v_texCoord;\n"
//                 "varying vec2 texCoord;\n"
//                 "void main() {\n"
//                 "  texCoord = a_texCoord;\n"
//                 "  a_homography = uMVPMatrix;\n"
//                 "  float width = textureSize.x;\n"
//                 "  float height = textureSize.y;\n"
//                 "  vec3 sPoint = vec3((a_texCoord.x)*width,(a_texCoord.y)*height,1.0);\n"
//                 // "  vec3 dPoint = uMVPMatrix*sPoint;\n"
//                 // "  float X = dPoint.x/(dPoint.z*width);\n"
//                 // "  float Y = dPoint.y/(dPoint.z*height);\n"
//                 "  float x = sPoint.x*uMVPMatrix[0][0]+sPoint.y*uMVPMatrix[0][1]+uMVPMatrix[0][2];\n"
//                 "  float y = sPoint.x*uMVPMatrix[1][0]+sPoint.y*uMVPMatrix[1][1]+uMVPMatrix[1][2];\n"
//                 "  float z = sPoint.x*uMVPMatrix[2][0]+sPoint.y*uMVPMatrix[2][1]+uMVPMatrix[2][2];\n"
//                 "  float X = x/(z*width);\n"
//                 "  float Y = y/(z*height);\n"
//                 "  v_texCoord = vec2(X,Y);\n"
//
//                 "  gl_Position = a_position;\n"
//                 "}\n";
//
// const char gPerspectiveFragmentShader[] =
//         "#extension GL_OES_EGL_image_external : require\n"
//         "precision highp float;\n"
//                 "varying vec2 v_texCoord;\n"
//                 "varying vec2 texCoord;\n"
//                 "uniform sampler2D texture;\n"
//                 "uniform samplerExternalOES u_samplerTexture;\n"
//                 "uniform float layerNum;\n"
//                 "uniform float param1;\n"
//                 "uniform float param2;\n"
//
//                 "void main() {\n"
//                 // "if(layerNum == 2){\n"
//                 // "vec4 tex1 = texture2D(texture,texCoord);\n"
//                 // "vec4 tex2 = texture2D(u_samplerTexture,v_texCoord);\n"
//                 // "if(tex1.r+tex1.g+tex1.b + 60 < tex2.r+tex2.g+tex2.b)\n"
//                 // "gl_FragColor = tex2;} else {\n"
//                 // "gl_FragColor = vec4(texture2D(u_samplerTexture,v_texCoord).rgb, param2);\n"
//                 "gl_FragColor = texture2D(texture,texCoord)*param1;\n"
//                 "gl_FragColor += texture2D(u_samplerTexture,v_texCoord)*param2;\n"
//                 // "}\n"
// /*
// 				"  vec4 colorSrc, colorDst;"
//                 "  if (param1 == 0.0) {\n"
// 				"    gl_FragColor = texture2D(u_samplerTexture,v_texCoord)*param2;\n"
//  				"  } else {\n"
// 				"    colorSrc = texture2D(texture,texCoord)*param1;\n"
// 				"    colorDst  = texture2D(u_samplerTexture,v_texCoord)*param2;\n"
//
//  				"    if ((colorSrc.g / layerNum - colorDst.g) > 0.01) {\n"
// 				"      gl_FragColor = colorSrc + colorSrc / vec4(layerNum, layerNum, layerNum, layerNum);\n"
// 				"    } else {\n"
// 				"      gl_FragColor = colorSrc+colorDst;\n"
// 				"    }\n"
// 				"  }\n"
// */
//
//                 "}\n";

class PerspectiveAdd
{
public:
    PerspectiveAdd();
    ~PerspectiveAdd();

public:
    int initOpenGLES(alloc_device_t *m_alloc_dev, int width, int height);
    int Progress(Mat & _outMat, int* targetAddr, float _iso, int mode);
    bool setMode(int homoMethod = RANSAC);
	int updateImageData(struct cv_fimc_buffer *m_buffers_capture);
    int perspectiveAndAdd(int index, int texIndex, float param1, float param2, const vector <fHomography> & HomographyVec, Mat &dstImage, int* targetAddr, int mode);
	int getResult(int targetAddr);
private:
    float mISO;
	bool mFoundHomography;
	int mCurrentId;
    vector <Mat> m_images;
    vector <Mat> m_grays;
    double work_begin;
    double work_end;
    unsigned char *gData;
    float mWidth;
    float mHeight;
    int HomoMethod;
    bool checkInitOpenGLES;

    GLuint vPositionHandle;
    GLuint vTexCoordHandle;

	GLuint mTextureLocation;
	GLuint mLayerNumLocation;
	GLuint mParam1location;
	GLuint mParam2location;

	GLuint vHomograyHandle;
    GLuint vHomograyHandle1;
    GLuint vHomograyHandle2;
    GLuint vHomograyHandle3;
    GLuint vHomograyHandle4;
    GLuint vHomograyHandle5;
    GLuint vHomograyHandle6;

    GLuint vSizeHandle;

    GLuint programObject;
    // texture
    GLuint textureID1;
    GLuint textureID2;
    GLuint textureID3;
    GLuint textureID4;
    GLuint textureID5;
    GLuint textureID6;

    EGLContext context;
    EGLDisplay display;
	EGLSurface surface;

    GLuint targetTexId;
    GLuint fboTargetHandle;
    EGLImageKHR mTargetEGLImage;
    sp <GraphicBuffer> mTargetGraphicBuffer;
    char* mTargetGraphicBufferAddr;
	buffer_handle_t mTargetBufferHandle;

	class CameraGLTexImage {
	public:
		int mId;
		int mShareFd;
		int mEmptyShareFd;
		EGLDisplay eglDisplay;
		char* mBufferAddr;
		buffer_handle_t mHandle;
	    EGLImageKHR mEGLImage;
	    sp <GraphicBuffer> mGraphicBuffer;

		CameraGLTexImage(int id, EGLDisplay display = NULL, char* bufAddr = NULL, buffer_handle_t handle = NULL, int share_fd = 0);
		~CameraGLTexImage();

		int createTexImage(int width, int height, int format);
		int updateTexImage(int width, int height, int format);
		void clear();
		buffer_handle_t bufferHandleAlloc(uint32_t w, uint32_t h, PixelFormat format, uint32_t usage);
		void bufferHandleFree(buffer_handle_t handle);
	};
	CameraGLTexImage* mCameraGLTexImage[6];// = {NULL};
	GLuint mTextureIds[6];

private:
    int InitEGL();
    int DestroyEGL();
    void checkFBO();
    void workBegin();
    void workEnd(char* module_name = "null");
    GLuint LoadShader( GLenum type, const char *shaderSrc );
    GLuint LoadProgram( const char *vShaderStr, const char *fShaderStr );

    void initializeTmpResEGLImage(alloc_device_t *m_alloc_dev, int fboWidth, int fboHeight, GLuint *tex,
                                  GLuint * fbo, GLuint texGroup);
	int rebindFrameBufferObject(int fboTex, int fboId, long address);
};







#endif //MY_JNI_PERSPECTIVEADD_H
