#include <MutliFrameDenoise.h>
#include "opencv2/opencv.hpp"
#include <stdlib.h>
#include "log.h"

#include <fcntl.h>
#if ANDROID5X
#include <external/skia/include/core/SkImageEncoder.h>
#include <external/skia/include/core/SkBitmap.h>
#include <external/skia/include/core/SkData.h>
#include <external/skia/include/core/SkStream.h>
#endif

MutliFrameDenoise* mMutliFrameDenoise;
struct cv_fimc_buffer* m_buffers_capture;
int width = 1280;
int height = 960;

using namespace cv;
using namespace std;

#if ANDROID5X
int SkSavePng(int width, int height, void* result) {
	int fd = open("/data/local/result.png", O_WRONLY | O_CREAT | O_TRUNC, 0664);
	 if (fd == -1) {
		 LOGE("Error opening file: /data/local/result.png");
		 return -1;
	 }

	 const SkImageInfo info = SkImageInfo::Make(width, height, kRGBA_8888_SkColorType,
															kPremul_SkAlphaType);
	 SkBitmap b;
	 b.installPixels(info, (void*)(result), width*4);
	 SkDynamicMemoryWStream stream;
	 SkImageEncoder::EncodeStream(&stream, b,
			 SkImageEncoder::kPNG_Type, SkImageEncoder::kDefaultQuality);
	 SkData* streamData = stream.copyToData();
	 write(fd, streamData->data(), streamData->size());
	 streamData->unref();
	 close(fd);
}
#endif

int main () {
	char filename[100];
	Mat rgb, yv12;
	char* imagePath = "/data/local/gray/%d.jpg";
	m_buffers_capture = new cv_fimc_buffer();
	mMutliFrameDenoise = new MutliFrameDenoise();

	if (!mMutliFrameDenoise->initialized) {
		LOGD("initializing...");
		sprintf(filename, imagePath, 1);
		rgb = imread(filename);
		width = rgb.cols;
		height = rgb.rows;
		mMutliFrameDenoise->initOpenGLES(NULL, width, height);
	}

	//char result[width*height*4];
	int result;

	LOGD("updating...");
	for (int i=0; i<6; i++) {
		sprintf(filename, imagePath, i);
		rgb = imread(filename);

		cvtColor(rgb, yv12, COLOR_RGB2YUV_YV12);
		// LOGD("YV12_TYPE = %d,CV_8UC1 = %d,",yv12.type(),CV_8UC1);
		m_buffers_capture->start = yv12.data;
		m_buffers_capture->share_fd = 0;
		m_buffers_capture->length = width * height * 3 / 2;
		m_buffers_capture->handle = NULL;

		mMutliFrameDenoise->updateImageData(m_buffers_capture);
	}


	LOGD("processing...");
	mMutliFrameDenoise->processing(&result, OUTPUT_FMT_RGBA);
	LOGD("precess result: %.8x", result);

	mMutliFrameDenoise->getResult(0);
	//SkSavePng(width, height, (void*)result);
	//Mat rgba(height, width, CV_8UC4, (void*)result);
	//imwrite("/data/local/result.jpg", rgba);

	LOGD("destroying...");
	mMutliFrameDenoise->destroy();

	return 0;
}
