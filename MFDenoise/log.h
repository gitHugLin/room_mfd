//
// Created by linqi on 16-1-19.
//

#ifndef MY_JNI_LOG_H
#define MY_JNI_LOG_H


#include <android/log.h>

#ifdef __cplusplus
extern "C" {
#endif

//#ifndef LOG_TAG
#define LOG_TAG    "mfdenoise"
//#endif

#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define LOGF(...)  __android_log_print(ANDROID_LOG_FATAL,LOG_TAG,__VA_ARGS__)

#ifdef __cplusplus
}
#endif


#endif //MY_JNI_LOG_H
