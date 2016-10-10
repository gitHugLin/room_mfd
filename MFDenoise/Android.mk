LOCAL_PATH := $(call my-dir)

# include $(CLEAR_VARS)
# LOCAL_MODULE := libopencv_java3
# LOCAL_MODULE_TAGS := optional
# LOCAL_MODULE_CLASS := SHARED_LIBRARIES
# LOCAL_MODULE_STEM := $(LOCAL_MODULE)
# LOCAL_MODULE_SUFFIX := .so
# ifneq ($(strip $(TARGET_2ND_ARCH)), )
# LOCAL_MULTILIB := both
# LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
# LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
# else
# LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
# endif
# include $(BUILD_PREBUILT)
#
# include $(CLEAR_VARS)
# LOCAL_MODULE := opencv_java3
# LOCAL_MODULE_TAGS := optional
# LOCAL_MODULE_CLASS := SHARED_LIBRARIES
# LOCAL_MODULE_STEM := $(LOCAL_MODULE)
# LOCAL_MODULE_SUFFIX := .so
# ifneq ($(strip $(TARGET_2ND_ARCH)), )
# LOCAL_MULTILIB := both
# LOCAL_SRC_FILES_$(TARGET_ARCH) := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
# LOCAL_SRC_FILES_$(TARGET_2ND_ARCH) := lib/$(TARGET_2ND_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
# else
# LOCAL_SRC_FILES := lib/$(TARGET_ARCH)/$(LOCAL_MODULE)$(LOCAL_MODULE_SUFFIX)
# endif
# include $(BUILD_PREBUILT)


include $(CLEAR_VARS)

//OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on
# OPENCV_LIB_TYPE := SHARED -lm -ljnigraphics -lui
# include $(LOCAL_PATH)/./sdk/native/jni/OpenCV.mk
include /Users/linqi/SDKDir/OpenCV-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_SDK_VERSION := 14
LOCAL_NDK_STL_VARIANT := gnustl_static

LOCAL_MODULE := libMFDenoise
LOCAL_LDFLAGS := -Wl,--build-id -Wl,--no-fatal-warnings
LOCAL_LDLIBS :=  -llog -landroid -lGLESv2 -lEGL -lcutils -lui -lutils -lgui
LOCAL_CFLAGS :=  -DEGL_EGLEXT_PROTOTYPES -DGL_GLEXT_PROTOTYPES -DROCKCHIP_GPU_LIB_ENABLE -DHAVE_PTHREADS -DHAVE_SYS_UIO_H
#-DANDROID5X
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/./sdk/native/jni/include \
    frameworks/native/include \
    hardware/libhardware/include \
    system/core/include \
	hardware/rk29/libgralloc_ump \
	bionic/libc/kernel/common

LOCAL_SRC_FILES := \
	MutliFrameDenoise.cpp \
	src/format.cpp \
	src/MyThread.cpp \
	src/OrbPatch.cpp \
	src/MutGetHomography.cpp \
	src/GetHomography.cpp \
	src/PerspectiveAdd.cpp\
	src/MySemophore.cpp\
	src/MyAlgorithm.cpp\
	src/MyFeature2D.cpp\
	src/MyORB.cpp\
	src/utils.cpp\
	src/MyPointSetRegistrator.cpp\
	src/MyRho.cpp

LOCAL_C_INCLUDE := $(LOCAL_PATH)/./sdk/native/jni/include
LOCAL_C_INCLUDES += include
LOCAL_C_INCLUDES += ./
LOCAL_SHARED_LIBRARIES :=  libopencv_java3 libGLESv2 libEGL libcutils libui libutils libgui libm
include $(BUILD_SHARED_LIBRARY)

#-------------------------------------------------
# include $(CLEAR_VARS)
# #//OPENCV_CAMERA_MODULES:=on
# OPENCV_INSTALL_MODULES:=on
# #//OPENCV_LIB_TYPE := SHARED -lm -ljnigraphics -lui
# # include $(LOCAL_PATH)/./sdk/native/jni/OpenCV.mk
# include /Users/linqi/SDKDir/OpenCV-android-sdk/sdk/native/jni/OpenCV.mk
#
# LOCAL_SDK_VERSION := 14
# LOCAL_NDK_STL_VARIANT := gnustl_static
#
# LOCAL_LDFLAGS := -Wl,--build-id -Wl,--no-fatal-warnings
# LOCAL_CFLAGS :=  -DSK_SUPPORT_LEGACY_SETCONFIG
#
# LOCAL_C_INCLUDES += $(LOCAL_PATH)
# LOCAL_C_INCLUDES += \
#    $(LOCAL_PATH)/./sdk/native/jni/include \
#    frameworks/native/include \
#    hardware/libhardware/include \
#    system/core/include \
#    hardware/rk29/libgralloc_ump \
#    bionic/libc/kernel/common
#
# LOCAL_C_INCLUDES += \
#    external/skia/include/core \
#    external/skia/include/effects \
#    external/skia/include/images \
#    external/skia/src/ports \
#    external/skia/include/utils
#
# LOCAL_SRC_FILES := \
# 	main.cpp
#
# LOCAL_MODULE:= mfdenoise
#
# LOCAL_SHARED_LIBRARIES := libMFDenoise libopencv_java3 libskia libcutils libutils libgui libui libm
# include $(BUILD_EXECUTABLE)
