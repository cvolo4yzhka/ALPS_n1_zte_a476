LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE = orientationd
LOCAL_MODULE_CLASS = EXECUTABLES
LOCAL_MODULE_OWNER = mtk
LOCAL_PROPRIETARY_MODULE = true
LOCAL_SHARED_LIBRARIES = libc++
LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = arm64/orientationd
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE = orientationd
LOCAL_MODULE_CLASS = EXECUTABLES
LOCAL_MODULE_OWNER = mtk
LOCAL_PROPRIETARY_MODULE = true
LOCAL_SHARED_LIBRARIES = libc++
LOCAL_MULTILIB = 32
LOCAL_SRC_FILES_32 = arm/orientationd
include $(BUILD_PREBUILT)