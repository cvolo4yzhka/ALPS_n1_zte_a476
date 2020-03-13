LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE = sink
LOCAL_MODULE_CLASS = EXECUTABLES
LOCAL_MODULE_OWNER = mtk
LOCAL_PROPRIETARY_MODULE = true
LOCAL_MODULE_TAGS = debug
LOCAL_SHARED_LIBRARIES = libbinder libgui libmedia libstagefright libstagefright_foundation libui libsink libc++
LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = arm64/sink
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE = sink
LOCAL_MODULE_CLASS = EXECUTABLES
LOCAL_MODULE_OWNER = mtk
LOCAL_PROPRIETARY_MODULE = true
LOCAL_MODULE_TAGS = debug
LOCAL_SHARED_LIBRARIES = libbinder libgui libmedia libstagefright libstagefright_foundation libui libsink libc++
LOCAL_MULTILIB = 32
LOCAL_SRC_FILES_32 = arm/sink
include $(BUILD_PREBUILT)
