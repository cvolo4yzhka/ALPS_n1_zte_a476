LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE = md_monitor_ctrl
LOCAL_MODULE_CLASS = EXECUTABLES
LOCAL_MODULE_OWNER = mtk
LOCAL_PROPRIETARY_MODULE = true
LOCAL_MODULE_TAGS = optional
LOCAL_SHARED_LIBRARIES = libselinux libccci_util libmdloggerrecycle libc++
LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = arm64/md_monitor_ctrl
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE = md_monitor_ctrl
LOCAL_MODULE_CLASS = EXECUTABLES
LOCAL_MODULE_OWNER = mtk
LOCAL_PROPRIETARY_MODULE = true
LOCAL_MODULE_TAGS = optional
LOCAL_SHARED_LIBRARIES = libselinux libccci_util libmdloggerrecycle libc++
LOCAL_MULTILIB = 32
LOCAL_SRC_FILES_32 = arm/md_monitor_ctrl
include $(BUILD_PREBUILT)
