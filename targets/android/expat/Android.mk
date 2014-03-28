LOCAL_PATH := $(call my-dir)
LOCAL_PATH := $(LOCAL_PATH)/../../../externals/expat

include $(CLEAR_VARS)

ifneq ($(OSMAND_BUILDING_NEON_LIBRARY),true)
    LOCAL_MODULE := osmand_expat
else
    LOCAL_MODULE := osmand_expat_neon
    LOCAL_ARM_NEON := true
endif

LOCAL_SRC_PATH := \
    upstream.patched/lib

LOCAL_EXPORT_C_INCLUDES := \
    $(LOCAL_PATH)/upstream.patched/lib

LOCAL_CFLAGS := \
    $(LOCAL_EXPORT_CFLAGS) \
    -DHAVE_EXPAT_CONFIG_H

LOCAL_C_INCLUDES := \
    $(LOCAL_EXPORT_C_INCLUDES) \
    $(LOCAL_PATH)

LOCAL_SRC_FILES := \
    $(LOCAL_SRC_PATH)/xmlparse.c \
    $(LOCAL_SRC_PATH)/xmlrole.c \
    $(LOCAL_SRC_PATH)/xmltok.c \
    $(LOCAL_SRC_PATH)/xmltok_impl.c \
    $(LOCAL_SRC_PATH)/xmltok_ns.c

include $(BUILD_STATIC_LIBRARY)
