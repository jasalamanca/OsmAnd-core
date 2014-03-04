LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ifneq ($(OSMAND_BUILDING_NEON_LIBRARY),true)
    LOCAL_MODULE := OsmAndCoreJNI
else
    LOCAL_MODULE := OsmAndCoreJNI_neon
    LOCAL_ARM_NEON := true
endif

ifeq ($(LOCAL_ARM_NEON),true)
    OSMAND_BINARY_SUFFIX := _neon
else
    OSMAND_BINARY_SUFFIX :=
endif

LOCAL_STATIC_LIBRARIES := OsmAndCore$(OSMAND_BINARY_SUFFIX)

ifneq ($(OSMAND_USE_PREBUILT),true)
    LOCAL_SRC_FILES := gen/cpp/swig.cpp

    $(info $(shell $(LOCAL_PATH)/generate.sh))
    include $(BUILD_SHARED_LIBRARY)
else
    LOCAL_SRC_FILES := \
        $(OSMAND_ANDROID_PREBUILT_ROOT)/$(TARGET_ARCH_ABI)/lib$(LOCAL_MODULE).so
    include $(PREBUILT_SHARED_LIBRARY)
endif

include $(LOCAL_PATH)/../../Android.mk