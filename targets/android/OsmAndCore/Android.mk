LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OSMAND_CORE_RELATIVE := ../../../native
OSMAND_CORE := $(LOCAL_PATH)/$(OSMAND_CORE_RELATIVE)

LOCAL_C_INCLUDES := $(LOCAL_PATH)/src \
	$(OSMAND_CORE)/include \
	$(OSMAND_CORE)/src

LOCAL_CPP_EXTENSION := .cc .cpp
LOCAL_SRC_FILES := \
	src/Logging.cpp \
	$(OSMAND_CORE_RELATIVE)/src/ElapsedTimer.cpp \
	$(OSMAND_CORE_RELATIVE)/src/common.cpp \
	$(OSMAND_CORE_RELATIVE)/src/mapObjects.cpp \
	$(OSMAND_CORE_RELATIVE)/src/multipolygons.cpp \
	$(OSMAND_CORE_RELATIVE)/src/renderRules.cpp \
	$(OSMAND_CORE_RELATIVE)/src/rendering.cpp \
	$(OSMAND_CORE_RELATIVE)/src/binaryRead.cpp \
        $(OSMAND_CORE_RELATIVE)/src/generalRouter.cpp \
	$(OSMAND_CORE_RELATIVE)/src/binaryRoutePlanner.cpp \
	$(OSMAND_CORE_RELATIVE)/src/proto/osmand_index.pb.cc \
	$(OSMAND_CORE_RELATIVE)/src/java_wrap.cpp
	
ifdef OSMAND_PROFILE_NATIVE_OPERATIONS
	LOCAL_CFLAGS += \
		-DOSMAND_NATIVE_PROFILING
endif

# Name of the local module
ifneq ($(OSMAND_BUILDING_NEON_LIBRARY),true)
	LOCAL_MODULE := osmand
else
	LOCAL_MODULE := osmand_neon
	LOCAL_ARM_NEON := true
endif

LOCAL_CFLAGS := \
	-DGOOGLE_PROTOBUF_NO_RTTI \
	-DSK_BUILD_FOR_ANDROID \
	-DSK_BUILD_FOR_ANDROID_NDK \
	-DSK_ALLOW_STATIC_GLOBAL_INITIALIZERS=0 \
	-DSK_RELEASE \
	-DSK_CPU_LENDIAN \
	-DGR_RELEASE=1 \
	-DANDROID_BUILD \
	-fPIC

ifeq ($(LOCAL_ARM_NEON),true)
    OSMAND_BINARY_SUFFIX := _neon
else
    OSMAND_BINARY_SUFFIX :=
endif

LOCAL_STATIC_LIBRARIES := \
    osmand_protobuf$(OSMAND_BINARY_SUFFIX) \
    osmand_skia$(OSMAND_BINARY_SUFFIX) \
#		osmand_jpeg \
#		osmand_ft2 \
#		osmand_png \
#		osmand_gif \
#		osmand_expat

#LOCAL_WHOLE_STATIC_LIBRARIES := osmand_skia$(OSMAND_BINARY_SUFFIX)

LOCAL_LDLIBS := -lz -llog -ldl

include $(BUILD_SHARED_LIBRARY)
