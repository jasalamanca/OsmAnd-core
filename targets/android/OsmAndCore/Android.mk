LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OSMAND_PROTOBUF := $(LOCAL_PATH)/../../../externals/protobuf/upstream.patched
OSMAND_SKIA_ROOT := $(LOCAL_PATH)/../../../externals/skia
OSMAND_SKIA := $(LOCAL_PATH)/../../../externals/skia/upstream.patched
OSMAND_EXPAT := $(LOCAL_PATH)/../../../externals/expat/upstream.patched
OSMAND_CORE_RELATIVE := ../../../native
OSMAND_CORE := $(LOCAL_PATH)/$(OSMAND_CORE_RELATIVE)

LOCAL_C_INCLUDES := $(LOCAL_PATH)/src \
    $(OSMAND_PROTOBUF)/src \
	$(OSMAND_SKIA_ROOT) \
	$(OSMAND_SKIA) \
	$(OSMAND_EXPAT)/lib \
	$(OSMAND_SKIA)/include/core \
	$(OSMAND_SKIA)/include/config \
	$(OSMAND_SKIA)/include/effects \
	$(OSMAND_SKIA)/include/images \
	$(OSMAND_SKIA)/include/ports \
	$(OSMAND_SKIA)/include/utils \
	$(OSMAND_SKIA)/include/utils/android \
	$(OSMAND_SKIA)/src/core \
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
	
ifneq ($(LOCAL_ARM_NEON),true)
	LOCAL_STATIC_LIBRARIES := \
		osmand_protobuf \
		osmand_jpeg \
		osmand_ft2 \
		osmand_png \
		osmand_gif \
		osmand_expat
	LOCAL_WHOLE_STATIC_LIBRARIES := osmand_skia
else
	LOCAL_STATIC_LIBRARIES := \
		osmand_protobuf_neon \
		osmand_jpeg_neon \
		osmand_ft2_neon \
		osmand_png_neon \
		osmand_gif_neon \
		osmand_expat_neon
	LOCAL_WHOLE_STATIC_LIBRARIES := osmand_skia_neon
endif

LOCAL_LDLIBS := -lz -llog -ldl

include $(BUILD_SHARED_LIBRARY)
