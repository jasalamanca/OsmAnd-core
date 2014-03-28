project(skia_osmand)

include("../../common.cmake")

set(ROOT "${OSMAND_ROOT}/externals/skia")
set(UPSTREAM "${ROOT}/upstream.patched")

set(target_specific_sources "")
#set(target_specific_dependencies "")
set(target_specific_libraries_public "")
set(target_specific_include_dirs_private "")
set(target_specific_include_dirs_public "")
set(target_specific_definitions_private "")
set(target_specific_definitions_public "")

set(used_arch_specific_sources FALSE)
if(CMAKE_TARGET_CPU_ARCH_FAMILY STREQUAL "x86" OR (CMAKE_TARGET_CPU_ARCH_FAMILY STREQUAL "fat" AND CMAKE_TARGET_OS STREQUAL "ios"))
	set(used_arch_specific_sources TRUE)
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/opts/opts_check_SSE2.cpp"
		"${UPSTREAM}/src/opts/SkBitmapFilter_opts_SSE2.cpp"
		"${UPSTREAM}/src/opts/SkBitmapProcState_opts_SSE2.cpp"
		"${UPSTREAM}/src/opts/SkBitmapProcState_opts_SSSE3.cpp"
		"${UPSTREAM}/src/opts/SkBlitRect_opts_SSE2.cpp"
		"${UPSTREAM}/src/opts/SkBlitRow_opts_SSE2.cpp"
		"${UPSTREAM}/src/opts/SkUtils_opts_SSE2.cpp"
	)
	if(CMAKE_COMPILER_FAMILY STREQUAL "gcc")
		set_source_files_properties(
			"${UPSTREAM}/src/opts/SkBitmapFilter_opts_SSE2.cpp"
			"${UPSTREAM}/src/opts/SkBitmapProcState_opts_SSE2.cpp"
			"${UPSTREAM}/src/opts/SkBlitRect_opts_SSE2.cpp"
			"${UPSTREAM}/src/opts/SkBlitRow_opts_SSE2.cpp"
			"${UPSTREAM}/src/opts/SkUtils_opts_SSE2.cpp"
			PROPERTIES
				COMPILE_FLAGS "-msse2")
		set_source_files_properties(
			"${UPSTREAM}/src/opts/SkBitmapProcState_opts_SSSE3.cpp"
			PROPERTIES
				COMPILE_FLAGS "-mssse3")
	endif()
endif()
if(CMAKE_TARGET_CPU_ARCH_FAMILY STREQUAL "arm" OR (CMAKE_TARGET_CPU_ARCH_FAMILY STREQUAL "fat" AND CMAKE_TARGET_OS STREQUAL "ios"))
	set(used_arch_specific_sources TRUE)
	if(NOT CMAKE_TARGET_OS STREQUAL "ios")
		set(target_specific_definitions_public ${target_specific_definitions_public}
			-D__ARM_HAVE_OPTIONAL_NEON_SUPPORT
		)
	endif()
	enable_language(ASM)
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/opts/opts_check_arm.cpp"
		"${UPSTREAM}/src/opts/SkBlitRow_opts_arm.cpp"
		"${UPSTREAM}/src/opts/SkBlitRow_opts_arm_neon.cpp"
		"${UPSTREAM}/src/opts/SkBlitMask_opts_arm.cpp"
		"${UPSTREAM}/src/opts/SkBitmapProcState_opts_arm.cpp"
		"${UPSTREAM}/src/opts/SkBitmapProcState_matrixProcs_neon.cpp"
		"${UPSTREAM}/src/opts/SkBitmapProcState_arm_neon.cpp"
		"${UPSTREAM}/src/opts/memset32_neon.S"
		"${UPSTREAM}/src/opts/memset16_neon.S"
		"${UPSTREAM}/src/opts/memset.arm.S"
	)
	if(CMAKE_TARGET_OS STREQUAL "ios" AND CMAKE_COMPILER_FAMILY STREQUAL "clang")
		set_source_files_properties(
			"${UPSTREAM}/src/opts/memset32_neon.S"
			"${UPSTREAM}/src/opts/memset16_neon.S"
			"${UPSTREAM}/src/opts/memset.arm.S"
			PROPERTIES
				COMPILE_FLAGS "-no-integrated-as")
	endif()
endif()
if(NOT used_arch_specific_sources)
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/opts/SkBlitRow_opts_none.cpp"
		"${UPSTREAM}/src/opts/SkBlitMask_opts_none.cpp"
		"${UPSTREAM}/src/opts/SkBitmapProcState_opts_none.cpp"
		"${UPSTREAM}/src/opts/SkUtils_opts_none.cpp"
	)
endif()

if(CMAKE_TARGET_OS STREQUAL "windows")
	set(target_specific_include_dirs_private ${target_specific_include_dirs_private}
		"${UPSTREAM}/src/utils/win"
	)
	set(target_specific_include_dirs_public ${target_specific_include_dirs_public}
		"${UPSTREAM}/include/utils/win"
	)
	file(GLOB src_utils_win "${UPSTREAM}/src/utils/win/*")
#	file(GLOB src_gpu_gl_win "${UPSTREAM}/src/gpu/gl/win/*")
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/ports/SkDebug_win.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_win_dw.cpp"
		"${UPSTREAM}/src/ports/SkFontMgr_default_dw.cpp"
		"${UPSTREAM}/src/ports/SkTime_win.cpp"
		"${UPSTREAM}/src/ports/SkThread_win.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_win.cpp"
		"${UPSTREAM}/src/ports/SkTLS_win.cpp"
		${src_utils_win}
#		${src_gpu_gl_win}
	)
	set(target_specific_definitions_public ${target_specific_definitions_public}
		-DSK_BUILD_FOR_WIN32
		-DSK_FONTHOST_USES_FONTMGR
	)
	set(target_specific_libraries_public ${target_specific_libraries_public}
		Usp10
	)
elseif(CMAKE_TARGET_OS STREQUAL "darwin")
	set(target_specific_include_dirs_public ${target_specific_include_dirs_public}
		"${UPSTREAM}/include/utils/mac"
	)
	file(GLOB src_utils_mac "${UPSTREAM}/src/utils/mac/*")
#	file(GLOB src_gpu_gl_mac "${UPSTREAM}/src/gpu/gl/mac/*")
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/ports/SkDebug_stdio.cpp"
		"${UPSTREAM}/src/ports/SkThread_pthread.cpp"
		"${UPSTREAM}/src/ports/SkTime_Unix.cpp"
		"${UPSTREAM}/src/ports/SkImageDecoder_CG.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_mac.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_posix.cpp"
		"${UPSTREAM}/src/ports/SkTLS_pthread.cpp"
		${src_utils_mac}
#		${src_gpu_gl_mac}
	)
	set(target_specific_definitions_public ${target_specific_definitions_public}
		-DSK_BUILD_FOR_MAC
		-DSK_FONTHOST_USES_FONTMGR
		-D_CRT_SECURE_NO_WARNINGS
		-DGR_GL_FUNCTION_TYPE=__stdcall
	)
	find_library(CORE_FOUNDATION_LIBRARY CoreFoundation)
	find_library(CORE_TEXT_LIBRARY CoreText)
	find_library(CORE_GRAPHICS_LIBRARY CoreGraphics)
	set(target_specific_libraries_public ${target_specific_libraries_public}
		${CORE_FOUNDATION_LIBRARY}
		${CORE_TEXT_LIBRARY}
		${CORE_GRAPHICS_LIBRARY}
	)
elseif(CMAKE_TARGET_OS STREQUAL "ios")
	set(target_specific_include_dirs_public ${target_specific_include_dirs_public}
		"${UPSTREAM}/include/utils/mac"
		"${UPSTREAM}/include/utils/ios"
	)
#	file(GLOB src_gpu_gl_ios "${UPSTREAM}/src/gpu/gl/iOS/*")
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/ports/SkDebug_stdio.cpp"
		"${UPSTREAM}/src/ports/SkThread_pthread.cpp"
		"${UPSTREAM}/src/ports/SkTime_Unix.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_posix.cpp"
		"${UPSTREAM}/src/ports/SkTLS_pthread.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_mac.cpp"
		"${UPSTREAM}/src/utils/mac/SkStream_mac.cpp"
#		${src_gpu_gl_ios}
	)
	set(target_specific_definitions_public ${target_specific_definitions_public}
		-DSK_BUILD_FOR_IOS
	)
	find_library(CORE_FOUNDATION_LIBRARY CoreFoundation)
	find_library(CORE_TEXT_LIBRARY CoreText)
	find_library(CORE_GRAPHICS_LIBRARY CoreGraphics)
	set(target_specific_libraries_public ${target_specific_libraries_public}
		${CORE_FOUNDATION_LIBRARY}
		${CORE_TEXT_LIBRARY}
		${CORE_GRAPHICS_LIBRARY}
	)
elseif(CMAKE_TARGET_OS STREQUAL "linux")
#	file(GLOB src_gpu_gl_unix "${UPSTREAM}/src/gpu/gl/unix/*")
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/ports/SkDebug_stdio.cpp"
		"${UPSTREAM}/src/ports/SkThread_pthread.cpp"
		"${UPSTREAM}/src/ports/SkTime_Unix.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_FreeType_common.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_FreeType.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_linux.cpp"
		"${UPSTREAM}/src/utils/SkThreadUtils_pthread.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_posix.cpp"
		"${UPSTREAM}/src/ports/SkTLS_pthread.cpp"
#		${src_gpu_gl_unix}
	)
	set(target_specific_definitions_public ${target_specific_definitions_public}
		-DSK_BUILD_FOR_UNIX
	)
#	set(target_specific_dependencies ${target_specific_dependencies}
#		freetype2_osmand
#	)
	set(target_specific_libraries_public ${target_specific_libraries_public}
		freetype2_osmand
		pthread
	)
endif()

file(GLOB src_core "${UPSTREAM}/src/core/*")
file(GLOB src_effects "${UPSTREAM}/src/effects/*")
file(GLOB src_effects_gradients "${UPSTREAM}/src/effects/gradients/*")
file(GLOB src_image "${UPSTREAM}/src/image/*")
file(GLOB src_images "${UPSTREAM}/src/images/*")
file(GLOB src_images__remove
	"${UPSTREAM}/src/images/SkImageDecoder_libwebp.*"
	"${UPSTREAM}/src/images/SkImageRef_ashmem.*")
list(REMOVE_ITEM src_images ${src_images__remove})
file(GLOB src_sfnt "${UPSTREAM}/src/sfnt/*")
file(GLOB src_pathops "${UPSTREAM}/src/pathops/*")
file(GLOB src_pathops__remove
	"${UPSTREAM}/src/pathops/main.cpp")
list(REMOVE_ITEM src_pathops ${src_pathops__remove})
#file(GLOB src_gpu "${UPSTREAM}/src/gpu/*")
#file(GLOB src_gpu__remove
#	"${UPSTREAM}/src/gpu/GrAddPathRenderers*"
#	"${UPSTREAM}/src/gpu/gr_unittests.cpp"
#	"${UPSTREAM}/src/gpu/FlingState.cpp"
#	"${UPSTREAM}/src/gpu/GrRectanizer_fifo.cpp")
#list(REMOVE_ITEM src_gpu ${src_gpu__remove})
#file(GLOB src_gpu_effects "${UPSTREAM}/src/gpu/effects/*")
#file(GLOB src_gpu_gl "${UPSTREAM}/src/gpu/gl/*")
#file(GLOB src_gpu_gl__remove
#	"${UPSTREAM}/src/gpu/gl/GrGLCreateNativeInterface_none.cpp"
#	"${UPSTREAM}/src/gpu/gl/GrGLDefaultInterface_none.cpp")
#list(REMOVE_ITEM src_gpu_gl ${src_gpu_gl__remove})

add_library(skia_osmand STATIC
	${src_core}
	${src_effects}
	${src_effects_gradients}
	${src_image}
	${src_images}
	${src_sfnt}
	${src_pathops}
#	${src_gpu}
#	"${UPSTREAM}/src/gpu/GrAddPathRenderers_default.cpp"
#	${src_gpu_effects}
#	${src_gpu_gl}
	"${UPSTREAM}/src/utils/SkBase64.cpp"
	"${UPSTREAM}/src/utils/SkBitSet.cpp"
	"${UPSTREAM}/src/utils/SkBoundaryPatch.cpp"
	"${UPSTREAM}/src/utils/SkCamera.cpp"
	"${UPSTREAM}/src/utils/SkCondVar.cpp"
	"${UPSTREAM}/src/utils/SkCountdown.cpp"
	"${UPSTREAM}/src/utils/SkCubicInterval.cpp"
	"${UPSTREAM}/src/utils/SkCullPoints.cpp"
	"${UPSTREAM}/src/utils/SkDeferredCanvas.cpp"
	"${UPSTREAM}/src/utils/SkDumpCanvas.cpp"
	"${UPSTREAM}/src/utils/SkInterpolator.cpp"
	"${UPSTREAM}/src/utils/SkJSON.cpp"
	"${UPSTREAM}/src/utils/SkLayer.cpp"
	"${UPSTREAM}/src/utils/SkMatrix44.cpp"
	"${UPSTREAM}/src/utils/SkMeshUtils.cpp"
	"${UPSTREAM}/src/utils/SkNinePatch.cpp"
	"${UPSTREAM}/src/utils/SkNullCanvas.cpp"
	"${UPSTREAM}/src/utils/SkNWayCanvas.cpp"
	"${UPSTREAM}/src/utils/SkOSFile.cpp"
	"${UPSTREAM}/src/utils/SkParse.cpp"
	"${UPSTREAM}/src/utils/SkParseColor.cpp"
	"${UPSTREAM}/src/utils/SkParsePath.cpp"
	"${UPSTREAM}/src/utils/SkPictureUtils.cpp"
	"${UPSTREAM}/src/utils/SkProxyCanvas.cpp"
	"${UPSTREAM}/src/utils/SkThreadPool.cpp"
	"${UPSTREAM}/src/utils/SkUnitMappers.cpp"
	"${UPSTREAM}/src/pipe/SkGPipeRead.cpp"
	"${UPSTREAM}/src/pipe/SkGPipeWrite.cpp"
	"${UPSTREAM}/src/ports/SkGlobalInitialization_default.cpp"
	"${UPSTREAM}/src/ports/SkOSFile_stdio.cpp"
	${target_specific_sources}
)

add_dependencies(skia_osmand
	expat_osmand
	png_osmand
	gif_osmand
	jpeg_osmand
#	${target_specific_dependencies}
)

add_definitions(-DSK_RELEASE -DGR_RELEASE=1 -DNDEBUG -DSK_SUPPORT_GPU=0)
add_definitions(
		${target_specific_definitions_public}
)

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-rtti -fno-exceptions")
endif()

include_directories(AFTER SYSTEM
#	PRIVATE
		"${UPSTREAM}/src/core"
		"${UPSTREAM}/src/config"
		"${UPSTREAM}/src/effects"
		"${UPSTREAM}/src/image"
		"${UPSTREAM}/src/images"
		"${UPSTREAM}/src/ports"
		"${UPSTREAM}/src/utils"
		"${UPSTREAM}/src/xml"
		"${UPSTREAM}/src/sfnt"
#		"${UPSTREAM}/src/gpu"
		${target_specific_include_dirs_private}
#	PUBLIC
		"${OSMAND_ROOT}/externals/freetype/upstream.patched/include"
		"${OSMAND_ROOT}/externals/giflib/upstream.patched/lib"
		"${OSMAND_ROOT}/externals/jpeg/upstream.patched"
		"${OSMAND_ROOT}/externals/libpng/upstream.patched"
		"${UPSTREAM}/include/core"
		"${UPSTREAM}/include/lazy"
		"${UPSTREAM}/include/pathops"
		"${UPSTREAM}/src/core"
		"${UPSTREAM}/include/config"
		"${UPSTREAM}/include/effects"
		"${UPSTREAM}/include/images"
		"${UPSTREAM}/include/pipe"
		"${UPSTREAM}/include/ports"
		"${UPSTREAM}/include/utils"
		"${UPSTREAM}/include/xml"
		"${UPSTREAM}/include/gpu"
		${target_specific_include_dirs_public}
)
target_link_libraries(skia_osmand
	LINK_PUBLIC
		expat_osmand
		png_osmand
		gif_osmand
		jpeg_osmand
		${target_specific_libraries_public}
)

if(CMAKE_TARGET_CPU_ARCH_FAMILY STREQUAL "fat" AND CMAKE_TARGET_OS STREQUAL "ios")
	set(ios_iphoneos_exclude_list
		"opts_check_SSE2.cpp"
		"SkBitmapFilter_opts_SSE2.cpp"
		"SkBitmapProcState_opts_SSE2.cpp"
		"SkBitmapProcState_opts_SSSE3.cpp"
		"SkBlitRect_opts_SSE2.cpp"
		"SkBlitRow_opts_SSE2.cpp"
		"SkUtils_opts_SSE2.cpp")
	string(REPLACE ";" " " ios_iphoneos_exclude "${ios_iphoneos_exclude_list}")
	set_target_properties(skia_osmand
		PROPERTIES
			XCODE_ATTRIBUTE_EXCLUDED_SOURCE_FILE_NAMES[sdk=iphoneos*][arch=*] "${ios_iphoneos_exclude}")

	set(ios_iphonesimulator_exclude_list
		"opts_check_arm.cpp"
		"SkBlitRow_opts_arm.cpp"
		"SkBlitRow_opts_arm_neon.cpp"
		"SkBlitMask_opts_arm.cpp"
		"SkBitmapProcState_opts_arm.cpp"
		"SkBitmapProcState_matrixProcs_neon.cpp"
		"SkBitmapProcState_arm_neon.cpp"
		"memset32_neon.S"
		"memset16_neon.S"
		"memset.arm.S")
	string(REPLACE ";" " " ios_iphonesimulator_exclude "${ios_iphonesimulator_exclude_list}")
	set_target_properties(skia_static
		PROPERTIES
			XCODE_ATTRIBUTE_EXCLUDED_SOURCE_FILE_NAMES[sdk=iphonesimulator*][arch=*] "${ios_iphonesimulator_exclude}")
endif()
