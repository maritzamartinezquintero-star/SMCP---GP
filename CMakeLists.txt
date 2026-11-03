cmake_minimum_required(VERSION 3.28)

project(SMCP
    VERSION 3.0.2
    DESCRIPTION "Structured-light 3D scanner with projector-camera calibration"
    LANGUAGES CXX)

# With a localized cl.exe, Ninja would otherwise record no header dependencies.
include("${CMAKE_SOURCE_DIR}/cmake/MsvcShowIncludesPrefix.cmake")

include(CTest)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(SMCP_WITH_SPINNAKER "Build capture support for FLIR/Teledyne cameras (Spinnaker SDK)" ON)
option(SMCP_DEPLOY_RUNTIME "Copy the Qt, OpenCV, and Spinnaker DLLs next to the executable after building" ON)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Every configuration (Debug or Release, Ninja or Visual Studio) outputs to build/<preset>/bin.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(_cfg Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${_cfg}" _CFG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/bin")
endforeach()

if(NOT MSVC)
    message(FATAL_ERROR "SMCP can only be built with MSVC (Visual Studio 2022/2026, x64 toolset).")
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# Qt 5: Qt5_DIR may point to the kit root (.../msvc2017_64) or to lib/cmake/Qt5.
if(DEFINED Qt5_DIR AND NOT EXISTS "${Qt5_DIR}/Qt5Config.cmake"
        AND EXISTS "${Qt5_DIR}/lib/cmake/Qt5/Qt5Config.cmake")
    set(Qt5_DIR "${Qt5_DIR}/lib/cmake/Qt5" CACHE PATH "lib/cmake/Qt5 directory of the Qt kit" FORCE)
endif()
find_package(Qt5 5.14 REQUIRED COMPONENTS Core Gui Widgets OpenGL Concurrent)
get_target_property(_qmake_location Qt5::qmake IMPORTED_LOCATION)
get_filename_component(SMCP_QT_BIN_DIR "${_qmake_location}" DIRECTORY)
message(STATUS "Qt ${Qt5_VERSION}: ${SMCP_QT_BIN_DIR}")

# OpenCV 2.4: its OpenCVConfig.cmake does not recognize toolsets newer than vc15,
# so the x64/vc14 binaries are selected (compatible with v141..v145).
if(NOT DEFINED OpenCV_ARCH)
    set(OpenCV_ARCH x64)
endif()
if(NOT DEFINED OpenCV_RUNTIME)
    set(OpenCV_RUNTIME vc14)
endif()
find_package(OpenCV 2.4 REQUIRED COMPONENTS core imgproc highgui calib3d features2d flann)
message(STATUS "OpenCV ${OpenCV_VERSION}: ${OpenCV_LIB_PATH}")

# Spinnaker (optional).
if(SMCP_WITH_SPINNAKER)
    find_package(Spinnaker REQUIRED)
    message(STATUS "Spinnaker: ${Spinnaker_LIBRARY_RELEASE}")
endif()

find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------------------
# Sources (explicit lists per subdirectory, no GLOB)
# ---------------------------------------------------------------------------
set(SMCP_SOURCES_APP
    src/app/main.cpp
    src/app/Application.cpp
    src/app/Application.h
    src/app/MainWindow.cpp
    src/app/MainWindow.h
    src/app/MainWindow.ui
)

set(SMCP_SOURCES_UI
    src/ui/AboutDialog.cpp
    src/ui/AboutDialog.h
    src/ui/AboutDialog.ui
    src/ui/CalibrationDialog.cpp
    src/ui/CalibrationDialog.h
    src/ui/CalibrationDialog.ui
    src/ui/models/TreeModel.cpp
    src/ui/models/TreeModel.h
    src/ui/pointcloud_editor/AiInferenceProgressDialog.cpp
    src/ui/pointcloud_editor/AiInferenceProgressDialog.h
    src/ui/pointcloud_editor/AiInferenceProgressDialog.ui
    src/ui/pointcloud_editor/AiModelConfigDialog.cpp
    src/ui/pointcloud_editor/AiModelConfigDialog.h
    src/ui/pointcloud_editor/ExportPointcloudDialog.cpp
    src/ui/pointcloud_editor/ExportPointcloudDialog.h
    src/ui/pointcloud_editor/ExportPointcloudDialog.ui
    src/ui/pointcloud_editor/PointcloudEditorDialog.cpp
    src/ui/pointcloud_editor/PointcloudEditorDialog.h
    src/ui/pointcloud_editor/PointcloudEditorDialog.ui
    src/ui/pointcloud_editor/PlaneFitDialog.cpp
    src/ui/pointcloud_editor/PlaneFitDialog.h
    src/ui/pointcloud_editor/PlaneFitDialog.ui
    src/ui/pointcloud_editor/RemoveOutliersDialog.cpp
    src/ui/pointcloud_editor/RemoveOutliersDialog.h
    src/ui/pointcloud_editor/RemoveOutliersDialog.ui
    src/ui/pointcloud_editor/SphereFitDialog.cpp
    src/ui/pointcloud_editor/SphereFitDialog.h
    src/ui/pointcloud_editor/SphereFitDialog.ui
    src/ui/preview/ImageLabel.cpp
    src/ui/preview/ImageLabel.h
    src/ui/preview/PixmapWidget.cpp
    src/ui/preview/PixmapWidget.h
    src/ui/preview/PointcloudPreviewWidget.cpp
    src/ui/preview/PointcloudPreviewWidget.h
    src/ui/preview/PointcloudWidget.cpp
    src/ui/preview/PointcloudWidget.h
    src/ui/ProcessingDialog.cpp
    src/ui/ProcessingDialog.h
    src/ui/ProcessingDialog.ui
    src/ui/widgets/ProjectorWidget.cpp
    src/ui/widgets/ProjectorWidget.h
)

# The capture dialog requires Spinnaker.
set(SMCP_SOURCES_UI_CAPTURE
    src/ui/CaptureDialog.cpp
    src/ui/CaptureDialog.h
    src/ui/CaptureDialog.ui
)

set(SMCP_SOURCES_CORE
    src/core/BusyCursorGuard.h
    src/core/CalibrationData.cpp
    src/core/CalibrationData.h
    src/core/GPCompensation.cpp
    src/core/GPCompensation.h
    src/core/IoUtil.cpp
    src/core/IoUtil.h
    src/core/PointcloudOps.cpp
    src/core/PointcloudOps.h
    src/core/Scan3d.cpp
    src/core/Scan3d.h
    src/core/StructuredLight.cpp
    src/core/StructuredLight.h
)

set(SMCP_SOURCES_AI
    src/ai/WslModelService.cpp
    src/ai/WslModelService.h
)

set(SMCP_SOURCES_CAMERA
    src/camera/CameraSettings.h
    src/camera/CameraUtilities.cpp
    src/camera/CameraUtilities.h
    src/camera/CameraWorker.cpp
    src/camera/CameraWorker.h
)

set(SMCP_SOURCES_EXPORT
    src/export/IOExport.cpp
    src/export/IOExport.h
)

set(SMCP_SOURCES_COMMON
    src/common/Literals.h
    src/common/Settings.h
    src/common/CvMatConvert.cpp
    src/common/CvMatConvert.h
)

set(SMCP_RESOURCES
    resources/resources.qrc
)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(SMCP
    ${SMCP_SOURCES_APP}
    ${SMCP_SOURCES_UI}
    ${SMCP_SOURCES_CORE}
    ${SMCP_SOURCES_AI}
    ${SMCP_SOURCES_EXPORT}
    ${SMCP_SOURCES_COMMON}
    ${SMCP_RESOURCES}
)

if(SMCP_WITH_SPINNAKER)
    target_sources(SMCP PRIVATE ${SMCP_SOURCES_UI_CAPTURE} ${SMCP_SOURCES_CAMERA})
endif()

# Folder view in Visual Studio and other IDEs.
source_group(TREE "${CMAKE_SOURCE_DIR}" FILES
    ${SMCP_SOURCES_APP} ${SMCP_SOURCES_UI} ${SMCP_SOURCES_UI_CAPTURE} ${SMCP_SOURCES_CORE} ${SMCP_SOURCES_AI}
    ${SMCP_SOURCES_CAMERA} ${SMCP_SOURCES_EXPORT} ${SMCP_SOURCES_COMMON} ${SMCP_RESOURCES})

set_target_properties(SMCP PROPERTIES
    OUTPUT_NAME SMCP
    DEBUG_POSTFIX _d
    # Release has no console (WinMain through Qt5::WinMain); Debug keeps the console.
    WIN32_EXECUTABLE $<NOT:$<CONFIG:Debug>>
)

# A single include directory: includes are qualified with their subdirectory
# ("app/Application.h", "core/Scan3d.h", ...). Each .ui form lives next to its class.
target_include_directories(SMCP PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_compile_definitions(SMCP PRIVATE
    NOMINMAX
    _CRT_SECURE_NO_WARNINGS
    _SCL_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    _ENABLE_EXTENDED_ALIGNED_STORAGE
    UNICODE
    _UNICODE
    # opencv2/flann (2.4) uses std::random_shuffle, removed in C++17; the MSVC STL
    # keeps it when this macro is defined.
    _HAS_AUTO_PTR_ETC=1
    $<$<BOOL:${SMCP_WITH_SPINNAKER}>:USE_SPINNAKER>
)

target_compile_options(SMCP PRIVATE
    /W3
    /MP
    /utf-8
    /external:W0   # no warnings from Qt, OpenCV, and Spinnaker headers
)

target_link_libraries(SMCP PRIVATE
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::Concurrent
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_calib3d
    opencv_features2d
    opencv_flann
    OpenGL::GL
    $<$<BOOL:${SMCP_WITH_SPINNAKER}>:Spinnaker::Spinnaker>
)

if(BUILD_TESTING)
    add_executable(SMCP_ai_tests
        tests/AiModelTests.cpp
        src/ai/WslModelService.cpp
        src/ai/WslModelService.h
    )
    target_include_directories(SMCP_ai_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_compile_definitions(SMCP_ai_tests PRIVATE UNICODE _UNICODE)
    target_compile_options(SMCP_ai_tests PRIVATE /W3 /utf-8)
    target_link_libraries(SMCP_ai_tests PRIVATE Qt5::Core)
    add_test(NAME ai_model_contract COMMAND SMCP_ai_tests)

    add_executable(SMCP_gp_tests
        tests/GPCompensationTests.cpp
        src/core/GPCompensation.cpp
        src/core/GPCompensation.h
    )
    target_include_directories(SMCP_gp_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_compile_definitions(SMCP_gp_tests PRIVATE NOMINMAX _USE_MATH_DEFINES)
    target_compile_options(SMCP_gp_tests PRIVATE /W3 /utf-8 /external:W0)
    target_link_libraries(SMCP_gp_tests PRIVATE opencv_core)
    add_test(NAME gp_compensation_matches_python
        COMMAND SMCP_gp_tests
                "${CMAKE_SOURCE_DIR}/tests/fixtures/modelo_gp_smcp.yml"
                "${CMAKE_SOURCE_DIR}/tests/fixtures/gp_reference.txt"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()

# ---------------------------------------------------------------------------
# compile_commands.json for clangd (Zed): adds /vctoolsdir and the Windows SDK to
# every command so clang-cl finds the STL outside the Visual Studio environment.
# ---------------------------------------------------------------------------
if(CMAKE_EXPORT_COMPILE_COMMANDS AND CMAKE_GENERATOR MATCHES "Ninja|Makefiles")
    # .../VC/Tools/MSVC/<ver>/bin/Hostx64/x64/cl.exe -> .../VC/Tools/MSVC/<ver>
    get_filename_component(_vctoolsdir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    foreach(_i RANGE 2)
        get_filename_component(_vctoolsdir "${_vctoolsdir}" DIRECTORY)
    endforeach()
    set(_winsdkdir "$ENV{WindowsSdkDir}")
    set(_winsdkver "$ENV{WindowsSDKVersion}")
    string(REGEX REPLACE "[\\/]+$" "" _winsdkdir "${_winsdkdir}")
    string(REGEX REPLACE "[\\/]+$" "" _winsdkver "${_winsdkver}")
    file(TO_CMAKE_PATH "${_winsdkdir}" _winsdkdir)

    add_custom_target(clangd_compile_commands ALL
        COMMAND "${CMAKE_COMMAND}"
                -DDB=${CMAKE_BINARY_DIR}/compile_commands.json
                -DVCTOOLSDIR=${_vctoolsdir}
                -DWINSDKDIR=${_winsdkdir}
                -DWINSDKVER=${_winsdkver}
                -P "${CMAKE_SOURCE_DIR}/cmake/PatchCompileCommands.cmake"
        COMMENT "Completing compile_commands.json for clangd"
        VERBATIM)
endif()

# ---------------------------------------------------------------------------
# Post-build: self-contained executable in build/<preset>/bin
# ---------------------------------------------------------------------------
if(SMCP_DEPLOY_RUNTIME)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${SMCP_QT_BIN_DIR}" REQUIRED)

    add_custom_command(TARGET SMCP POST_BUILD
        COMMAND "${WINDEPLOYQT_EXECUTABLE}"
                --no-compiler-runtime --no-translations --no-opengl-sw --no-system-d3d-compiler
                --no-angle --no-quick-import
                "$<TARGET_FILE:SMCP>"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "$<TARGET_FILE:opencv_core>"
                "$<TARGET_FILE:opencv_imgproc>"
                "$<TARGET_FILE:opencv_highgui>"
                "$<TARGET_FILE:opencv_calib3d>"
                "$<TARGET_FILE:opencv_features2d>"
                "$<TARGET_FILE:opencv_flann>"
                "$<TARGET_FILE_DIR:SMCP>"
        COMMENT "Deploying Qt and OpenCV next to $<TARGET_FILE_NAME:SMCP>"
        VERBATIM)

    if(SMCP_WITH_SPINNAKER)
        add_custom_command(TARGET SMCP POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "$<IF:$<CONFIG:Debug>,${Spinnaker_RUNTIME_DEBUG},${Spinnaker_RUNTIME_RELEASE}>"
                    "$<TARGET_FILE_DIR:SMCP>"
            COMMENT "Deploying Spinnaker next to $<TARGET_FILE_NAME:SMCP>"
            COMMAND_EXPAND_LISTS
            VERBATIM)
    endif()
endif()
