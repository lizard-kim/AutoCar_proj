cmake_minimum_required(VERSION 3.14)
project(laneDetection)

set(CMAKE_CXX_STANDARD 14)

# set dir for header and library
include_directories(/usr/local/Cellar/opencv@3/3.4.5_3/include)
link_directories(/usr/local/Cellar/opencv@3/3.4.5_3/lib)

# sources and libs
set(SOURCE_FILES main.cpp laneDetection.cpp)
set(EXT_LIBS opencv_stitching opencv_superres opencv_videostab opencv_aruco opencv_bgsegm opencv_bioinspired opencv_ccalib opencv_dnn_objdetect opencv_dpm opencv_face opencv_freetype opencv_fuzzy opencv_hfs opencv_img_hash opencv_line_descriptor opencv_optflow opencv_reg opencv_rgbd opencv_saliency opencv_sfm opencv_stereo opencv_structured_light opencv_phase_unwrapping opencv_surface_matching opencv_tracking opencv_datasets opencv_dnn opencv_plot opencv_xfeatures2d opencv_shape opencv_video opencv_ml opencv_ximgproc opencv_xobjdetect opencv_objdetect opencv_calib3d opencv_features2d opencv_highgui opencv_videoio opencv_imgcodecs opencv_flann opencv_xphoto opencv_photo opencv_imgproc opencv_core)
#set(SOURCE_FILES main.cpp PlateDetection.cpp PlateDetection.h)
#set(EXT_LIBS opencv_core opencv_imgcodecs opencv_highgui opencv_imgproc opencv_calib3d opencv_features2d)

# linking phase
add_executable(laneDetection ${SOURCE_FILES})
target_link_libraries(laneDetection ${EXT_LIBS})

