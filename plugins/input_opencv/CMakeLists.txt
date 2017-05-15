
# TODO: which components do I need?
# To fix the error: "undefined symbol: _ZN2cv12VideoCaptureC1Ev"
# find_package(OpenCV COMPONENTS core imgproc highgui videoio)
if (EXISTS /opt/vc/include)
    set(HAS_RASPI ON)
else()
    set(HAS_RASPI OFF)
endif()

find_package(OpenCV REQUIRED)


MJPG_STREAMER_PLUGIN_OPTION(input_opencv "OpenCV input plugin"
                            ONLYIF OpenCV_FOUND)

if (PLUGIN_INPUT_OPENCV)
    include_directories(${OpenCV_INCLUDE_DIRS})

    include_directories(/opt/vc/include)
    include_directories(/opt/vc/include/interface/vcos)
    include_directories(/opt/vc/include/interface/vcos/pthreads)
    include_directories(/opt/vc/include/interface/vmcs_host)
    include_directories(/opt/vc/include/interface/vmcs_host/linux)
    link_directories(/opt/vc/lib)

    MJPG_STREAMER_PLUGIN_COMPILE(input_opencv input_opencv.cpp)
    
    target_link_libraries(input_opencv ${OpenCV_LIBS} mmal_core mmal_util mmal_vc_client vcos bcm_host)
    
    add_subdirectory(filters/cvfilter_cpp)
    #add_subdirectory(filters/cvfilter_py)
    
endif()


