
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_SUBDIRECTORY( kbhit )



MJPG_STREAMER_PLUGIN_OPTION(cvfilter_cpp "OpenCV feilong filter")
MJPG_STREAMER_PLUGIN_COMPILE(cvfilter_cpp filter_cpp.cpp)
target_link_libraries(cvfilter_cpp ${OpenCV_LIBS} kbhit)

