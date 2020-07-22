# CMake 最低版本要求
#cmake_minimum_required(VERSION 3.0 FATAL_ERROR)#new
cmake_minimum_required(VERSION 2.8.3) #old
project(ArmorDector)

# 库文件的放置目录
set(OpenCV_DIR /home/OpenCV/opencv-3.4.8/build)
#set(Torch_DIR /home/yeahoo/libtorch/share/cmake/Torch)#new
set(SDK_DIR /usr/lib/libgxiapi.so)

# 开启 DeBug 调试
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# C++11 标准、o3优化、多线程
set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS} -o3 -pthread")

#find_package(Torch REQUIRED)#new
find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS)) #old

include_directories("/usr/incude/eigen3")#new
include_directories(${OpenCV_INCLUDE_DIRS} )
#		    /home/yeahoo/libtorch/incldue) 
#		    /home/yeahoo/libtorch/include/torch/csrc/api/include)#new
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

set(CPP_SOURCES 
        src/ArmorDector.cpp 
        src/CRC_Check.cpp  
        src/DahengCamera.cpp
        src/ImageProcess.cpp
        src/KalmanFilter.cpp               
        src/main.cpp 
        src/Prediction.cpp
        src/Serial.cpp
    )

## 指定生成目标
add_executable(ArmorDector ${CPP_SOURCES})
#target_link_libraries(ArmorDector${OpenCV_LIBS}) #old
target_link_libraries(ArmorDector ${OpenCV_LIBS} ${SDK_DIR})#new
#target_link_libraries(ArmorDector ${OpenCV_LIBS} ${TORCH_LIBRARIES} ${SDK_DIR})#new
#set_property(TARGET ArmorDector PROPERTY CXX_STANDARD 11)#new
