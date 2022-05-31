cmake_minimum_required(VERSION 3.0.2)
project(wtr_serial)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

# 链接头文件
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(serial)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        wtr_serial_msg
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES obeject_detect
        CATKIN_DEPENDS wtr_serial_msg
        #  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
        ${serial_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(main1 src/qra_serial.cpp src/main1.cpp)
target_link_libraries(main1 ${THIRD_PARTY_LIBS})
