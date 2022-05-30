cmake_minimum_required(VERSION 3.0.2)
project(main_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE " -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        rospy
        geometry_msgs
        std_msgs
        wtr_serial
)

find_package(Eigen3 REQUIRED)


include_directories(
        /usr/include/eigen3/
        /usr/local/include/osqp/
        /usr/local/include/osqp-eigen/
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MathTools
        ${CMAKE_CURRENT_SOURCE_DIR}/include/StateWorkerInclude
        ${CMAKE_CURRENT_SOURCE_DIR}/include/reference


)

catkin_package(
        CATKIN_DEPENDS
        wtr_serial
)


add_executable(
        main_test
        src/demo.cpp
        )
target_link_libraries(
        main_test
        ${catkin_LIBRARIES}
)