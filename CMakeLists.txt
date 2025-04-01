cmake_minimum_required(VERSION 3.0.2)
project(minimum_snap)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )

# 查找 Eigen 库
find_package(Eigen3)
find_package(OsqpEigen REQUIRED)
if(TARGET OsqpEigen::OsqpEigen)
    message(STATUS "*** OsqpEigen found ***")
endif()


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES navit_interacter
        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node  src/qp_test_node.cpp src/qp_test.cpp)

target_link_libraries(${PROJECT_NAME}_node
        PUBLIC
        OsqpEigen::OsqpEigen
        ${catkin_LIBRARIES}
        )





