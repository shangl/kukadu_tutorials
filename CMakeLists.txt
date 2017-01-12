cmake_minimum_required(VERSION 2.8.3)
project(kukadu_tutorials)

find_package(catkin REQUIRED COMPONENTS
  kukadu
  actionlib_msgs
  control_msgs
  eigen_conversions
  geometry_msgs
  iis_robot_dep
  message_generation
  message_runtime
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_msgs
  tf_conversions
  trajectory_msgs
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# compile the core kukadu with c++11 if possible (required because the vision part crashes if compiled with c++11 due to PCL)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-DCPP11SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
    add_definitions(-DCPP11SUPPORTED)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(guided src/guided.cpp)
target_link_libraries(guided kukadu ${catkin_LIBRARIES} komo swift algo ann gui ors optim motion rt lapack blas glut GLU GL X11 ${GTK_LIBRARIES} ${GTK2_LIBRARIES} gthread-2.0 qhull m)

#add_executable(simple_planning src/simple_planning.cpp)
#target_link_libraries(simple_planning kukadu ${catkin_LIBRARIES})

add_executable(komo_planning src/komo_planning.cpp)
target_link_libraries(komo_planning kukadu ${catkin_LIBRARIES})

add_executable(datatransfer src/datatransfer.cpp)
target_link_libraries(datatransfer kukadu ${catkin_LIBRARIES})

add_executable(storage_singleton src/storage_singleton.cpp)
target_link_libraries(storage_singleton kukadu ${catkin_LIBRARIES})

add_executable(hand src/hand.cpp)
target_link_libraries(hand kukadu ${catkin_LIBRARIES})

#add_executable(moveit_planning src/moveit_planning.cpp)
#target_link_libraries(moveit_planning kukadu ${catkin_LIBRARIES})

#add_executable(cart_traj src/cartesian_trajectories.cpp)
#target_link_libraries(cart_traj kukadu ${catkin_LIBRARIES})

#add_executable(gaussian src/gaussianprocess.cpp)
#target_link_libraries(gaussian kukadu ${catkin_LIBRARIES})

add_executable(dmp src/dmp.cpp)
target_link_libraries(dmp kukadu ${catkin_LIBRARIES})

add_executable(database src/database.cpp)
target_link_libraries(database kukadu ${catkin_LIBRARIES})

add_executable(graphics src/graphics.cpp)
target_link_libraries(graphics kukadu ${catkin_LIBRARIES} kukadugui)

add_executable(artracker src/artracker.cpp)
target_link_libraries(artracker ${catkin_LIBRARIES} artoolkitplus kukadu)

add_executable(calibrator src/calibrator.cpp)
target_link_libraries(calibrator ${catkin_LIBRARIES} artoolkitplus kukadu)

add_executable(control_queue src/control_queue.cpp)
if(COMPILER_SUPPORTS_CXX11)
    SET_TARGET_PROPERTIES(control_queue PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(calibrator PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(graphics PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(artracker PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(database PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(komo_planning PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(datatransfer PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(storage_singleton PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(dmp PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(guided PROPERTIES COMPILE_FLAGS "-std=c++11")
    SET_TARGET_PROPERTIES(hand PROPERTIES COMPILE_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    SET_TARGET_PROPERTIES(control_queue PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(calibrator PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(graphics PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(artracker PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(database PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(komo_planning PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(datatransfer PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(storage_singleton PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(dmp PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(guided PROPERTIES COMPILE_FLAGS "-std=c++0x")
    SET_TARGET_PROPERTIES(hand PROPERTIES COMPILE_FLAGS "-std=c++0x")
endif()
target_link_libraries(control_queue kukadu ${catkin_LIBRARIES})

#add_dependencies(artoolkitplus ${catkin_EXPORTED_TARGETS} artoolkitplus_gencfg kukadu kukaduvision)
#add_dependencies(reflexxeslib ${catkin_EXPORTED_TARGETS})
add_dependencies(guided ${catkin_EXPORTED_TARGETS})
add_dependencies(hand ${catkin_EXPORTED_TARGETS})
add_dependencies(komo_planning ${catkin_EXPORTED_TARGETS})
add_dependencies(datatransfer ${catkin_EXPORTED_TARGETS})
add_dependencies(storage_singleton ${catkin_EXPORTED_TARGETS})
#add_dependencies(moveit_planning ${catkin_EXPORTED_TARGETS})
#add_dependencies(simple_planning ${catkin_EXPORTED_TARGETS})
add_dependencies(dmp ${catkin_EXPORTED_TARGETS})
#add_dependencies(gaussian ${catkin_EXPORTED_TARGETS})
add_dependencies(database ${catkin_EXPORTED_TARGETS})
add_dependencies(graphics ${catkin_EXPORTED_TARGETS} kukadugui)
add_dependencies(control_queue ${catkin_EXPORTED_TARGETS})
add_dependencies(artracker ${catkin_EXPORTED_TARGETS} artoolkitplus kukadu)
add_dependencies(calibrator ${catkin_EXPORTED_TARGETS} artoolkitplus kukadu)
