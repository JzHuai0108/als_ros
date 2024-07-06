cmake_minimum_required(VERSION 3.0.2)
project(als_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  octomap_server
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES als
 CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf octomap_server
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_library(als_mcl src/MCL.cpp src/GLPoseSampler.cpp)
target_link_libraries(als_mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(mcl src/mcl_main.cpp)
target_link_libraries(mcl als_mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(mrf_failure_detector src/mrf_failure_detector.cpp)
target_link_libraries(mrf_failure_detector ${catkin_LIBRARIES})

add_executable(gl_pose_sampler src/gl_pose_sampler.cpp)
target_link_libraries(gl_pose_sampler als_mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(gl_pose_sampler_eval src/GLPoseSampler.cpp src/gl_pose_sampler_eval.cpp)
target_link_libraries(gl_pose_sampler_eval ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(classifier_dataset_generator src/classifier_dataset_generator.cpp)
target_link_libraries(classifier_dataset_generator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mae_classifier_learning src/mae_classifier_learning.cpp)
target_link_libraries(mae_classifier_learning ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(evaluator src/evaluator.cpp)
target_link_libraries(evaluator ${catkin_LIBRARIES})

add_executable(scan2pc src/scan2pc.cpp)
target_link_libraries(scan2pc ${catkin_LIBRARIES})

add_executable(slamer src/slamer.cpp)
target_link_libraries(slamer als_mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
    mcl mrf_failure_detector gl_pose_sampler classifier_dataset_generator mae_classifier_learning evaluator scan2pc slamer
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    als_mcl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/als_ros/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
