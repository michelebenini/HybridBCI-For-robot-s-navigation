cmake_minimum_required(VERSION 2.8.3)
project(hybrid_bci)

add_compile_options(-std=c++11 -lopencv_core)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure  
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  cv_bridge
  image_transport
  tf
  sensor_msgs
  vision_msgs
  rospy
  roscpp
  cnbiros_tobi_msgs
)

find_package(PkgConfig)
pkg_search_module(CNBILOOP REQUIRED cnbiloop)
pkg_search_module(TOBICORE REQUIRED tobicore)
pkg_search_module(TOBIID REQUIRED tobiid)
pkg_search_module(TOBIIC REQUIRED tobiic)
find_package(OpenCV REQUIRED)


add_message_files(DIRECTORY msg FILES P300.msg)
add_message_files(DIRECTORY msg FILES P300_person.msg)
add_message_files(DIRECTORY msg FILES motorimagery.msg)
add_message_files(DIRECTORY msg FILES motorimagery_raw.msg)
add_message_files(DIRECTORY msg FILES direction_distribution.msg)
add_message_files(DIRECTORY msg FILES P300_roi.msg)
add_message_files(DIRECTORY msg FILES person_roi.msg)
add_message_files(DIRECTORY msg FILES ClassificationResult.msg)
add_service_files(DIRECTORY srv FILES RegisterFace.srv)
add_service_files(DIRECTORY srv FILES ClassifyAll.srv)


add_message_files(
   FILES
   P300.msg
   P300_person.msg
   motorimagery.msg
   motorimagery_raw.msg
   P300_roi.msg
   person_roi.msg
   direction_distribution.msg
   ClassificationResult.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs sensor_msgs vision_msgs tfpose_ros face_classification
 )



 generate_dynamic_reconfigure_options(
    cfg/Parameters.cfg
 )

catkin_package(
  INCLUDE_DIRS 
    include
    ${CNBILOOP_INCLUDE_DIRS}
    ${TOBICORE_INCLUDE_DIRS}
    ${TOBIID_INCLUDE_DIRS}
    ${TOBIIC_INCLUDE_DIRS}
  LIBRARIES 
    ${CNBILOOP_LIBRARIES}
    ${TOBICORE_LIBRARIES}
    ${TOBIID_LIBRARIES}
    ${TOBIIC_LIBRARIES}
	    ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy cv_bridge image_transport sensor_msgs std_msgs 
  DEPENDS
    CNBILOOP
    TOBICORE
    TOBIID
    TOBIIC
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CNBILOOP_INCLUDE_DIRS} 
  ${TOBICORE_INCLUDE_DIRS} 
  ${TOBIID_INCLUDE_DIRS} 
  ${TOBIIC_INCLUDE_DIRS} 
)

add_executable(cmd_simu src/cmd_simu.cpp)
add_executable(hybrid_bci src/hybrid_bci.cpp)
add_executable(move src/move_bci.cpp)
add_executable(view_dist src/view_dist.cpp)
add_executable(p300_handler src/p300_handler.cpp)
add_executable(mi_handler src/mi_handler.cpp)

target_link_libraries(cmd_simu 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} )
target_link_libraries(hybrid_bci 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  )

target_link_libraries(move 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
target_link_libraries(view_dist 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
)
target_link_libraries(p300_handler 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${TOBICORE_LIBRARIES}
  ${TOBIID_LIBRARIES}
  ${TOBIIC_LIBRARIES}
)
target_link_libraries(mi_handler 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${CNBILOOP_LIBRARIES} 
  ${TOBICORE_LIBRARIES}
  ${TOBIID_LIBRARIES}
  ${TOBIIC_LIBRARIES}
)

add_dependencies(cmd_simu hybrid_bci_generate_messages_cpp)
add_dependencies(hybrid_bci hybrid_bci_generate_messages_cpp)
add_dependencies(move hybrid_bci_generate_messages_cpp)
add_dependencies(view_dist hybrid_bci_generate_messages_cpp)
add_dependencies(p300_handler hybrid_bci_generate_messages_cpp)
add_dependencies(mi_handler hybrid_bci_generate_messages_cpp)