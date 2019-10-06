Hybrid Brain-Computer Interface for Robot's navigation

This folder contains file and data to run a system based on BCI to pilot a robots.
In the Documentaton folder you can find the full description of the system and some slides.
In the BCI Simulator folder you can find a simulator to learn how the system evolves.
In the hybrid bci folder you can find the ROS system
In the cnbi_wtkapp_bin_hybrid folder you can find the processing for the cnbi loop
In the CNBI protocols you can find the protocols for the cnbi loop


To use you must have in your ROS enviroment:
- cnbiros_bci
- cnbiros_tobi_msgs
- face_classification
- open_face_recognition
- tf-pose-estimation
- OpenCV

To use you must have in your wtkapp/bin/.. enviroment cnbi_wtkapp_bin_hybrid:
- hybrid_p300_processing.cpp
- hybrid_smr_processing.cpp
- p300_utilities.cpp
- smr-2class_utilities.cpp
- Adapted Makefile.am

To use you must have in your whitoolkit/data/.. enviroment test:
- a1_20190306_BLDA.p300.dat
- classifier_bhbf.dat
- test.gdf
- hybrid_p300_protocol.xml
- hybrid_smr_protocol.xml

To execute the system:

cd ~/catkin_ws/
catkin_make

cd ~/workspace/whitoolkit/data/test
cl_runloop -d test.gdf -r

roslaunch hybrid_bci face_detector.launch
roslaunch hybrid_bci hybrid_system.launch

PRESS p on hybrid_system to start moving the robot




