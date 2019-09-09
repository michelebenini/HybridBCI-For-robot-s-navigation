To use you must have in your ros enviroment:
- cnbiros_bci
- cnbiros_tobi_msgs
- face_classification
- open_face_recognition
- tf-pose-estimation

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

roslaunch hybrid_bci roshybrid.launch

hybrid_p300_protocol -x hybrid_p300_protocol.xml
hybrid_smr_protocol -x hybrid_smr_protocol.xml

////////////// For test with simulator///////////////
roslaunch hybrid_bci simulation_world.launch
//////////////////////////////////////////////////////

rosrun open_face_recognition open_face_recognition_node.py
rosrun face_classification face_classification_node.py
rosrun hybrid_bci p300_handler
rosrun hybrid_bci mi_handler
rosrun hybrid_bci move
rosrun hybrid_bci cmd_simu
rosrun hybrid_bci hybrid_bci

PRESS p on cmd_simu to start moving the robot

TO SEE THE CURRENT DIRECTION DISTRIBUTION DIGIT:
rosrun hybrid_bci view_dist



