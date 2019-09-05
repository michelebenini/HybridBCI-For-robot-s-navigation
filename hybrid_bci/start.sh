#!/bin/bash
DirLoop="/home/michele/workspace/whitoolkit/data/P300/test"
DirCatkin="/home/michele/catkin_ws/"

Make="catkin_make"
ROSLaunch="roslaunch"
ROSrun="rosrun"
RunLoop="cl_runloop"
Hp300P="hybrid_p300_protocol"
HsmrP="hybrid_smr_protocol"


Command0="cl_runloop -d test.gdf -r"
Command1="roslaunch hybrid_bci roshybrid.launch"
Command2="roslaunch hybrid_bci simulation_world.launch"
Command3="rosrun open_face_recognition open_face_recognition_node.py"
Command4="rosrun face_classification face_classification_node.py"
Command5="rosrun hybrid_bci p300_handler"
Command6="rosrun hybrid_bci mi_handler"
Command7="rosrun hybrid_bci move"
Command8="rosrun hybrid_bci cmd_simu"
Command9="rosrun hybrid_bci hybrid_bci"
Command10="hybrid_p300_protocol -x hybrid_p300_protocol.xml"
Command11="hybrid_smr_protocol -x hybrid_smr_protocol.xml"


gnome-terminal --window \
    --working-directory=$DirLoop -e "cl_runloop -d test.gdf -r"\
    --tab --working-directory=$DirCatkin -e "roscore"

sleep 7

gnome-terminal --window \
    --working-directory=$DirCatkin -e "roslaunch hybrid_bci roshybrid.launch" \
    --tab --working-directory=$DirCatkin -e "roslaunch hybrid_bci simulation_world.launch" 

sleep 2

gnome-terminal --window \
    --working-directory=$DirCatkin -e "rosrun open_face_recognition open_face_recognition_node.py" \
    --tab --working-directory=$DirCatkin -e "rosrun face_classification face_classification_node.py" \
    --tab --working-directory=$DirCatkin -e "rosrun hybrid_bci move" \
    --tab --working-directory=$DirCatkin -e "rosrun hybrid_bci hybrid_bci" \
    --tab --active --working-directory=$DirCatkin -e "rosrun hybrid_bci cmd_simu"\
    --working-directory=$DirLoop -e "hybrid_p300_processing -x hybrid_p300_protocol.xml" \
    --tab --working-directory=$DirLoop -e "hybrid_smr_processing -x hybrid_smr_protocol.xml" 

# Handler has to execute from the user
#gnome-terminal --window \
#    --working-directory=$DirCatkin -e "rosrun hybrid_bci p300_handler" \
#    --tab --working-directory=$DirCatkin -e "rosrun hybrid_bci mi_handle" 